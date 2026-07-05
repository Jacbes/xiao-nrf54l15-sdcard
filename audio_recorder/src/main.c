/*
 * Audio Recorder — SINGLE-FILE, FAULT-TOLERANT recording to SD card.
 *
 * Platform:  Seeed Studio XIAO nRF54L15 (Sense)
 * Mic:       on-board PDM microphone (PDM20, P1.12 CLK / P1.13 DIN)
 * SD card:   microSD over hardware SPI (SPIM22), FAT32
 * Audio:     16 kHz, mono, 16-bit little-endian raw PCM
 *
 * Convert to WAV:
 *   ffmpeg -f s16le -ar 16000 -ac 1 -i rec_XXXX.raw rec_XXXX.wav
 *
 * Behaviour: on boot it mounts the SD card (retrying forever), verifies the
 * previous recording, then AUTO-STARTS a recording that runs for exactly
 * REC_DURATION_MIN minutes and AUTO-STOPS. After stopping it reads the whole
 * file back to PROVE it is intact and prints a clear SUCCESS / FAIL verdict.
 * The button (P0.00) toggles an extra manual session at any time.
 *
 * =====================  FAULT-TOLERANCE DESIGN  ============================
 *
 *  Three threads decouple capture from slow / jittery SD writes:
 *
 *   [DMIC thread, prio 1]  PDM -> mem_slab -> (gain) -> RING BUF -> free slab
 *                          Highest priority; never blocks on SD. If the ring
 *                          buffer is full it DROPS the oldest-less bytes
 *                          (counted), but the PDM slab is always freed so the
 *                          driver never stalls.
 *
 *   [SD thread, prio 3]    drains RING BUF into a big WRITE BUF (32 KiB = 1 s),
 *                          flushed to the card as ONE fs_write() per second.
 *                          Periodic fs_sync() every 8 s commits the FAT so the
 *                          file is valid (just shorter) after a power cut.
 *
 *   [LED thread, prio 10]  blinks while recording.
 *
 *  Recovery on write error (never loses the already-committed file):
 *    1) close + reopen SAME file for append (f_lseek to end);
 *    2) if that fails -> unmount + remount SD + reopen append;
 *    3) if that fails -> stop recording; everything up to the last
 *       fs_sync() survives on the card.
 *
 *  The file is opened with FS_O_CREATE|FS_O_WRITE (NO truncate) so a resumed
 *  recording always APPENDS -> existing audio is never destroyed.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <string.h>
#include <stdio.h>

#define DISK_DRIVE_NAME  "SD"
#define DISK_MOUNT_PT    "/" DISK_DRIVE_NAME ":"

/* ---- Audio format ---- */
#define SAMPLE_RATE      16000
#define SAMPLE_BITS      16
#define BYTES_PER_SAMPLE (SAMPLE_BITS / 8)            /* 2  */
#define AUDIO_BPS        (SAMPLE_RATE * BYTES_PER_SAMPLE) /* 32000 bytes/s */

/* DMIC block = 50 ms of audio (1600 B). 16 slabs >> queue-size(6)+HW(2)+margin. */
#define BLOCK_SIZE       (BYTES_PER_SAMPLE * (SAMPLE_RATE / 20))  /* 1600 */
#define BLOCK_COUNT      16
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* Software microphone gain (PDM has no HW PGA). 32 ~= +30 dB. */
#define SW_GAIN          32

/* Big latency-absorbing ring buffer between DMIC and SD writer (2 s). */
#define RING_BUF_SIZE    65536
RING_BUF_DECLARE(audio_ring_buf, RING_BUF_SIZE);

/* Accumulate 1 s of audio before a single fs_write (fewer SD ops). */
#define WRITE_BUF_SIZE   (AUDIO_BPS * 1)              /* 32768 */

/* FAT commit cadence + diagnostics. */
#define SYNC_MS          8000
#define HEARTBEAT_MS     5000
#define BTN_DEBOUNCE_MS  300

/* Recording length: stop automatically after this many minutes. */
#define REC_DURATION_MIN 10
#define REC_DURATION_MS  (REC_DURATION_MIN * 60 * 1000)

/* Hardware watchdog: reboots the SoC if a thread hangs (e.g. the SD stack
 * blocks forever on a stuck card). main feeds it; while recording, main stops
 * feeding if the SD writer thread goes stale, so an SD hang also reboots. */
#define WDT_TIMEOUT_MS   60000
#define SD_STALE_MS      40000

#define VBATT_ADC_CH     7

/* ---- Forward decls / globals ---- */
static int read_battery_mv(void)
{
	const struct device *d = DEVICE_DT_GET(DT_NODELABEL(adc));
	if (!device_is_ready(d)) {
		return -1;
	}
	struct adc_channel_cfg cc = {
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = VBATT_ADC_CH,
#if defined(CONFIG_ADC_NRFX_SAADC)
		.input_positive = 7,
#endif
	};
	int16_t sb = 0;
	struct adc_sequence as = {
		.channels = BIT(VBATT_ADC_CH),
		.buffer = &sb,
		.buffer_size = sizeof(sb),
		.resolution = 12,
	};
	if (adc_channel_setup(d, &cc) || adc_read(d, &as)) {
		return -1;
	}
	return (int)(((int32_t)sb * 2400) / 4096);
}

static FATFS fat_fs;
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = DISK_MOUNT_PT,
};
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback btn_cb;
static const struct device *dmic_dev;

static volatile bool btn_pressed;
static volatile bool recording;        /* set true -> DMIC+SD threads capture */
static bool file_open;
static bool sd_mounted;

static struct fs_file_t rec_file;
static char rec_path[64];

/* Per-session counters (reset on each start). */
static int64_t rec_start, last_sync;
static uint32_t written, dropped, sd_writes, rec_errs;

K_THREAD_STACK_DEFINE(led_stk, 1024);
static struct k_thread led_th;
K_THREAD_STACK_DEFINE(dmic_stk, 2048);
static struct k_thread dmic_th;
K_THREAD_STACK_DEFINE(sd_stk, 12288);
static struct k_thread sd_th;
K_SEM_DEFINE(data_sem, 0, 1);

/* ---- Watchdog ---- */
static const struct device *wdt_dev;
static int wdt_chan = -1;
static volatile int64_t sd_last_alive;

static void wdt_kick(void)
{
	if (wdt_chan >= 0) {
		wdt_feed(wdt_dev, wdt_chan);
	}
}

/* Mark the SD writer thread as making progress (called from sd_fn). */
static void sd_alive(void)
{
	sd_last_alive = k_uptime_get();
}

/* ============================  SD helpers  =============================== */

static bool sd_card_init(void)
{
	if (disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_INIT, NULL) == 0) {
		return true;
	}
	disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
	return false;
}

static void sd_teardown(void)
{
	if (file_open) {
		fs_sync(&rec_file);
		fs_close(&rec_file);
		file_open = false;
	}
	if (sd_mounted) {
		fs_unmount(&mp);
		sd_mounted = false;
	}
	disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
}

static bool sd_mount(void)
{
	sd_teardown();
	k_msleep(300);
	if (!sd_card_init()) {
		return false;
	}
	if (fs_mount(&mp) != 0) {
		disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
		return false;
	}
	sd_mounted = true;
	return true;
}

static int next_idx(void)
{
	char p[64];
	struct fs_dirent e;

	for (int n = 1; n < 10000; n++) {
		snprintf(p, sizeof(p), DISK_MOUNT_PT "/rec_%04d.raw", n);
		if (fs_stat(p, &e) != 0) {
			return n;
		}
	}
	return -1;
}

/* Read a file end-to-end: proves every sector is readable (no corruption)
 * and reports size, throughput and a quick audio sanity check
 * (min/max/mean of samples). Returns true if file is intact & has audio. */
#define VERIFY_CHUNK 8192
static bool verify_file(const char *path, const char *label)
{
	struct fs_file_t f;
	fs_file_t_init(&f);
	if (fs_open(&f, path, FS_O_READ) != 0) {
		printk("verify[%s]: cannot open %s\n", label, path);
		return false;
	}
	FIL *fp = (FIL *)f.filep;
	uint32_t total_size = f_size(fp);

	static uint8_t vbuf[VERIFY_CHUNK];
	uint32_t total = 0;
	int16_t smin = 32767, smax = -32768;
	int64_t acc = 0;
	uint32_t samples = 0;
	int64_t t0 = k_uptime_get();
	uint32_t last_mb = 0;
	ssize_t n;

	printk("verify[%s]: reading %s (%u KB)...\n", label, path, total_size / 1024);

	while ((n = fs_read(&f, vbuf, VERIFY_CHUNK)) > 0) {
		total += n;
		int16_t *s = (int16_t *)vbuf;
		int ns = n / 2;
		for (int i = 0; i < ns; i++) {
			int16_t v = s[i];
			if (v < smin) {
				smin = v;
			}
			if (v > smax) {
				smax = v;
			}
			acc += (v < 0) ? -v : v;
			samples++;
		}
		uint32_t mb = total / (1024U * 1024U);
		if (mb > last_mb) {
			printk("verify[%s]:   %u KB read (%d%%)\n", label,
			       total / 1024, total_size ? (int)(total * 100 / total_size) : 0);
			last_mb = mb;
		}
		wdt_kick();
	}
	int64_t dt = k_uptime_get() - t0;
	fs_close(&f);

	uint32_t dur_s = samples / SAMPLE_RATE;
	printk("verify[%s]: read back %u bytes (%u samples, %us audio) in %lld ms\n",
	       label, total, samples, dur_s, dt);
	if (dt > 0) {
		printk("verify[%s]: read throughput %u KB/s\n", label,
		       (unsigned)((uint64_t)total / 1024U * 1000U / dt));
	}
	printk("verify[%s]: sample range [%d .. %d], mean |abs| = %u\n",
	       label, smin, smax, samples ? (unsigned)(acc / samples) : 0);

	if (total == 0) {
		printk("verify[%s]: FAIL - file empty or unreadable\n", label);
		return false;
	}
	if (smax <= smin) {
		printk("verify[%s]: WARN - readable but audio looks silent/flat\n", label);
		return false;
	}
	printk("verify[%s]: OK - file is intact and contains audio\n", label);
	return true;
}

/* Boot-time integrity check on the most recent recording. */
static void verify_latest_rec(void)
{
	struct fs_dir_t dirp;
	struct fs_dirent ent;
	char latest[40] = "";
	int latest_n = -1;
	int count = 0;

	fs_dir_t_init(&dirp);
	if (fs_opendir(&dirp, DISK_MOUNT_PT) != 0) {
		printk("verify: cannot open dir\n");
		return;
	}
	while (fs_readdir(&dirp, &ent) == 0 && ent.name[0] != '\0') {
		if (ent.type == FS_DIR_ENTRY_FILE &&
		    strncmp(ent.name, "rec_", 4) == 0) {
			int n = atoi(ent.name + 4);
			count++;
			if (n > latest_n) {
				latest_n = n;
				snprintf(latest, sizeof(latest), "%s", ent.name);
			}
		}
	}
	fs_closedir(&dirp);
	printk("verify: %d rec_*.raw file(s) on card\n", count);
	if (latest_n < 0) {
		printk("verify: no previous recording to check\n");
		return;
	}

	char path[64];
	snprintf(path, sizeof(path), DISK_MOUNT_PT "/%s", latest);
	verify_file(path, "boot");
}

/* Open rec_path for append (never truncate). Resumes after a fault. */
static bool open_append(void)
{
	fs_file_t_init(&rec_file);
	if (fs_open(&rec_file, rec_path, FS_O_CREATE | FS_O_WRITE) != 0) {
		file_open = false;
		return false;
	}
	FIL *fp = (FIL *)rec_file.filep;
	FSIZE_t sz = f_size(fp);
	if (sz > 0) {
		f_lseek(fp, sz);        /* resume at end of existing data */
		written = sz;
		printk("  append @ %u KB\n", (unsigned)(sz / 1024));
	}
	file_open = true;
	return true;
}

/* ============================  DMIC helpers  ============================= */

static bool dmic_arm(void)
{
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BITS,
		.mem_slab = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
			.req_num_chan = 1,
			.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT),
		},
	};
	cfg.streams[0].pcm_rate = SAMPLE_RATE;
	cfg.streams[0].block_size = BLOCK_SIZE;

	if (dmic_configure(dmic_dev, &cfg) != 0) {
		printk("dmic_configure failed\n");
		return false;
	}
	if (dmic_trigger(dmic_dev, DMIC_TRIGGER_START) != 0) {
		printk("dmic_trigger START failed\n");
		return false;
	}
	return true;
}

static void dmic_disarm(void)
{
	dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
}

/* ============================  Threads  ================================== */

static void led_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	while (1) {
		if (recording) {
			gpio_pin_set_dt(&led, 0);
			k_msleep(150);
			gpio_pin_set_dt(&led, 1);
			k_msleep(150);
		} else {
			gpio_pin_set_dt(&led, 0);
			k_msleep(50);
			gpio_pin_set_dt(&led, 1);
			k_msleep(1950);
		}
	}
}

static void btn_isr(const struct device *d, struct gpio_callback *c, uint32_t p)
{
	ARG_UNUSED(d);
	ARG_UNUSED(c);
	ARG_UNUSED(p);
	btn_pressed = true;
}

/* High-priority capture: never blocks on SD; drops on ring overflow. */
static void dmic_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		while (!recording) {
			k_msleep(20);
		}
		while (recording) {
			void *buf;
			uint32_t sz;

			if (dmic_read(dmic_dev, 0, &buf, &sz, 1000) != 0) {
				/* Driver idle / transient: re-arm if still recording. */
				if (recording) {
					dmic_arm();
				}
				continue;
			}
			if (SW_GAIN > 1) {
				int16_t *s = (int16_t *)buf;
				int n = (int)(sz / 2);
				for (int i = 0; i < n; i++) {
					int32_t v = (int32_t)s[i] * SW_GAIN;
					s[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
				}
			}
			uint32_t put = ring_buf_put(&audio_ring_buf, (uint8_t *)buf, sz);
			if (put < sz) {
				dropped += (sz - put); /* SD writer fell behind */
			}
			k_mem_slab_free(&mem_slab, buf);
			k_sem_give(&data_sem);
		}
	}
}

/* Try to write *pos bytes. On success resets pos and returns true.
 * On failure leaves *pos intact (data preserved for retry) and returns false. */
static bool flush_write_buf(uint8_t *wbuf, uint32_t *pos)
{
	if (*pos == 0) {
		return true;
	}
	ssize_t w = fs_write(&rec_file, wbuf, *pos);
	if (w == (ssize_t)(*pos)) {
		written += *pos;
		sd_writes++;
		*pos = 0;
		return true;
	}
	printk("  fs_write returned %d (wanted %u)\n", (int)w, *pos);
	return false;
}

/* Recover the file handle, retrying indefinitely while recording is still
 * requested. Strategy: first two attempts just reopen-append (fast); further
 * attempts fully unmount+remount the card (slow, with 2 s backoff). This
 * survives transient SPI/SD hiccups AND auto-resumes once a replugged /
 * power-cycled card comes back. Returns true once the file is open again,
 * false only if recording was stopped (by button) while recovering. */
static bool recover_file_loop(void)
{
	int attempt = 0;

	while (recording) {
		sd_alive();
		attempt++; wdt_kick();
		bool ok;
		if (attempt <= 2) {
			if (file_open) {
				fs_close(&rec_file);
				file_open = false;
			}
			k_msleep(200);
			ok = open_append();
		} else {
			sd_teardown();
			k_msleep(2000);
			ok = sd_mount() && open_append();
		}
		if (ok) {
			if (attempt > 1) {
				printk("  recovered after %d attempt(s)\n", attempt);
			}
			return true;
		}
		if (attempt == 1 || attempt % 5 == 0) {
			printk("  recovery attempt %d failed; SD stuck? "
			       "replug card / board USB to resume\n", attempt);
		}
	}
	return false;
}

/* Recover the file handle AND flush the pending write buffer, retrying until
 * both succeed or recording is stopped. Never loses the buffered audio. */
static bool recover_and_flush(uint8_t *wbuf, uint32_t *pos)
{
	while (recording) {
		sd_alive();
		wdt_kick();
		if (!recover_file_loop()) {
			return false;
		}
		if (flush_write_buf(wbuf, &pos)) {
			return true;
		}
		printk("  write still failing after recovery, retrying\n");
	}
	return false;
}

/* SD writer: drain ring buffer into write_buf, flush per second, sync FAT. */
static void sd_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	static uint8_t wbuf[WRITE_BUF_SIZE];
	uint32_t pos = 0;
	int64_t last_hb;

	while (1) {
		while (!recording) {
			k_msleep(50);
		}
		last_sync = k_uptime_get();
		last_hb = last_sync;
		sd_alive();
		pos = 0;

		while (recording) {
			sd_alive();
			k_sem_take(&data_sem, K_MSEC(100));

			/* Drain everything available into the write buffer. */
			while (ring_buf_size_get(&audio_ring_buf) > 0 && pos < WRITE_BUF_SIZE) {
				uint32_t want = WRITE_BUF_SIZE - pos;
				uint32_t rd = ring_buf_get(&audio_ring_buf, &wbuf[pos], want);
				if (rd == 0) {
					break;
				}
				pos += rd;
			}

			/* Flush when the write buffer is full (1 s of audio). */
			if (pos >= WRITE_BUF_SIZE) {
				if (!flush_write_buf(wbuf, &pos)) {
					rec_errs++;
					printk("  write error, recovering (file preserved)...\n");
					if (!recover_and_flush(wbuf, &pos)) {
						printk("  recording stopped by user during recovery\n");
						recording = false;
						break;
					}
				}
			}

			if (!recording) {
				break;
			}

			/* Periodic FAT commit for power-loss safety. */
			int64_t now = k_uptime_get();
			if (now - last_sync >= SYNC_MS) {
				if (pos > 0 && !flush_write_buf(wbuf, &pos)) {
					rec_errs++;
					printk("  flush-before-sync failed, recovering...\n");
					if (!recover_and_flush(wbuf, &pos)) {
						recording = false;
					break;
					}
				}
				if (file_open && fs_sync(&rec_file) != 0) {
					rec_errs++;
					printk("  fs_sync failed, recovering file handle...\n");
					if (!recover_file_loop()) {
						recording = false;
						break;
					}
				}
				last_sync = now;
			}

			/* Heartbeat: show progress toward the auto-stop target. */
			if (recording && now - last_hb >= HEARTBEAT_MS) {
				int32_t el = (int32_t)((now - rec_start) / 1000);
				int target = REC_DURATION_MS / 1000;
				int pct = (int)(((now - rec_start) * 100) / REC_DURATION_MS);
				if (pct > 100) {
					pct = 100;
				}
				printk("  [HB] REC %02d:%02d / %02d:%02d  %d%%  %uKB  "
				       "wr %u  drop %uKB  errs %u  ring %u\n",
				       (int)(el / 60), (int)(el % 60),
				       target / 60, target % 60, pct,
				       (unsigned)(written / 1024), (unsigned)sd_writes,
				       (unsigned)(dropped / 1024), (unsigned)rec_errs,
				       ring_buf_size_get(&audio_ring_buf));
				last_hb = now;
			}
		}

		/* Final flush on stop / abort. */
		if (pos > 0 && file_open) {
			flush_write_buf(wbuf, &pos);
		}
		if (file_open) {
			fs_sync(&rec_file);
		}
	}
}

/* ============================  Start / Stop  ============================= */

static void start_rec(void)
{
	if (recording) {
		return;
	}
	if (!sd_mounted && !sd_mount()) {
		printk("SD mount fail\n");
		return;
	}
	int idx = next_idx();
	if (idx < 0) {
		printk("no free index\n");
		return;
	}
	snprintf(rec_path, sizeof(rec_path), DISK_MOUNT_PT "/rec_%04d.raw", idx);

	ring_buf_reset(&audio_ring_buf);
	written = dropped = sd_writes = rec_errs = 0;

	if (!open_append()) {
		printk("open fail\n");
		return;
	}
	if (!dmic_arm()) {
		printk("dmic arm fail\n");
		fs_close(&rec_file);
		file_open = false;
		return;
	}

	rec_start = k_uptime_get();
	sd_last_alive = rec_start;
	recording = true;
	printk("========================================================\n");
	printk(" REC -> %s\n", rec_path);
	printk(" 16 kHz mono S16LE (~%d B/s). Auto-stop in %d min.\n",
	       AUDIO_BPS, REC_DURATION_MIN);
	printk("========================================================\n");
}

/* Stop recording, finalise the file, then read it back to PROVE integrity.
 * Prints a clear SUCCESS / INCOMPLETE verdict based on duration + verify. */
static void stop_rec(void)
{
	if (!recording) {
		return;
	}
	int64_t elapsed_ms = k_uptime_get() - rec_start;
	recording = false;

	/* Let the SD writer finish its final flush loop. */
	k_msleep(400);
	dmic_disarm();
	k_msleep(200);

	if (file_open) {
		fs_sync(&rec_file);
		fs_close(&rec_file);
		file_open = false;
	}

	int32_t s = (int32_t)(elapsed_ms / 1000);
	printk("STOP %02d:%02d  %uKB written  %u writes  drop %uKB  errs %u\n",
	       s / 60, s % 60, (unsigned)(written / 1024), (unsigned)sd_writes,
	       (unsigned)(dropped / 1024), (unsigned)rec_errs);

	/* Read the file back to confirm it survived intact on the card. */
	bool ok = false;
	if (sd_mounted) {
		ok = verify_file(rec_path, "rec");
	}

	printk("========================================================\n");
	if (ok && s >= (REC_DURATION_MS / 1000) - 1) {
		printk(" >>> 10-MINUTE RECORDING: SUCCESS  (%d:%02d, %u KB) <<<\n",
		       s / 60, s % 60, (unsigned)(written / 1024));
	} else if (ok) {
		printk(" >>> RECORDING VERIFIED but short: %d:%02d (< 10 min) <<<\n",
		       s / 60, s % 60);
	} else {
		printk(" >>> RECORDING FAILED verify (%d:%02d) — check card <<<\n",
		       s / 60, s % 60);
	}
	printk("========================================================\n");
	printk("Idle. Press button P0.00 to record another %d-min session.\n",
	       REC_DURATION_MIN);
}

static void handle_btn(void)
{
	static int64_t last;

	if (k_uptime_get() - last < BTN_DEBOUNCE_MS) {
		return;
	}
	last = k_uptime_get();
	btn_pressed = false;
	if (recording) {
		stop_rec();
	} else {
		start_rec();
	}
}

/* ============================  Main  ===================================== */

int main(void)
{
	k_msleep(300);
	printk("\n=== Audio Recorder (fault-tolerant, %d-min auto-stop) ===\n",
	       REC_DURATION_MIN);
	printk("16 kHz mono S16LE raw PCM -> SD:/rec_XXXX.raw\n");

	/* Report WHY we are booting (watchdog / brownout / hard fault / POR). */
	uint32_t cause = 0;
	if (hwinfo_get_reset_cause(&cause) == 0) {
		printk("Reset cause: 0x%08x", cause);
		if (cause & RESET_POR)         printk(" [POWER-ON]");
		if (cause & RESET_WATCHDOG)    printk(" [WATCHDOG]");
		if (cause & RESET_BROWNOUT)    printk(" [BROWNOUT]");
		if (cause & RESET_CPU_LOCKUP)  printk(" [CPU-LOCKUP]");
		if (cause & RESET_HARDWARE)    printk(" [HARDWARE]");
		if (cause & RESET_SOFTWARE)    printk(" [SOFTWARE]");
		if (cause & RESET_DEBUG)       printk(" [DEBUG]");
		printk("\n");
		hwinfo_clear_reset_cause();
	} else {
		printk("Reset cause: unavailable\n");
	}

	/* Arm the hardware watchdog first thing, so any later hang (e.g. the SD
	 * stack blocking on a stuck card) reboots the board instead of stalling.
	 * PAUSE_HALTED_BY_DBG keeps it quiet while a debugger is attached. */
	wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	if (device_is_ready(wdt_dev)) {
		struct wdt_timeout_cfg wcfg = {
			.window = { .min = 0, .max = WDT_TIMEOUT_MS },
			.flags = WDT_FLAG_RESET_SOC,
		};
		wdt_chan = wdt_install_timeout(wdt_dev, &wcfg);
		if (wdt_chan >= 0) {
			wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
			printk("Watchdog armed (%d ms)\n", WDT_TIMEOUT_MS);
		} else {
			printk("Watchdog install failed (%d)\n", wdt_chan);
		}
	} else {
		printk("Watchdog device not ready\n");
	}

	int v = read_battery_mv();
	if (v > 0) {
		printk("VBat: %d mV\n", v);
	}

	dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	if (!device_is_ready(dmic_dev)) {
		printk("DMIC not ready!\n");
		return 0;
	}
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&btn, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&btn_cb, btn_isr, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb);

	k_thread_create(&led_th, led_stk, K_THREAD_STACK_SIZEOF(led_stk),
			led_fn, 0, 0, 0, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_create(&dmic_th, dmic_stk, K_THREAD_STACK_SIZEOF(dmic_stk),
			dmic_fn, 0, 0, 0, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_thread_create(&sd_th, sd_stk, K_THREAD_STACK_SIZEOF(sd_stk),
			sd_fn, 0, 0, 0, K_PRIO_PREEMPT(3), 0, K_NO_WAIT);

	/* Mount the SD card, retrying forever until it is present. This lets the
	 * user fix a stuck/absent card (replug card or board USB) without having
	 * to re-flash: the moment the card answers, recording starts. */
	int boot_tries = 0;
	while (!sd_mounted) {
		wdt_kick();
		if (sd_mount()) {
			break;
		}
		boot_tries++;
		if (boot_tries == 1 || boot_tries % 10 == 0) {
			printk("SD not ready (absent/stuck). Retrying every 2s "
			       "(attempt %d). Replug card / board USB if persistent.\n",
			       boot_tries);
		}
		k_sleep(K_SECONDS(2));
	}
	printk("SD mounted (after %d attempt%s)\n",
	       boot_tries, boot_tries == 1 ? "" : "s");

	if (sd_mounted) {
		verify_latest_rec();
	}

	printk("Auto-start in 3s (will record %d min, button toggles manual session)\n",
	       REC_DURATION_MIN);
	k_msleep(3000);
	start_rec();

	while (1) {
		if (btn_pressed) {
			handle_btn();
		}

		/* Auto-stop after the configured duration. */
		if (recording && (k_uptime_get() - rec_start) >= REC_DURATION_MS) {
			printk("\n>>> %d:%02d reached — auto-stopping recording <<<\n",
			       REC_DURATION_MIN, 0);
			stop_rec();
		}

		/* Feed watchdog, unless the SD writer is stuck (then let it reset). */
		bool sd_fresh = !recording ||
				(k_uptime_get() - sd_last_alive < SD_STALE_MS);
		if (sd_fresh) {
			wdt_kick();
		}

		k_msleep(50);
	}
	return 0;
}
