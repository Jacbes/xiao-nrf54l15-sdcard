/*
 * Audio Recorder -- button-controlled, fault-tolerant, ultra-low-power.
 *
 * Platform:  Seeed Studio XIAO nRF54L15
 * Mic:       on-board PDM (PDM20, P1.12 CLK / P1.13 DIN)
 * SD card:   microSD over SPI (SPIM22), FAT32
 * Audio:     16 kHz, mono, 16-bit S16LE raw PCM
 *
 * Behaviour:
 *   Boot       -> LED solid ON, idle, waiting for button.
 *   Button #1  -> start recording, LED blinks.
 *   Button #2  -> stop recording, LED solid ON, idle.
 *   10 s idle  -> system OFF (~1 µA), wake on button press.
 *
 * Fault tolerance:
 *   Three threads (dmic / sd / led). SD write errors trigger recovery
 *   (reopen -> remount -> exponential backoff). Persistent unrecoverable
 *   failure stops feeding the watchdog -> hardware reboot.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>
#include <ff.h>
#include <opus.h>
#include "ble_audio.h"
#include <string.h>
#include <stdio.h>
#if defined(CONFIG_RAM_POWER_DOWN_LIBRARY)
#include <ram_pwrdn.h>
#endif

/* Set to 1 for Opus evaluation: auto-record 30 s on boot. */
#define TEST_OPUS 0

/* =============================  Constants  ============================== */

#define DISK_DRIVE_NAME  "SD"
#define DISK_MOUNT_PT    "/" DISK_DRIVE_NAME ":"

/* Audio format */
#define SAMPLE_RATE      16000
#define SAMPLE_BITS      16
#define BYTES_PER_SAMPLE (SAMPLE_BITS / 8)
#define AUDIO_BPS        (SAMPLE_RATE * BYTES_PER_SAMPLE)  /* 32000 */

/* PDM capture: 50 ms blocks, 16 slabs (>> queue-size 6 + HW 2 + margin) */
#define BLOCK_SIZE       (BYTES_PER_SAMPLE * (SAMPLE_RATE / 20))  /* 1600 */
#define BLOCK_COUNT      16
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* Software gain (PDM has no HW PGA). 32 ~ +30 dB with soft-knee limiter. */
#define SW_GAIN          32

/* Ring buffer: 0.5 s of latency absorption between DMIC and SD writer. */
#define RING_BUF_SIZE    16384
RING_BUF_DECLARE(audio_ring_buf, RING_BUF_SIZE);

/* Write buffer: flush 0.5 s of audio per fs_write (8 KiB). */
#define WRITE_BUF_SIZE   (AUDIO_BPS / 4)

/* Timing */
#define SYNC_MS          8000    /* FAT commit interval */
#define IDLE_SLEEP_MS    10000   /* 10 s idle -> system OFF */
#define BTN_DEBOUNCE_MS  300
#define BTN_SHORT_MS     2000    /* click held < 2 s  -> record toggle */
#define BTN_LONG_MS      5000    /* hold  >= 5 s      -> pairing mode */
#define BATT_UPDATE_MS   5000    /* BLE battery refresh interval */

/* Watchdog: reboot if SD stack hangs. */
#define WDT_TIMEOUT_MS   60000
#define SD_STALE_MS      40000
#define RECOVERY_TIMEOUT_MS  (2 * SD_STALE_MS)  /* 80 s */

/* =============================  Globals  ================================ */

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
static volatile bool recording;
static volatile bool going_to_sleep;
static bool file_open;
static bool sd_mounted;

static struct fs_file_t rec_file;
static char rec_path[64];

static int64_t rec_start, last_sync, last_activity, last_batt;
static uint32_t written, dropped, sd_writes, rec_errs;

K_THREAD_STACK_DEFINE(led_stk, 512);
static struct k_thread led_th;
K_THREAD_STACK_DEFINE(dmic_stk, 2048);
static struct k_thread dmic_th;
K_THREAD_STACK_DEFINE(sd_stk, 24576);
static struct k_thread sd_th;
K_SEM_DEFINE(data_sem, 0, 1);

/* Opus encoder */
#define OPUS_FRAME_MS      20
#define OPUS_FRAME_SAMPLES (SAMPLE_RATE * OPUS_FRAME_MS / 1000)  /* 320 */
#define OPUS_FRAME_BYTES   (OPUS_FRAME_SAMPLES * BYTES_PER_SAMPLE)  /* 640 */
#define OPUS_BITRATE       24000
#define OPUS_MAX_PKT       4000

/* Encoder state buffer (static, avoids malloc).  opus_encoder_get_size(1)
 * is ~22 KiB for fixed-point 16 kHz mono -- 28 KiB gives headroom. */
static uint8_t  opus_enc_mem[24576] __aligned(4);
static OpusEncoder *opus_enc;
static uint8_t  opus_pcm_acc[OPUS_FRAME_BYTES];
static uint32_t opus_pcm_pos;
static uint8_t  opus_pkt[OPUS_MAX_PKT];
static uint32_t raw_pcm_bytes;   /* raw bytes that would have been written */
static uint32_t enc_bytes;        /* actual encoded bytes written */

/* BLE packet queue: SD thread enqueues opus packets, a dedicated BLE
 * thread dequeues and sends notifications. This keeps BLE stack usage
 * out of the SD thread's stack (which must host opus VLAs). */
struct ble_pkt {
	uint16_t len;
	uint8_t  data[80];  /* 24 kbps / 20 ms -> ~60 bytes max */
};
K_MSGQ_DEFINE(ble_pkt_q, sizeof(struct ble_pkt), 4, 4);
K_THREAD_STACK_DEFINE(ble_stk, 2048);
static struct k_thread ble_th;

static bool opus_enc_init(void)
{
	opus_enc = (OpusEncoder *)opus_enc_mem;
	int rc = opus_encoder_init(opus_enc, SAMPLE_RATE, 1,
				  OPUS_APPLICATION_VOIP);
	if (rc != OPUS_OK) {
		printk("opus init fail %d\n", rc);
		return false;
	}
	opus_encoder_ctl(opus_enc, OPUS_SET_BITRATE(OPUS_BITRATE));
	opus_encoder_ctl(opus_enc, OPUS_SET_COMPLEXITY(3));
	opus_encoder_ctl(opus_enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
	opus_encoder_ctl(opus_enc, OPUS_SET_DTX(1));
	opus_pcm_pos = 0;
	raw_pcm_bytes = 0;
	enc_bytes = 0;
	printk("Opus enc ready: %d Hz, mono, %d bps, %d ms frames\n",
	       SAMPLE_RATE, OPUS_BITRATE, OPUS_FRAME_MS);
	return true;
}

/* Encode one chunk of raw PCM through the opus encoder.
 * Writes length-prefixed packets to the open rec_file. */
static void opus_encode_chunk(const uint8_t *pcm, uint32_t len)
{
	uint32_t pos = 0;
	while (pos < len && recording) {
		uint32_t need = OPUS_FRAME_BYTES - opus_pcm_pos;
		uint32_t avail = len - pos;
		uint32_t chunk = (avail < need) ? avail : need;
		memcpy(&opus_pcm_acc[opus_pcm_pos], &pcm[pos], chunk);
		opus_pcm_pos += chunk;
		pos += chunk;

		if (opus_pcm_pos >= OPUS_FRAME_BYTES) {
			int n = opus_encode(opus_enc,
				(const opus_int16 *)opus_pcm_acc,
				OPUS_FRAME_SAMPLES,
				opus_pkt, OPUS_MAX_PKT);
			if (n > 0 && file_open) {
				uint8_t hdr[2] = { n & 0xFF, (n >> 8) & 0xFF };
				fs_write(&rec_file, hdr, 2);
				fs_write(&rec_file, opus_pkt, n);
				enc_bytes += n + 2;
				/* Send via BLE queue (separate thread). */
				if (ble_audio_is_connected() && n <= 80) {
					struct ble_pkt pkt;
					pkt.len = n;
					memcpy(pkt.data, opus_pkt, n);
					k_msgq_put(&ble_pkt_q, &pkt, K_NO_WAIT);
				}
			} else if (n < 0) {
				printk("opus_encode err %d\n", n);
			}
			raw_pcm_bytes += OPUS_FRAME_BYTES;
			opus_pcm_pos = 0;
		}
	}
}

/* Battery monitoring via ADC (channel 7, on-board 1:2 divider).
 * The DTS io-channels node (zephyr,user) references channel 7 with the
 * board's gain (1/4) and reference (internal, 0.9 V on nRF54L15). */
#define BATT_DIVIDER   2  /* external divider ratio (cell = pin × N) */

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "zephyr,user / io-channels missing in overlay"
#endif

static const struct adc_dt_spec batt_adc =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static bool adc_ready;

static int read_battery_mv(void)
{
	int16_t raw;
	int32_t mv;
	int err;

	if (!adc_ready) {
		if (!device_is_ready(batt_adc.dev)) return -1;
		if (adc_channel_setup_dt(&batt_adc) != 0) return -1;
		adc_ready = true;
	}

	struct adc_sequence seq = {0};
	adc_sequence_init_dt(&batt_adc, &seq);
	seq.buffer = &raw;
	seq.buffer_size = sizeof(raw);

	/* Read twice; first is sometimes stale after regulator power-on. */
	err = adc_read(batt_adc.dev, &seq);
	if (err) return -1;
	err = adc_read(batt_adc.dev, &seq);
	if (err) return -1;

	mv = (int32_t)raw;
	err = adc_raw_to_millivolts_dt(&batt_adc, &mv);
	if (err < 0) return -1;

	/* Pin-voltage to LiPo cell voltage: compensate the external divider. */
	mv *= BATT_DIVIDER;

	return (int)mv;
}

/* Watchdog */
static const struct device *wdt_dev;
static int wdt_chan = -1;
static volatile int64_t sd_last_alive;

/* =============================  Helpers  ================================ */

static void wdt_kick(void)
{
	if (wdt_chan >= 0) {
		wdt_feed(wdt_dev, wdt_chan);
	}
}

static void sd_alive(void)
{
	sd_last_alive = k_uptime_get();
}

/* Soft-knee limiter: gentle compression above -4.4 dBFS, then hard clip.
 * x/(1+x) approximation of tanh -- fast on Cortex-M, no LUT needed. */
static int16_t soft_limit(int32_t sample)
{
	const int32_t knee  = 19660;  /* 0.6 * 32767 */
	const int32_t range = 13107;  /* 0.4 * 32767 */
	int32_t a = (sample < 0) ? -sample : sample;
	int32_t s = (sample < 0) ? -1 : 1;
	if (a <= knee) {
		return (int16_t)sample;
	}
	int32_t over = a - knee;
	int32_t c = knee + (int32_t)((int64_t)range * over / (range + over));
	if (c > 32767) {
		c = 32767;
	}
	return (int16_t)(s * c);
}

/* =============================  SD layer  =============================== */

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
		f_lseek(fp, sz);
		written = sz;
		printk("  append @ %u KB\n", (unsigned)(sz / 1024));
	}
	file_open = true;
	return true;
}

/* =============================  DMIC  =================================== */

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
		printk("dmic_configure fail\n");
		return false;
	}
	if (dmic_trigger(dmic_dev, DMIC_TRIGGER_START) != 0) {
		printk("dmic_trigger fail\n");
		return false;
	}
	return true;
}

static void dmic_disarm(void)
{
	dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
}

/* =============================  Threads  ================================ */

/* LED: solid ON when idle, fast blink when recording, OFF when sleeping. */
static void led_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		if (going_to_sleep) {
			gpio_pin_set_dt(&led, 0);
			k_msleep(100);
		} else if (recording) {
			gpio_pin_set_dt(&led, 1);
			k_msleep(150);
			gpio_pin_set_dt(&led, 0);
			k_msleep(150);
		} else {
			gpio_pin_set_dt(&led, 1);
			k_msleep(500);
		}
	}
}

static void btn_isr(const struct device *d, struct gpio_callback *c, uint32_t p)
{
	ARG_UNUSED(d); ARG_UNUSED(c); ARG_UNUSED(p);
	btn_pressed = true;
}

/* BLE sender thread: drains opus packet queue and sends GATT
 * notifications.  Own stack (2 KiB) so SD thread stack is not
 * consumed by the BLE SoftDevice call chain. */
static void ble_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	struct ble_pkt pkt;

	while (1) {
		if (k_msgq_get(&ble_pkt_q, &pkt, K_MSEC(100)) == 0) {
			ble_audio_send(pkt.data, pkt.len);
		}
	}
}

/* DMIC capture: PDM -> gain -> ring buffer. Never blocks on SD. */
static void dmic_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		while (!recording) {
			k_msleep(20);
		}
		while (recording) {
			void *buf;
			uint32_t sz;
			if (dmic_read(dmic_dev, 0, &buf, &sz, 1000) != 0) {
				if (recording) {
					dmic_arm();
				}
				continue;
			}
			if (SW_GAIN > 1) {
				int16_t *s = (int16_t *)buf;
				int n = (int)(sz / 2);
				for (int i = 0; i < n; i++) {
					s[i] = soft_limit((int32_t)s[i] * SW_GAIN);
				}
			}
			uint32_t put = ring_buf_put(&audio_ring_buf, buf, sz);
			if (put < sz) {
				dropped += (sz - put);
			}
			k_mem_slab_free(&mem_slab, buf);
			k_sem_give(&data_sem);
		}
	}
}

/* ---- SD write helpers with fault recovery ---- */

/* Returns: 0=ok, -1=transient, -2=permanent (ENOSPC). */
static int flush_write_buf(uint8_t *wbuf, uint32_t *pos)
{
	if (*pos == 0) {
		return 0;
	}
	ssize_t w = fs_write(&rec_file, wbuf, *pos);
	if (w == (ssize_t)(*pos)) {
		written += *pos;
		sd_writes++;
		*pos = 0;
		return 0;
	}
	printk("  fs_write err %d\n", (int)w);
	if (w == -ENOSPC) {
		printk("  DISK FULL\n");
		return -2;
	}
	return -1;
}

/* Reopen file handle. incident_start is set by the caller and NOT reset here.
 * Returns true once file is open, false if recording stopped / timeout. */
static bool recover_file_loop(int64_t incident_start)
{
	int attempt = 0;
	int64_t backoff_ms = 2000;

	while (recording) {
		int64_t elapsed = k_uptime_get() - incident_start;
		if (elapsed < RECOVERY_TIMEOUT_MS) {
			sd_alive();
			wdt_kick();
		} else {
			printk("  recovery timeout %llds -- reboot pending\n",
			       elapsed / 1000);
		}
		attempt++;
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
			k_msleep((int)backoff_ms);
			if (backoff_ms < 30000) {
				backoff_ms = (backoff_ms * 3 > 30000) ? 30000
					     : backoff_ms * 3;
			}
			ok = sd_mount() && open_append();
		}
		if (ok) {
			if (attempt > 1) {
				printk("  recovered after %d tries\n", attempt);
			}
			return true;
		}
		if (attempt == 1 || attempt % 5 == 0) {
			printk("  recovery %d fail\n", attempt);
		}
	}
	return false;
}

/* Recover file + flush buffer. incident_start captured once. */
static bool recover_and_flush(uint8_t *wbuf, uint32_t *pos)
{
	int64_t incident_start = k_uptime_get();
	while (recording) {
		if (!recover_file_loop(incident_start)) {
			return false;
		}
		int rc = flush_write_buf(wbuf, pos);
		if (rc == 0) {
			return true;
		}
		if (rc == -2) {
			return false;
		}
		if (k_uptime_get() - incident_start > RECOVERY_TIMEOUT_MS) {
			printk("  write timeout %llds -- giving up\n",
			       (k_uptime_get() - incident_start) / 1000);
			return false;
		}
	}
	return false;
}

/* SD writer thread: drain ring -> opus encode -> SD card. */
static void sd_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	int64_t last_hb;
	uint8_t tmp[OPUS_FRAME_BYTES * 2];

	while (1) {
		while (!recording) {
			k_msleep(50);
		}
		last_sync = k_uptime_get();
		last_hb = last_sync;
		sd_alive();

		while (recording) {
			sd_alive();
			k_sem_take(&data_sem, K_MSEC(100));

			/* Drain ring -> opus encode -> SD. */
			while (ring_buf_size_get(&audio_ring_buf) > 0 && recording) {
				uint32_t rd = ring_buf_get(&audio_ring_buf,
							  tmp, sizeof(tmp));
				if (rd == 0) break;
				opus_encode_chunk(tmp, rd);
			}

			/* Periodic FAT sync. */
			int64_t now = k_uptime_get();
			if (now - last_sync >= SYNC_MS && file_open) {
				fs_sync(&rec_file);
				last_sync = now;
			}

			/* Heartbeat every 5 s. */
			if (recording && now - last_hb >= 5000) {
				int32_t el = (int32_t)((now - rec_start) / 1000);
				int vb = read_battery_mv();
				unsigned raw_kb = raw_pcm_bytes / 1024;
				unsigned enc_kb = enc_bytes / 1024;
				unsigned ratio = raw_kb ? (raw_kb * 100 / enc_kb) : 0;
				printk("  %02d:%02d  raw %uKB  enc %uKB  "
				       "ratio %u.%02u  drop %uKB  ring %u  "
				       "vbat %dmV\n",
				       (int)(el / 60), (int)(el % 60),
				       raw_kb, enc_kb,
				       (unsigned)(ratio / 100),
				       (unsigned)(ratio % 100),
				       (unsigned)(dropped / 1024),
				       ring_buf_size_get(&audio_ring_buf),
				       vb);
				last_hb = now;
			}
		}

		/* Final sync. */
		if (file_open) {
			fs_sync(&rec_file);
		}
	}
}

/* =============================  Rec control  ============================ */

static void start_rec(void)
{
	if (recording) return;
	if (!sd_mounted && !sd_mount()) {
		printk("SD mount fail\n");
		return;
	}
	int idx = next_idx();
	if (idx < 0) {
		printk("no free index\n");
		return;
	}
	snprintf(rec_path, sizeof(rec_path), DISK_MOUNT_PT "/rec_%04d.opus", idx);
	ring_buf_reset(&audio_ring_buf);
	written = dropped = sd_writes = rec_errs = 0;
	if (!open_append()) {
		printk("open fail\n");
		return;
	}
	if (!opus_enc_init()) {
		fs_close(&rec_file);
		file_open = false;
		return;
	}
	if (!dmic_arm()) {
		printk("dmic fail\n");
		fs_close(&rec_file);
		file_open = false;
		return;
	}
	rec_start = k_uptime_get();
	sd_last_alive = rec_start;
	last_activity = rec_start;
	recording = true;
	ble_audio_set_recording(true);
	printk("REC -> %s\n", rec_path);
}

static void stop_rec(void)
{
	if (!recording) return;
	int64_t elapsed_ms = k_uptime_get() - rec_start;
	recording = false;
	ble_audio_set_recording(false);
	k_msleep(400);
	dmic_disarm();
	k_msleep(200);
	if (file_open) {
		fs_sync(&rec_file);
		fs_close(&rec_file);
		file_open = false;
	}
	opus_pcm_pos = 0;
	int32_t s = (int32_t)(elapsed_ms / 1000);
	printk("STOP %02d:%02d  raw %uKB  enc %uKB  x%u  drop %uKB\n",
	       s / 60, s % 60,
	       (unsigned)(raw_pcm_bytes / 1024),
	       (unsigned)(enc_bytes / 1024),
	       (unsigned)(raw_pcm_bytes / (enc_bytes ? enc_bytes : 1)),
	       (unsigned)(dropped / 1024));
	last_activity = k_uptime_get();
}

/* Deassert the on-board regulator-enable GPIOs so the PDM mic, the VBat
 * divider and the RF front-end switch stop drawing current while we are
 * in System OFF. On the XIAO nRF54L15 these are `regulator-boot-on`, so
 * without this they stay latched active through sleep and dominate the
 * sleep current. GPIO_OUTPUT_INACTIVE deasserts each one respecting its
 * active-high/active-low polarity. */
static void cut_board_power(void)
{
	static const struct gpio_dt_spec pwr[] = {
		GPIO_DT_SPEC_GET(DT_NODELABEL(pdm_imu_pwr), enable_gpios),
		GPIO_DT_SPEC_GET(DT_NODELABEL(vbat_pwr),    enable_gpios),
		GPIO_DT_SPEC_GET(DT_NODELABEL(rfsw_pwr),    enable_gpios),
		GPIO_DT_SPEC_GET(DT_NODELABEL(rfsw_ctl),    enable_gpios),
	};

	for (int i = 0; i < ARRAY_SIZE(pwr); i++) {
		if (!device_is_ready(pwr[i].port)) {
			continue;
		}
		gpio_pin_configure_dt(&pwr[i], GPIO_OUTPUT_INACTIVE);
	}
}

/* =============================  Sleep  ================================== */

static void enter_system_off(void)
{
	int rc;

	printk("Entering system OFF\n");
	going_to_sleep = true;

	/* Make sure capture is stopped so the PDM peripheral is idle. */
	if (recording) {
		recording = false;
		k_msleep(200);
	}
	dmic_disarm();

	/* Unmount + deinit the SD stack so SPIM/EasyDMA releases its domain. */
	sd_teardown();

	/* LED off (explicit; the led thread already dims it on going_to_sleep). */
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	/* Cut power to the on-board mic / VBat divider / RF switch. */
	cut_board_power();

	/* Disable watchdog before sleep to prevent spurious reset. */
	if (wdt_chan >= 0) {
		wdt_disable(wdt_dev);
	}

	k_msleep(200);  /* let pending ISR / printk drain */

	/* Suspend the UART console so the UARTE peripheral powers down. This
	 * must succeed, otherwise the UART keeps the HF clock domain alive and
	 * System OFF current stays high. Guarded by CONFIG_UART_CONSOLE: the
	 * battery build has no UART device at all (RTT-only) and referencing
	 * the (absent) console node would fail at link time. */
#if defined(CONFIG_UART_CONSOLE)
	const struct device *cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (device_is_ready(cons)) {
		rc = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
		if (rc < 0) {
			printk("Could not suspend console (%d)\n", rc);
			return;
		}
	}
#endif

	/* Configure button for wake from System OFF.
	 * Per Zephyr example: GPIO_INPUT only, GPIO_INT_LEVEL_ACTIVE.
	 * Button is active-low: press pulls pin low, triggers wake. */
	rc = gpio_pin_configure_dt(&btn, GPIO_INPUT);
	if (rc < 0) {
		printk("btn cfg fail %d\n", rc);
		return;
	}
	rc = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_LEVEL_ACTIVE);
	if (rc < 0) {
		printk("btn int fail %d\n", rc);
		return;
	}

	hwinfo_clear_reset_cause();
	sys_poweroff();
	/* Never reached -- wake = full reset via main(). */
}

/* =============================  Main  =================================== */

int main(void)
{
	uint32_t reset_cause;

	k_msleep(300);
	printk("\n=== Audio Recorder (button + sleep) ===\n");

	/* Check wake cause */
	hwinfo_get_reset_cause(&reset_cause);
	if (reset_cause & RESET_LOW_POWER_WAKE) {
		printk("Wakeup from System OFF by GPIO\n");
	} else if (reset_cause & RESET_PIN) {
		printk("Reset by pin\n");
	} else {
		printk("Reset cause: 0x%08X\n", reset_cause);
	}

	/* Watchdog */
	wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	if (device_is_ready(wdt_dev)) {
		struct wdt_timeout_cfg wcfg = {
			.window = { .min = 0, .max = WDT_TIMEOUT_MS },
			.flags = WDT_FLAG_RESET_SOC,
		};
		wdt_chan = wdt_install_timeout(wdt_dev, &wcfg);
		if (wdt_chan >= 0) {
			wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
		}
	}

	/* DMIC */
	dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	if (!device_is_ready(dmic_dev)) {
		printk("DMIC not ready\n");
		return 0;
	}

	/* GPIO: LED ON, button with pull-up (active-low). */
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&btn, GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&btn_cb, btn_isr, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb);

	/* Threads */
	k_thread_create(&led_th, led_stk, K_THREAD_STACK_SIZEOF(led_stk),
			led_fn, 0, 0, 0, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_create(&dmic_th, dmic_stk, K_THREAD_STACK_SIZEOF(dmic_stk),
			dmic_fn, 0, 0, 0, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_thread_create(&sd_th, sd_stk, K_THREAD_STACK_SIZEOF(sd_stk),
			sd_fn, 0, 0, 0, K_PRIO_PREEMPT(3), 0, K_NO_WAIT);
	k_thread_create(&ble_th, ble_stk, K_THREAD_STACK_SIZEOF(ble_stk),
			ble_fn, 0, 0, 0, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

	/* Mount SD */
	while (!sd_mounted) {
		wdt_kick();
		if (sd_mount()) break;
		printk("SD not ready, retrying...\n");
		k_sleep(K_SECONDS(2));
	}
	printk("SD mounted\n");
	int vbat = read_battery_mv();
	printk("VBat: %d mV\n", vbat);
	printk("Ready. Short click <%ds = record, hold >%ds = pair.\n"
	       "Sleeps after %d s idle.\n",
	       (int)(BTN_SHORT_MS / 1000),
	       (int)(BTN_LONG_MS / 1000),
	       (int)(IDLE_SLEEP_MS / 1000));

	last_activity = k_uptime_get();
	last_batt = last_activity;

#if defined(CONFIG_RAM_POWER_DOWN_LIBRARY)
	/* Power down RAM blocks the application image does not use. Lowers
	 * System-ON idle current (each unused 32 KiB block ~0.3-0.5 uA). Safe
	 * here because the heap is a fixed k_heap, not the libc heap. */
	power_down_unused_ram();
#endif

#if TEST_OPUS
	/* Auto-test: record 30 seconds, then stop. */
	printk("[TEST] auto-start recording 30s...\n");
	start_rec();
	if (recording) {
		k_msleep(30000);
		stop_rec();
		printk("[TEST] done.\n");
	}
#endif

	/* BLE init + normal advertising (reconnect to bonded device).
	 * Button short/long press is handled in the main loop. */
	if (ble_audio_init() == 0) {
		/* Publish the boot battery reading so the phone shows it as
		 * soon as it connects / subscribes. */
		ble_audio_set_battery_mv(read_battery_mv());
		ble_audio_start_advertising(false);
	}

	/* If the wake button is still held, feed it into the main-loop
	 * handler so the user can short-tap to record or long-hold to pair. */
	if (!gpio_pin_get_dt(&btn)) {
		btn_pressed = true;
	}

	while (1) {
		/* Periodic battery refresh over BLE. */
		if (ble_audio_has_conn() &&
		    (k_uptime_get() - last_batt) >= BATT_UPDATE_MS) {
			last_batt = k_uptime_get();
			ble_audio_set_battery_mv(read_battery_mv());
		}

		if (btn_pressed) {
			static int64_t last_btn;
			if (k_uptime_get() - last_btn >= BTN_DEBOUNCE_MS) {
				last_btn = k_uptime_get();
				btn_pressed = false;
				last_activity = last_btn;

				/* Boot/idle button window:
				 *   click held < 2 s  -> toggle recording
				 *   hold  >= 5 s      -> clear bonds & re-pair
				 *   2..5 s            -> ignored (ambiguous)
				 * Poll the button while it is held, up to BTN_LONG_MS. */
				int64_t press_start = k_uptime_get();
				while (!gpio_pin_get_dt(&btn) &&
				       (k_uptime_get() - press_start) < BTN_LONG_MS) {
					k_msleep(50);
				}
				int64_t held_ms = k_uptime_get() - press_start;

				if (held_ms >= BTN_LONG_MS) {
					/* Long hold -> pairing mode. */
					printk("Long press %lldms -> pairing\n", held_ms);
					stop_rec();
					ble_audio_clear_bonds();
					ble_audio_start_advertising(true);
				} else if (held_ms < BTN_SHORT_MS) {
					/* Short click -> toggle recording. */
					if (recording) {
						stop_rec();
					} else {
						start_rec();
					}
				} else {
					printk("Press %lldms ignored (need <%ds or >%ds)\n",
					       held_ms,
					       (int)(BTN_SHORT_MS / 1000),
					       (int)(BTN_LONG_MS / 1000));
				}
			} else {
				btn_pressed = false;
			}
		}

		/* Idle timeout -> system OFF. Skip while recording or while a
		 * phone is connected over BLE. Wake on button GPIO. */
		if (!recording && !ble_audio_has_conn() &&
		    (k_uptime_get() - last_activity) >= IDLE_SLEEP_MS) {
			enter_system_off();
		}

		/* Watchdog: feed only if SD writer is making progress. */
		bool sd_ok = !recording ||
			     (k_uptime_get() - sd_last_alive < SD_STALE_MS);
		if (sd_ok) {
			wdt_kick();
		}

		k_msleep(50);
	}
}
