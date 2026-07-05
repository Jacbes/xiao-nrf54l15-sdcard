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
 *   2 min idle -> system OFF (ultra-low-power), wake on button press.
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
#include <zephyr/sys/poweroff.h>
#include <hal/nrf_gpio.h>
#include <ff.h>
#include <string.h>
#include <stdio.h>

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

/* Ring buffer: 2 s of latency absorption between DMIC and SD writer. */
#define RING_BUF_SIZE    65536
RING_BUF_DECLARE(audio_ring_buf, RING_BUF_SIZE);

/* Write buffer: flush 1 s of audio per fs_write. */
#define WRITE_BUF_SIZE   AUDIO_BPS

/* Timing */
#define SYNC_MS          8000    /* FAT commit interval */
#define IDLE_SLEEP_MS    10000   /* 10 s idle -> system OFF */
#define BTN_DEBOUNCE_MS  300

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
static bool file_open;
static bool sd_mounted;

static struct fs_file_t rec_file;
static char rec_path[64];

static int64_t rec_start, last_sync, last_activity;
static uint32_t written, dropped, sd_writes, rec_errs;

K_THREAD_STACK_DEFINE(led_stk, 512);
static struct k_thread led_th;
K_THREAD_STACK_DEFINE(dmic_stk, 2048);
static struct k_thread dmic_th;
K_THREAD_STACK_DEFINE(sd_stk, 12288);
static struct k_thread sd_th;
K_SEM_DEFINE(data_sem, 0, 1);

/* Battery monitoring via ADC (channel 7, on-board divider). */
#define VBATT_ADC_CH     7
static bool adc_ch_ready;

static int read_battery_mv(void)
{
	const struct device *d = DEVICE_DT_GET(DT_NODELABEL(adc));
	if (!device_is_ready(d)) return -1;
	if (!adc_ch_ready) {
		struct adc_channel_cfg cc = {
			.gain = ADC_GAIN_1_4,
			.reference = ADC_REF_INTERNAL,
			.acquisition_time = ADC_ACQ_TIME_DEFAULT,
			.channel_id = VBATT_ADC_CH,
#if defined(CONFIG_ADC_NRFX_SAADC)
			.input_positive = 7,
#endif
		};
		if (adc_channel_setup(d, &cc) != 0) return -1;
		adc_ch_ready = true;
	}
	int16_t sb = 0;
	struct adc_sequence as = {
		.channels = BIT(VBATT_ADC_CH),
		.buffer = &sb,
		.buffer_size = sizeof(sb),
		.resolution = 12,
	};
	if (adc_read(d, &as)) return -1;
	return (int)(((int32_t)sb * 2400) / 4096);
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

/* LED: solid ON when idle, fast blink when recording. */
static void led_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		if (recording) {
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

/* SD writer thread: drain ring -> write buf -> SD card. */
static void sd_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	static uint8_t wbuf[WRITE_BUF_SIZE];
	uint32_t pos = 0;
	int64_t last_hb;
	int64_t sync_incident_start;

	while (1) {
		while (!recording) {
			k_msleep(50);
		}
		last_sync = k_uptime_get();
		last_hb = last_sync;
		sync_incident_start = 0;
		sd_alive();
		pos = 0;

		while (recording) {
			sd_alive();
			k_sem_take(&data_sem, K_MSEC(100));

			/* Drain ring into write buffer. */
			while (ring_buf_size_get(&audio_ring_buf) > 0
			       && pos < WRITE_BUF_SIZE) {
				uint32_t rd = ring_buf_get(&audio_ring_buf,
							   &wbuf[pos],
							   WRITE_BUF_SIZE - pos);
				if (rd == 0) break;
				pos += rd;
			}

			/* Flush when buffer full (1 s audio). */
			if (pos >= WRITE_BUF_SIZE) {
				int wrc = flush_write_buf(wbuf, &pos);
				if (wrc < 0) {
					rec_errs++;
					if (wrc == -2) { recording = false; break; }
					if (!recover_and_flush(wbuf, &pos)) {
						recording = false;
						break;
					}
				}
			}
			if (!recording) break;

			/* Periodic FAT sync. */
			int64_t now = k_uptime_get();
			if (now - last_sync >= SYNC_MS) {
				if (pos > 0) {
					int wrc = flush_write_buf(wbuf, &pos);
					if (wrc < 0) {
						rec_errs++;
						if (wrc == -2) { recording = false; break; }
						if (!recover_and_flush(wbuf, &pos)) {
							recording = false;
							break;
						}
					}
				}
				if (file_open && fs_sync(&rec_file) != 0) {
					rec_errs++;
					if (sync_incident_start == 0) {
						sync_incident_start = k_uptime_get();
					}
					if (!recover_file_loop(sync_incident_start)) {
						recording = false;
						break;
					}
					if (k_uptime_get() - sync_incident_start
					    > RECOVERY_TIMEOUT_MS) {
						printk("  sync timeout -- giving up\n");
						recording = false;
						break;
					}
				} else if (file_open) {
					sync_incident_start = 0;
				}
				last_sync = now;
			}

			/* Heartbeat every 5 s. */
			if (recording && now - last_hb >= 5000) {
				int32_t el = (int32_t)((now - rec_start) / 1000);
				int vb = read_battery_mv();
				printk("  %02d:%02d  %uKB  drop %uKB  errs %u  "
				       "ring %u  vbat %dmV\n",
				       (int)(el / 60), (int)(el % 60),
				       (unsigned)(written / 1024),
				       (unsigned)(dropped / 1024),
				       (unsigned)rec_errs,
				       ring_buf_size_get(&audio_ring_buf), vb);
				last_hb = now;
			}
		}

		/* Final flush. */
		if (pos > 0 && file_open) {
			flush_write_buf(wbuf, &pos);
		}
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
	snprintf(rec_path, sizeof(rec_path), DISK_MOUNT_PT "/rec_%04d.raw", idx);
	ring_buf_reset(&audio_ring_buf);
	written = dropped = sd_writes = rec_errs = 0;
	if (!open_append()) {
		printk("open fail\n");
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
	printk("REC -> %s\n", rec_path);
}

static void stop_rec(void)
{
	if (!recording) return;
	int64_t elapsed_ms = k_uptime_get() - rec_start;
	recording = false;
	k_msleep(400);
	dmic_disarm();
	k_msleep(200);
	if (file_open) {
		fs_sync(&rec_file);
		fs_close(&rec_file);
		file_open = false;
	}
	int32_t s = (int32_t)(elapsed_ms / 1000);
	printk("STOP %02d:%02d  %uKB  drop %uKB  errs %u\n",
	       s / 60, s % 60,
	       (unsigned)(written / 1024),
	       (unsigned)(dropped / 1024),
	       (unsigned)rec_errs);
	last_activity = k_uptime_get();
}

/* =============================  Main  =================================== */

int main(void)
{
	k_msleep(300);
	printk("\n=== Audio Recorder (button + sleep) ===\n");

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

	/* GPIO: LED ON, button with pull-up (active-low, also used for
	 * System OFF wake: pin senses low level while sleeping). */
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
	printk("Ready. Press button to record. Sleep after %d s idle.\n",
	       (int)(IDLE_SLEEP_MS / 1000));

	last_activity = k_uptime_get();

	while (1) {
		if (btn_pressed) {
			static int64_t last_btn;
			if (k_uptime_get() - last_btn >= BTN_DEBOUNCE_MS) {
				last_btn = k_uptime_get();
				btn_pressed = false;
				last_activity = last_btn;
				if (recording) {
					stop_rec();
				} else {
					start_rec();
				}
			} else {
				btn_pressed = false;
			}
		}

		/* Idle timeout -> system OFF. Wake on button GPIO. */
		if (!recording
		    && (k_uptime_get() - last_activity) >= IDLE_SLEEP_MS) {
			printk("Idle %ds -- entering system OFF\n",
			       (int)(IDLE_SLEEP_MS / 1000));
			sd_teardown();
			gpio_pin_set_dt(&led, 0);  /* LED off */
			k_msleep(100);

			/* Configure GPIO SENSE on the button pin so the chip wakes
			 * when the button is pressed (pin goes low). Without this,
			 * only the RESET pin can wake the chip from System OFF. */
			nrf_gpio_cfg_sense_input(
				NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios),
				NRF_GPIO_PIN_PULLUP,
				NRF_GPIO_PIN_SENSE_LOW);

			sys_poweroff();
			/* Never reached -- wake = full reset via main(). */
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
