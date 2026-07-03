/*
 * Audio recorder for Seeed XIAO nRF54L15 (Sense).
 *
 * Architecture (Nordic best practice for nRF54):
 *   - DMIC reader thread (high priority): drains PDM buffers ASAP into a
 *     ring buffer, freeing mem_slab buffers immediately.  This prevents
 *     the "Failed to allocate buffer" / "No room in RX queue" cascade
 *     caused by the Zephyr DMIC driver stopping PDM on slab exhaustion.
 *   - SD writer thread (normal priority): reads PCM from ring buffer,
 *     applies gain, writes to file.
 *
 * Press P0.00 to start/stop recording.  LED P2.00 blinks while recording.
 * Audio is written in chunks: every CHUNK_SECONDS the file is fs_sync'd +
 * fs_closed and re-opened for append.  Already-closed chunks survive
 * power loss / SD removal.
 *
 * Audio format: 16 kHz, mono, 16-bit little-endian raw PCM.
 *   ffmpeg -f s16le -ar 16000 -ac 1 -i rec_0001.raw out.wav
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ---- Configuration ---- */
#define DISK_DRIVE_NAME  "SD"
#define DISK_MOUNT_PT    "/" DISK_DRIVE_NAME ":"

#define SAMPLE_RATE      16000
#define SAMPLE_BITS      16
#define BYTES_PER_SAMPLE (SAMPLE_BITS / 8)
#define CHANNELS         1

/* 50 ms per DMIC block — smaller blocks = faster turnaround, less slab
 * pressure.  At 16 kHz mono 16-bit, 50 ms = 1600 bytes. */
#define BLOCK_SIZE       (BYTES_PER_SAMPLE * (SAMPLE_RATE / 20) * CHANNELS)
#define BLOCK_COUNT      16

/* Digital gain.  nRF54L PDM has no hardware gain registers, so all
 * amplification is done in software in the DMIC reader thread.
 *  x8 = +18 dB,  x16 = +24 dB,  x32 = +30 dB,  x64 = +36 dB */
#define SW_GAIN          32

/* Ring buffer between DMIC reader and SD writer.
 * 20 KB ≈ 640 ms at 32 KB/s — enough to absorb SD card write stalls. */
#define RING_BUF_SIZE    20480

#define CHUNK_SECONDS    5
#define CHUNK_MS         (CHUNK_SECONDS * 1000)
#define BTN_DEBOUNCE_MS  300

/* Battery ADC: AIN7 (P1.11 = A1 pin on XIAO).
 * Wiki says AIN7_VBAT is connected via voltage divider.
 * With gain=1/4, ref=0.6V internal, 12-bit:
 *   range = 0.6V * 4 = 2.4V, 1 LSB ≈ 0.586 mV */
#define VBATT_ADC_CHANNEL    7
#define VBATT_ADC_RESOLUTION 12

static int read_battery_mv(void)
{
	const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
	if (!device_is_ready(adc_dev)) {
		return -1;
	}

	struct adc_channel_cfg ch_cfg = {
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = VBATT_ADC_CHANNEL,
#if defined(CONFIG_ADC_NRFX_SAADC)
		.input_positive = 7, /* AIN7 */
#endif
	};

	int16_t sample_buf = 0;
	struct adc_sequence seq = {
		.channels = BIT(VBATT_ADC_CHANNEL),
		.buffer = &sample_buf,
		.buffer_size = sizeof(sample_buf),
		.resolution = VBATT_ADC_RESOLUTION,
	};

	int ret = adc_channel_setup(adc_dev, &ch_cfg);
	if (ret) {
		return -2;
	}

	ret = adc_read(adc_dev, &seq);
	if (ret) {
		return -3;
	}

	/* Convert raw to millivolts at ADC pin.
	 * gain=1/4, ref=0.6V, 12-bit: 1 LSB = 2400/4096 mV.
	 * Board voltage divider ratio unknown — report raw ADC mV. */
	int32_t mv = ((int32_t)sample_buf * 2400) / 4096;

	return (int)mv;
}

/* ---- Static data ---- */
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

K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* Ring buffer: DMIC reader → SD writer */
RING_BUF_DECLARE(audio_ring_buf, RING_BUF_SIZE);

/* Recording state */
static volatile bool button_pressed;
static volatile bool recording;
static int rec_index;
static int64_t chunk_start_ms;
static struct fs_file_t rec_file;
static char rec_path[64];

/* Threads */
static K_THREAD_STACK_DEFINE(led_stack, 1024);
static struct k_thread led_thread_data;

static K_THREAD_STACK_DEFINE(dmic_stack, 2048);
static struct k_thread dmic_thread_data;

static K_THREAD_STACK_DEFINE(sdwriter_stack, 4096);
static struct k_thread sdwriter_thread_data;

/* Semaphore: SD writer blocks until DMIC reader signals data available */
K_SEM_DEFINE(data_ready_sem, 0, 1);

/* ---- LED blink thread ---- */
static void led_thread_fn(void *p1, void *p2, void *p3)
{
	while (1) {
		if (recording) {
			gpio_pin_set_dt(&led, 0);
			k_msleep(200);
			gpio_pin_set_dt(&led, 1);
			k_msleep(200);
		} else {
			/* Heartbeat: brief flash every 2s so user
			 * can see firmware is alive (even on battery) */
			gpio_pin_set_dt(&led, 0);
			k_msleep(50);
			gpio_pin_set_dt(&led, 1);
			k_msleep(1950);
		}
	}
}

/* ---- Button ISR ---- */
static void btn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	button_pressed = true;
}

/* ---- SD card helpers ---- */
static bool card_responds(void)
{
	if (disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_INIT, NULL) == 0) {
		return true;
	}
	(void)disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
	return false;
}

static bool sd_mount(void)
{
	if (!card_responds()) {
		return false;
	}
	if (fs_mount(&mp) != 0) {
		(void)disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
		return false;
	}
	return true;
}

static int find_next_rec_index(void)
{
	char path[64];
	struct fs_dirent ent;

	for (int n = 1; n < 10000; n++) {
		snprintf(path, sizeof(path), DISK_MOUNT_PT "/rec_%04d.raw", n);
		if (fs_stat(path, &ent) != 0) {
			return n;
		}
	}
	return -1;
}

static bool open_rec_file(int index)
{
	snprintf(rec_path, sizeof(rec_path), DISK_MOUNT_PT "/rec_%04d.raw", index);
	fs_file_t_init(&rec_file);
	if (fs_open(&rec_file, rec_path, FS_O_CREATE | FS_O_WRITE) != 0) {
		printk("  [FAIL] open %s\n", rec_path);
		return false;
	}
	printk("  opened %s\n", rec_path);
	return true;
}

static void reopen_rec_file(void)
{
	FIL *fp;

	if (fs_sync(&rec_file) != 0) {
		printk("  [FAIL] fs_sync on reopen\n");
		recording = false;
		return;
	}
	if (fs_close(&rec_file) != 0) {
		printk("  [FAIL] fs_close on reopen\n");
		recording = false;
		return;
	}

	fs_file_t_init(&rec_file);
	if (fs_open(&rec_file, rec_path, FS_O_CREATE | FS_O_WRITE) != 0) {
		printk("  [FAIL] reopen %s\n", rec_path);
		recording = false;
		return;
	}

	fp = (FIL *)rec_file.filep;
	if (f_lseek(fp, f_size(fp)) != FR_OK) {
		printk("  [WARN] seek to end failed\n");
	}
}

static void close_rec_file(void)
{
	int rc = fs_sync(&rec_file);
	if (rc != 0) {
		printk("  [FAIL] fs_sync: %d\n", rc);
	}
	rc = fs_close(&rec_file);
	if (rc != 0) {
		printk("  [FAIL] fs_close: %d\n", rc);
	} else {
		printk("  closed (synced)\n");
	}
}

/* ---- DMIC start/stop with hardware gain ---- */
static bool dmic_start(void)
{
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BITS,
		.mem_slab  = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
			.req_num_chan = 1,
			.req_chan_map_lo =
				dmic_build_channel_map(0, 0, PDM_CHAN_LEFT),
		},
	};
	int ret;

	cfg.streams[0].pcm_rate = SAMPLE_RATE;
	cfg.streams[0].block_size = BLOCK_SIZE;

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret != 0) {
		printk("  [FAIL] dmic_configure: %d\n", ret);
		return false;
	}
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret != 0) {
		printk("  [FAIL] dmic_trigger START: %d\n", ret);
		return false;
	}
	return true;
}

static void dmic_stop(void)
{
	(void)dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
}

/* ---- DMIC reader thread (high priority) ----
 * Drains PDM buffers from the DMIC driver into the ring buffer as fast as
 * possible, freeing mem_slab buffers immediately.  This is the key to
 * preventing the "Failed to allocate buffer: -12" cascade: the slab never
 * runs dry because buffers are returned within microseconds.
 *
 * If the ring buffer is full (SD writer stalled), data is dropped. */
static void dmic_reader_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	while (1) {
		/* Wait until recording is started */
		while (!recording) {
			k_msleep(20);
		}

		while (recording) {
			void *buffer;
			uint32_t size;
			int ret;

			ret = dmic_read(dmic_dev, 0, &buffer, &size, 500);
			if (ret != 0) {
				/* PDM stopped (slab exhausted or RX queue
				 * overflow).  Only restart if still recording. */
				if (recording) {
					dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
				}
				continue;
			}

			/* Apply minimal software gain (hardware already
			 * provides +20 dB; SW_GAIN adds ~+10 dB more). */
			if (SW_GAIN > 1) {
				int16_t *s = (int16_t *)buffer;
				int n = size / 2;
				for (int i = 0; i < n; i++) {
					int32_t v = (int32_t)s[i] * SW_GAIN;
					if (v > 32767) v = 32767;
					else if (v < -32768) v = -32768;
					s[i] = (int16_t)v;
				}
			}

			/* Copy to ring buffer.  ring_buf_put returns the
			 * number of bytes actually written — if the ring
			 * buffer is full, excess data is silently dropped.
			 * This is intentional: it provides backpressure
			 * without crashing. */
			ring_buf_put(&audio_ring_buf, buffer, size);

			/* Free the slab buffer IMMEDIATELY */
			k_mem_slab_free(&mem_slab, buffer);

			/* Signal SD writer that data is available */
			k_sem_give(&data_ready_sem);
		}
	}
}

/* ---- SD writer thread (normal priority) ----
 * Reads PCM data from the ring buffer and writes to the SD card file.
 * Handles chunked writes, error recovery, and file management. */
static void sd_writer_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
	uint8_t tmp_buf[BLOCK_SIZE];

	while (1) {
		/* Wait until recording is started */
		while (!recording) {
			k_msleep(50);
		}

		chunk_start_ms = k_uptime_get();

		while (recording) {
			/* Wait for data from DMIC reader */
			k_sem_take(&data_ready_sem, K_MSEC(200));

			/* Drain ring buffer and write to file */
			while (ring_buf_size_get(&audio_ring_buf) > 0) {
				uint32_t read = ring_buf_get(&audio_ring_buf,
							     tmp_buf,
							     sizeof(tmp_buf));
				if (read == 0) {
					break;
				}

				ssize_t w = fs_write(&rec_file, tmp_buf, read);
				if (w != (ssize_t)read) {
					/* SD write error.  Do NOT call fs_close
					 * (would corrupt FAT).  Unmount clears the
					 * in-memory FATFS state.  Keep recording=true
					 * so the DMIC reader keeps draining PDM
					 * buffers (prevents slab exhaustion). */
					printk("  SD err %d, recovering...\n",
					       (int)w);
					/* Drain stale data from ring buffer */
					while (ring_buf_size_get(&audio_ring_buf) > 0) {
						uint32_t skip = ring_buf_get(
							&audio_ring_buf,
							tmp_buf, sizeof(tmp_buf));
						if (!skip) break;
					}
					fs_unmount(&mp);
					disk_access_ioctl(DISK_DRIVE_NAME,
						DISK_IOCTL_CTRL_DEINIT, NULL);
					k_msleep(1000);
					if (sd_mount()) {
						int idx = find_next_rec_index();
						if (idx > 0 &&
						    open_rec_file(idx)) {
							rec_index = idx;
							chunk_start_ms =
								k_uptime_get();
							printk("  ok -> rec_%04d.raw\n",
							       rec_index);
							continue;
						}
					}
					printk("  recovery failed, stopping\n");
					recording = false;
					break;
				}
			}

			/* Chunked file reopen for power-loss safety */
			if (recording &&
			    k_uptime_get() - chunk_start_ms >= CHUNK_MS) {
				reopen_rec_file();
				if (recording) {
					chunk_start_ms = k_uptime_get();
				}
			}
		}
	}
}

/* ---- Button-driven recording control ---- */
static void start_recording(void)
{
	int idx = find_next_rec_index();
	if (idx < 0) {
		printk("No free recording index\n");
		return;
	}
	if (!open_rec_file(idx)) {
		return;
	}
	if (!dmic_start()) {
		fs_close(&rec_file);
		return;
	}

	/* Flush the ring buffer */
	ring_buf_reset(&audio_ring_buf);

	rec_index = idx;
	recording = true;
	printk("Recording started (file rec_%04d.raw)\n", rec_index);
}

static void stop_recording(void)
{
	if (!recording) {
		return;
	}
	recording = false;
	dmic_stop();
	/* Give SD writer time to flush remaining data */
	k_msleep(500);
	close_rec_file();
	printk("Recording stopped\n");
}

static void handle_button(void)
{
	static int64_t last_btn_ms;
	int64_t now = k_uptime_get();

	if (now - last_btn_ms < BTN_DEBOUNCE_MS) {
		return;
	}
	last_btn_ms = now;
	button_pressed = false;

	if (recording) {
		stop_recording();
	} else {
		start_recording();
	}
}

/* ---- Main ---- */
int main(void)
{
	k_msleep(300);

	printk("\n");
	printk("========================================================\n");
	printk(" XIAO nRF54L15 Audio Recorder\n");
	printk(" Button: P0.00   LED: P2.00   Mic: P1.12 CLK, P1.13 DAT\n");
	printk(" SD: P1.04 CS, P1.05 SCK, P1.06 MOSI, P1.07 MISO (4 MHz)\n");
	printk(" SW gain: x%d (~+%d dB)\n",
	       SW_GAIN, (int)(20.0 * log10((double)SW_GAIN)));

	/* Read battery voltage */
	int vbat_mv = read_battery_mv();
	if (vbat_mv > 0) {
		printk(" Battery: %d mV\n", vbat_mv);
	} else {
		printk(" Battery: N/A (err %d)\n", vbat_mv);
	}

	printk("========================================================\n");

	dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	if (!device_is_ready(dmic_dev)) {
		printk("DMIC device not ready\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&btn)) {
		printk("GPIO not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&btn, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&btn_cb, btn_isr, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb);

	/* Start all threads */
	k_thread_create(&led_thread_data, led_stack,
			K_THREAD_STACK_SIZEOF(led_stack),
			led_thread_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(10), 0, K_NO_WAIT);

	k_thread_create(&dmic_thread_data, dmic_stack,
			K_THREAD_STACK_SIZEOF(dmic_stack),
			dmic_reader_thread_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(1), 0, K_NO_WAIT);

	k_thread_create(&sdwriter_thread_data, sdwriter_stack,
			K_THREAD_STACK_SIZEOF(sdwriter_stack),
			sd_writer_thread_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

	/* Try to mount SD card — don't block forever, LED/button must
	 * work even without SD card. */
	for (int i = 0; i < 10; i++) {
		if (sd_mount()) {
			break;
		}
		printk("Waiting for SD card... (%d/10)\n", i + 1);
		k_sleep(K_SECONDS(2));
	}
	printk("SD card mounted\n");
	printk("Press button (P0.00) to start/stop recording\n");

	/* Main loop: just handles button presses */
	while (1) {
		if (button_pressed) {
			handle_button();
		}
		k_msleep(50);
	}

	return 0;
}
