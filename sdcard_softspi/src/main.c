/*
 * SD card test for Seeed XIAO nRF54L15 (Sense) over software (bit-bang) SPI.
 *
 * Wiring:
 *   SCK  = P2.01 (D8)
 *   MOSI = P2.02 (D10)
 *   MISO = P2.04 (D9)
 *   CS   = P1.04 (D0)
 *
 * Behaviour on each boot:
 *   1. Wait until the SD card answers (so wiring / re-seating / power-cycling
 *      the card can be fixed live without re-flashing).
 *   2. Create a new persistent file test{N}.txt containing "Hello" (N grows
 *      every boot: test1.txt, test2.txt, ...).
 *   3. Run a full read/write/verify self-test.
 *   4. Heartbeat the status once per few seconds.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(sdtest, LOG_LEVEL_INF);

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT   "/" DISK_DRIVE_NAME ":"

static FATFS fat_fs;
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = DISK_MOUNT_PT,
};

static int g_pass;
static int g_fail;
static char g_boot_file[48];   /* path of the test{N}.txt created this boot */

#define CHECK(cond, ...)                                                       \
	do {                                                                   \
		if (cond) {                                                    \
			g_pass++;                                              \
			printk("  [ OK ] " __VA_ARGS__);                       \
			printk("\n");                                          \
		} else {                                                       \
			g_fail++;                                              \
			printk("  [FAIL] " __VA_ARGS__);                       \
			printk("\n");                                          \
		}                                                              \
	} while (0)

#define BLOCK        4096
#define LARGE_BYTES  (128 * 1024)
static uint8_t wbuf[BLOCK];
static uint8_t rbuf[BLOCK];

#define SMALL_PATH   DISK_MOUNT_PT "/sdtest.txt"
#define LARGE_PATH   DISK_MOUNT_PT "/sdbig.bin"

static const char small_payload[] =
	"XIAO nRF54L15 SD card self-test\r\n"
	"SCK=P2.01 MOSI=P2.02 MISO=P2.04 CS=P1.04\r\n"
	"The quick brown fox jumps over the lazy dog 0123456789\r\n";

static void fill_pattern(uint8_t *buf, size_t len, uint32_t seed)
{
	for (size_t i = 0; i < len; i++) {
		buf[i] = (uint8_t)((seed + i * 31u + (i >> 3)) & 0xFF);
	}
}

static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

	fs_dir_t_init(&dirp);
	res = fs_opendir(&dirp, path);
	if (res) {
		printk("       Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printk("       Listing %s :\n", path);
	for (;;) {
		res = fs_readdir(&dirp, &entry);
		if (res || entry.name[0] == 0) {
			break;
		}
		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("         [DIR ] %s\n", entry.name);
		} else {
			printk("         [FILE] %s (%zu bytes)\n",
			       entry.name, entry.size);
		}
		count++;
	}
	fs_closedir(&dirp);
	return res == 0 ? count : res;
}

/* Try to bring the card up. Returns true (card left initialised) or false. */
static bool card_responds(void)
{
	static const char *pdrv = DISK_DRIVE_NAME;

	if (disk_access_ioctl(pdrv, DISK_IOCTL_CTRL_INIT, NULL) == 0) {
		return true;
	}
	(void)disk_access_ioctl(pdrv, DISK_IOCTL_CTRL_DEINIT, NULL);
	return false;
}

/* ---- raw disk info (card already initialised) ------------------------ */
static bool test_raw_disk(uint32_t *out_block_count, uint32_t *out_block_size)
{
	static const char *pdrv = DISK_DRIVE_NAME;
	uint32_t block_count = 0;
	uint32_t block_size = 0;
	bool ok = true;

	printk("\n--- 1. Raw disk access (card identify) ---\n");
	CHECK(true, "SD card init (400 kHz handshake over SPI)");

	if (disk_access_ioctl(pdrv, DISK_IOCTL_GET_SECTOR_COUNT,
			      &block_count) != 0) {
		CHECK(false, "read sector count");
		ok = false;
	} else {
		CHECK(block_count > 0, "sector count = %u", block_count);
	}

	if (disk_access_ioctl(pdrv, DISK_IOCTL_GET_SECTOR_SIZE,
			      &block_size) != 0) {
		CHECK(false, "read sector size");
		ok = false;
	} else {
		CHECK(block_size == 512, "sector size = %u bytes", block_size);
	}

	if (ok) {
		uint64_t bytes = (uint64_t)block_count * block_size;
		printk("       => Card capacity: %u MB (%u MiB)\n",
		       (uint32_t)(bytes / 1000000ULL),
		       (uint32_t)(bytes >> 20));
	}

	*out_block_count = block_count;
	*out_block_size = block_size;
	return ok;
}

static bool test_mount(void)
{
	printk("\n--- 2. Mount FAT filesystem ---\n");

	int res = fs_mount(&mp);

	CHECK(res == FR_OK, "fs_mount(%s) -> %d", DISK_MOUNT_PT, res);
	if (res != FR_OK) {
		return false;
	}

	struct fs_statvfs sbuf;

	res = fs_statvfs(DISK_MOUNT_PT, &sbuf);
	if (res == 0) {
		uint64_t total = (uint64_t)sbuf.f_blocks * sbuf.f_frsize;
		uint64_t freeb = (uint64_t)sbuf.f_bfree * sbuf.f_frsize;

		printk("       Volume total : %u MB\n",
		       (uint32_t)(total / 1000000ULL));
		printk("       Volume free  : %u MB\n",
		       (uint32_t)(freeb / 1000000ULL));
		CHECK(total > 0, "statvfs reports non-zero volume size");
	} else {
		CHECK(false, "fs_statvfs -> %d", res);
	}

	lsdir(DISK_MOUNT_PT);
	return true;
}

/* ---- persistent per-boot file: create test{N}.txt = "Hello" ---------- */
static void create_boot_file(void)
{
	char path[64];
	struct fs_dirent ent;
	struct fs_file_t f;
	int n = 1;

	printk("\n--- Boot file: create test{N}.txt containing \"Hello\" ---\n");

	for (;;) {
		snprintf(path, sizeof(path), DISK_MOUNT_PT "/test%d.txt", n);
		if (fs_stat(path, &ent) != 0) {
			break;
		}
		if (++n > 100000) {
			break;
		}
	}

	fs_file_t_init(&f);
	int rc = fs_open(&f, path, FS_O_CREATE | FS_O_WRITE);
	if (rc != 0) {
		CHECK(false, "create %s", path);
		return;
	}
	ssize_t w = fs_write(&f, "Hello", 5);
	fs_sync(&f);
	fs_close(&f);
	CHECK(w == 5, "wrote \"Hello\" to %s (%d bytes)", path, (int)w);
	if (w == 5) {
		strncpy(g_boot_file, path, sizeof(g_boot_file) - 1);
	}

	lsdir(DISK_MOUNT_PT);
}

static bool test_small_file(void)
{
	struct fs_file_t f;
	int res;
	bool ok = true;

	printk("\n--- 3. Small file write / read-back / verify ---\n");

	fs_file_t_init(&f);
	res = fs_open(&f, SMALL_PATH, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	CHECK(res == 0, "open %s for write", SMALL_PATH);
	if (res != 0) {
		return false;
	}
	ssize_t n = fs_write(&f, small_payload, sizeof(small_payload) - 1);
	CHECK(n == (ssize_t)(sizeof(small_payload) - 1), "wrote %d bytes", (int)n);
	fs_close(&f);

	fs_file_t_init(&f);
	res = fs_open(&f, SMALL_PATH, FS_O_READ);
	CHECK(res == 0, "reopen %s for read", SMALL_PATH);
	if (res != 0) {
		return false;
	}
	memset(rbuf, 0, sizeof(rbuf));
	n = fs_read(&f, rbuf, sizeof(rbuf));
	fs_close(&f);
	CHECK(n == (ssize_t)(sizeof(small_payload) - 1), "read back %d bytes", (int)n);

	if (n == (ssize_t)(sizeof(small_payload) - 1) &&
	    memcmp(rbuf, small_payload, n) == 0) {
		CHECK(true, "content matches byte-for-byte");
	} else {
		CHECK(false, "content matches byte-for-byte");
		ok = false;
	}
	return ok;
}

static bool test_large_file(void)
{
	struct fs_file_t f;
	int res;
	bool ok = true;
	int64_t t0, dt;

	printk("\n--- 4. Large file (%d KiB) throughput + integrity ---\n",
	       LARGE_BYTES / 1024);

	fs_file_t_init(&f);
	res = fs_open(&f, LARGE_PATH, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	CHECK(res == 0, "open %s for write", LARGE_PATH);
	if (res != 0) {
		return false;
	}
	size_t written = 0;
	t0 = k_uptime_get();
	while (written < LARGE_BYTES) {
		fill_pattern(wbuf, BLOCK, written);
		if (fs_write(&f, wbuf, BLOCK) != BLOCK) {
			CHECK(false, "write chunk at offset %zu", written);
			ok = false;
			break;
		}
		written += BLOCK;
	}
	fs_sync(&f);
	dt = k_uptime_get() - t0;
	fs_close(&f);
	if (ok) {
		uint32_t kbps = dt > 0 ? (uint32_t)(((uint64_t)written * 1000U) / 1024U / dt) : 0;
		CHECK(written == LARGE_BYTES, "wrote %zu bytes in %lld ms => %u KiB/s",
		      written, dt, kbps);
	}

	fs_file_t_init(&f);
	res = fs_open(&f, LARGE_PATH, FS_O_READ);
	CHECK(res == 0, "reopen %s for read", LARGE_PATH);
	if (res != 0) {
		return false;
	}
	size_t rd = 0;
	bool mismatch = false;
	t0 = k_uptime_get();
	while (rd < LARGE_BYTES) {
		if (fs_read(&f, rbuf, BLOCK) != BLOCK) {
			CHECK(false, "read chunk at offset %zu", rd);
			ok = false;
			break;
		}
		fill_pattern(wbuf, BLOCK, rd);
		if (memcmp(rbuf, wbuf, BLOCK) != 0) {
			mismatch = true;
		}
		rd += BLOCK;
	}
	dt = k_uptime_get() - t0;
	fs_close(&f);
	if (ok) {
		uint32_t kbps = dt > 0 ? (uint32_t)(((uint64_t)rd * 1000U) / 1024U / dt) : 0;
		CHECK(rd == LARGE_BYTES, "read %zu bytes in %lld ms => %u KiB/s", rd, dt, kbps);
		CHECK(!mismatch, "%d KiB verified with no data corruption", LARGE_BYTES / 1024);
		if (mismatch) {
			ok = false;
		}
	}
	return ok;
}

static bool test_cleanup_and_remount(void)
{
	int res;
	bool ok = true;
	struct fs_dirent ent;

	printk("\n--- 5. Delete scratch files + unmount/remount ---\n");

	res = fs_unlink(SMALL_PATH);
	CHECK(res == 0, "delete %s", SMALL_PATH);
	res = fs_unlink(LARGE_PATH);
	CHECK(res == 0, "delete %s", LARGE_PATH);
	CHECK(fs_stat(SMALL_PATH, &ent) != 0, "%s no longer present", SMALL_PATH);
	CHECK(fs_stat(LARGE_PATH, &ent) != 0, "%s no longer present", LARGE_PATH);

	res = fs_unmount(&mp);
	CHECK(res == 0, "unmount");
	if (res != 0) {
		ok = false;
	}
	res = fs_mount(&mp);
	CHECK(res == FR_OK, "remount");
	if (res != FR_OK) {
		ok = false;
	} else {
		fs_unmount(&mp);
	}
	return ok;
}

int main(void)
{
	uint32_t bc = 0, bs = 0;

	k_msleep(300);

	printk("\n");
	printk("========================================================\n");
	printk(" XIAO nRF54L15  --  SD card (SPI) test\n");
	printk(" Pins: SCK=P2.01  MOSI=P2.02  MISO=P2.04  CS=P1.04\n");
	printk("========================================================\n");

	/* Wait until the card answers, so wiring / re-seating / power-cycling the
	 * card can be fixed live without re-flashing or resetting.
	 */
	int waits = 0;
	while (!card_responds()) {
		if (waits == 0) {
			printk("\nSD card is not answering (no reply to CMD0).\n");
			printk("Check connections: SCK=P2.01, MOSI=P2.02, MISO=P2.04, "
			       "CS=P1.04, plus 3V3 and GND.\n");
			printk("Re-seat / power-cycle the card. Retrying every 2 s...\n");
		}
		waits++;
		printk("[wait %ds] no card yet...\n", waits * 2);
		k_sleep(K_SECONDS(2));
	}
	if (waits > 0) {
		printk("\nSD card started answering after ~%d s.\n", waits * 2);
	}

	/* Card is up. Run the identify + full self-test and create this boot's
	 * persistent test{N}.txt = "Hello".
	 */
	test_raw_disk(&bc, &bs);

	if (test_mount()) {
		create_boot_file();       /* <-- creates test{N}.txt with "Hello" */
		test_small_file();
		test_large_file();
		test_cleanup_and_remount();
	} else {
		printk("\n*** Could not mount FAT -- is the card FAT32? ***\n");
	}

	printk("\n========================================================\n");
	printk(" RESULT: %d checks passed, %d failed\n", g_pass, g_fail);
	if (g_fail == 0 && g_pass > 0) {
		printk(" >>>>>>>>>>   SD CARD TEST: PASSED   <<<<<<<<<<\n");
	} else {
		printk(" >>>>>>>>>>   SD CARD TEST: FAILED   <<<<<<<<<<\n");
	}
	printk("========================================================\n");

	while (1) {
		k_sleep(K_SECONDS(3));
		if (g_boot_file[0] != '\0') {
			printk("[status] SD OK - this boot wrote \"Hello\" to %s\n",
			       g_boot_file);
		} else {
			printk("[status] SD not available\n");
		}
	}
	return 0;
}
