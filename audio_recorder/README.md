# Audio Recorder — XIAO nRF54L15 + SD Card

Fault-tolerant voice recorder with PDM microphone, SD card storage, and ultra-low-power sleep.

## Hardware

- **Board:** Seeed Studio XIAO nRF54L15 (Sense)
- **Microphone:** On-board PDM (PDM20, P1.12 CLK / P1.13 DIN)
- **Storage:** microSD over hardware SPI (SPIM22), FAT32
- **Button:** P0.00 (user button, active-low with pull-up)
- **LED:** P2.00

## Audio Format

- Sample rate: 16 kHz
- Bit depth: 16-bit signed little-endian (S16LE)
- Channels: mono
- Software gain: x32 (~+30 dB) with soft-knee limiter
- Data rate: ~32 KB/s, ~1.9 MB/min
- Output: raw PCM files (`rec_XXXX.raw`)

Convert to WAV:
```bash
ffmpeg -f s16le -ar 16000 -ac 1 -i rec_XXXX.raw rec_XXXX.wav
```

## How It Works

### Power-On

1. LED turns ON (solid) — device is idle
2. SD card is mounted
3. VBat voltage is read and printed
4. Device waits for button press

### Recording

1. **Button press** — recording starts, LED blinks (150ms on / 150ms off)
2. Audio is captured by PDM mic, processed with soft-knee limiter, written to SD card
3. **Button press again** — recording stops, LED returns to solid ON
4. After recording stops, device returns to idle

### Auto-Sleep

- If no button press for **10 seconds** in idle mode, device enters **System OFF** (ultra-low-power, ~1 µA)
- Wake from sleep: press the user button (P0.00)
- On wake, device performs a full reset and returns to idle state

### Fault Tolerance

Three threads decouple audio capture from SD card writes:

| Thread | Priority | Role |
|--------|----------|------|
| DMIC | 1 (highest) | PDM capture → gain → ring buffer |
| SD | 3 | Ring buffer → write buffer → SD card |
| LED | 10 (lowest) | LED status indication |

**Recovery on write error:**
1. Close and reopen file for append (fast, 200ms)
2. If that fails: unmount/remount SD card with exponential backoff (2s → 6s → 18s → 30s cap)
3. If recovery fails for >80 seconds: stop feeding watchdog → hardware reboot

**Watchdog:** 60-second hardware watchdog reboots the SoC if any thread hangs. During recovery, watchdog is fed only if recovery is making progress. Persistent failure (stuck SD card, disk full, etc.) triggers a clean reboot.

**ENOSPC handling:** If the SD card fills up during recording, the device detects `-ENOSPC` from `fs_write` and stops recording cleanly instead of looping forever.

### Heartbeat

Every 5 seconds during recording, the device prints:
```
  01:23  2560KB  drop 0KB  errs 0  ring 0  vbat 1320mV
```

| Field | Meaning |
|-------|---------|
| `01:23` | Recording time (mm:ss) |
| `2560KB` | Data written to SD card |
| `drop 0KB` | Data dropped due to ring buffer overflow |
| `errs 0` | SD write errors encountered |
| `ring 0` | Current ring buffer fill level (bytes) |
| `vbat 1320mV` | Battery voltage via ADC |

The `ring` value spikes briefly during `fs_sync()` (typically 3200 bytes = 100ms of audio) and returns to 0. Values up to 22400 (34% of 65536 buffer) are normal during longer sync operations.

## Build Configurations

### Default: USB / Debug (UART console)

```bash
west build -b xiao_nrf54l15/nrf54l15/cpuapp -p always
west flash -r openocd
```

Logs output to serial terminal (COM6, 115200 baud).

### Battery / Production (RTT only)

```bash
west build -b xiao_nrf54l15/nrf54l15/cpuapp -p always -- -DOVERLAY_CONFIG="prj_battery.conf"
west flash -r openocd
```

UART is fully disabled. Logs via Segger RTT only (requires J-Link or CMSIS-DAP with RTT support). This configuration is **required** for boot from 3.7V LiPo battery on hardware revision v1.0 (known Seeed limitation: enabled UART prevents battery-only boot).

## File Structure

```
audio_recorder/
├── src/main.c              # All application code (single-file)
├── prj.conf                # Default config (USB/UART)
├── prj_battery.conf        # Battery overlay (UART disabled, RTT enabled)
├── CMakeLists.txt          # Build configuration
├── boards/
│   └── xiao_nrf54l15_nrf54l15_cpuapp.overlay
│                           # Device tree: SPI, SD card, PDM, watchdog
└── README.md               # This file
```

## SD Card File Naming

Files are named `rec_0001.raw`, `rec_0002.raw`, etc. The index increments automatically. If a file already exists (e.g., after a power cycle), the device appends to it rather than overwriting.

## Known Limitations

- **Raw PCM format:** Files are not WAV. Need conversion with ffmpeg or VLC.
- **No file rotation:** Single file per recording session (button press to button press).
- **Fixed gain:** Software gain (x32) with soft-knee limiter. No AGC (automatic gain control).
- **Single microphone:** No stereo or spatial audio. Speaker diarization must be done server-side.
- **VBat ADC reading:** The raw voltage (~1320mV at USB power) depends on the on-board voltage divider. Absolute values are not calibrated, but relative changes are valid for monitoring battery drain.

## Future Improvements

### BLE Audio Streaming (planned)

Stream audio to a phone in real-time for server-side transcription:

```
PDM mic → DMIC thread → ring buffer ──→ SD writer (16 kHz)
                                        ──→ BLE thread (16 kHz, L2CAP CoC)
```

- BLE 5.0 with L2CAP CoC for high throughput (~60-100 KB/s)
- Phone app receives audio, forwards to server via WebSocket
- Server runs Whisper (transcription) + pyannote (speaker diarization)

**Required firmware changes:**
- Add BLE service with L2CAP CoC characteristic
- Add BLE thread that reads from a second ring buffer
- Fork audio data from DMIC to both SD and BLE ring buffers
- RAM optimization: reduce DMIC block count (16→8), reduce ring buffer (64KB→32KB), trim thread stacks

**Estimated effort:** ~150 lines of firmware code, ~300 lines of phone app, ~200 lines of server code.

### Automatic Gain Control (AGC)

Replace fixed gain (x32) with adaptive gain that normalizes volume across speakers at different distances from the microphone. Attack time ~100ms, release time ~2s.

### WAV Header Support

Write a RIFF/WAV header at file creation and backfill the size fields on clean stop. Most players will play files with incorrect/zero size headers to EOF, but a proper header improves compatibility.

### File Rotation

Split long recordings into multiple files (e.g., every 15 minutes). Limits data loss on power failure to the current segment only. Previous segments are already safely synced to the card.

### Higher Sample Rate

48 kHz recording for better voice quality and speaker differentiation (voice formants at 4-8 kHz are partially lost at 16 kHz). Increases storage requirement to ~9.6 MB/min. BLE stream would need downsampling to 16 kHz (Whisper natively works at 16 kHz).

### Server-Side Pipeline

```
Phone (BLE receiver)
  ↓ WebSocket (audio chunks every 5s)
Server (Python / FastAPI)
  ├─ Whisper large-v3          → transcript text
  ├─ pyannote/speaker-diarization-3.1  → who spoke when
  └─ output: JSON with timestamped, speaker-labeled transcript
```

**Dependencies:** `whisper`, `pyannote.audio`, `fastapi`, `uvicorn`

**Output format:**
```json
[
  {"start": 0.0, "end": 5.2, "speaker": "Speaker_1", "text": "Давайте начнём совещание"},
  {"start": 5.3, "end": 12.1, "speaker": "Speaker_2", "text": "По проекту X есть обновления"}
]
```

## Tested Scenarios

- [x] 10-minute auto-recording with verification
- [x] 112-minute continuous recording (215 MB, 0 errors, 0 drops)
- [x] Button wake from System OFF sleep
- [x] Idle timeout → System OFF → button wake → record → stop → sleep cycle
- [x] SD card recovery on transient write errors
- [x] Free space check before recording
- [x] Battery voltage monitoring
