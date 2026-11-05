# Changelog

## 2026-11-04 (firmware 1.7.0 — acknowledge frames once the bus is proven; S pin polarity)

### Fixed
- **On a port with no other listener, every frame was recorded several times.** A CAN sender resends a frame until some node acknowledges it, and the dongle only listened. A 2024 Blazer's OBD port acked nothing, so 83% of its recorded frames were back-to-back resends and the extra bus time crowded out the rest; the Impala's port shares a bus with modules that ack. Reproduced with the CAN simulator requiring acks (85% copies). The dongle now acknowledges (TWAI NORMAL) once the bus is proven; on the same bench run that gave 0 copies (69,411 frames, all unique) and 0 bus errors. See robo-services `docs/issues/2026_11_04_can_repeats_on_isolated_obd_port.md`.
- **The TJA1051 S pin was driven backwards.** NXP's datasheet (Rev. 9, 7.1): S LOW = Normal, S HIGH = Silent (transmitter off). The firmware held S LOW for "listen-only" and set it HIGH for `TX_ENABLE`, so TX mode would have silenced the transceiver and no ACK could reach the bus. Listen mode now holds S HIGH as well as TWAI LISTEN_ONLY (passive at both the controller and the transceiver), and boot starts there.

### Added
- **Ack policy.** `ACK:AUTO` (default, restored on every connect): listen, and switch to ack after 200 valid frames with fewer than one bus error per 10 of them, so a wrong bitrate never joins the bus. In ack mode, error-warning level, bus-off, or errors above that ratio fall back to listen until the next connect. `ACK:OFF` stays listen-only; `ACK:ON` acks at once. `TX_ENABLE` / `TX_DISABLE` (dbc-forge's toggle) now mean `ACK:ON` / `ACK:OFF`. Disconnect and `BAUD:` return to listen. The firmware still never transmits a frame of its own (`twai_transmit` is not called).
- **STAT gains `can_mode,can_repeats,bus_errors`.** Mode 0 listen, 1 ack, 2 listen after a fallback; repeats are frames identical to the one just before in the same ms (resends); bus errors in the 5 s window.

### Not yet done
- `android_app/app/src/main/assets/firmware/firmware.bin` (the OTA bundle) is not rebuilt for 1.7.0. dbc-forge's app parses STAT as before; the new fields are ignored.

## 2026-11-03 (firmware 1.6.0–1.6.4 — MTU-sized notifications, CAN filter, steady IMU, no lost notifications)

### Fixed
- **Notifications longer than the BLE MTU were silently cut short.** The dongle negotiated a 256-byte ATT MTU (NimBLE's default preferred MTU) but sent up to 512 bytes per notification, so everything past byte 253 was lost: under CAN load that was the IMU line (appended last) and any CAN frames after it, and a cut line could still parse with a shortened timestamp counter. Notifications are now filled only up to the negotiated MTU − 3, and the dongle asks for a 517-byte MTU (`NimBLEDevice::setMTU`); a phone that agrees gets 514-byte notifications. Found from the robo-services phone gateway, which counts cut-off notifications.
- **Lines were dropped when the buffer was full** (`appendFrame`/`appendImu` returned without flushing). A line that does not fit now flushes first; a notification the stack refuses (no buffer space) is kept and retried instead of discarded.
- **IMU rate sagged to ~26–44 Hz under CAN load.** The 50 Hz timer reset to `millis()` on each emit, so time spent draining CAN stretched every interval. It now advances by the period (resyncing if more than one period behind) and is also checked inside the CAN drain loop (1.6.1).

- **Phones lost up to 15% of notifications (1.6.4).** The buffer was flushed at the end of every loop pass: ~210 notifications/s averaging ~104 bytes. Comparing the firmware's sent count with what the phone app received over the same 5 s showed whole notifications missing in many windows although every `notify()` had succeeded. A notification is now sent when nearly full or when its oldest line is 50 ms old (`kMaxBufferAgeMs`): ~50/s at ~430 bytes, and every window then matched sent = received. On the bench (Impala CAN replay, 23-id filter) 90 of 90 seconds carried full 50 Hz IMU.

### Added
- **Diagnostics every 5 s (1.6.2):** a `[STAT]` serial line and the same counters as a `STAT,imu_lines,imu_drop,imu_events,imu_resets,imu_ready,can,can_drop,notify_fail,max_pass_ms,notifies,notify_bytes` line over NUS TX (the last two from 1.6.3), so they can be read from the phone while the dongle is on a bus (it cannot be on USB and the bench CAN simulator at once). dbc-forge's app counts the STAT line as one unparsed line per 5 s until it learns to skip it.
- **`FILTER:<hex>,<hex>,...` command** on the NUS RX characteristic: forward only those CAN ids (up to 64); `FILTER:` forwards everything again. Reset on every connect, so dbc-forge, which never sends it, always sees the whole bus. The Impala's 23 DBC ids are about half its HS-CAN frame rate.

### Not yet done
- `android_app/app/src/main/assets/firmware/firmware.bin` (the OTA bundle) is still the previous build; rebundle once 1.6.1 is confirmed on the bench.

## 2026-06-12 (session 32 — lateral/longitudinal accel discovery (11.11))

### Added
- **Lateral & longitudinal acceleration reference channels** for signal discovery, derived **kinematically with no orientation calibration**: `LONG_ACCEL` = d(GPS speed)/dt; `LAT_ACCEL` = v·yawRate (centripetal, using GPS speed + mount-tolerant IMU yaw rate). New `ReferenceChannel` entries with `refPointsFor` derivations + a GPS-speed interpolator; `availableChannels()` enables them when the needed data is present (long needs GPS; lateral needs GPS + IMU). Discover/Add flows pick them up automatically (target names `LONGITUDINAL_ACCEL` / `LATERAL_ACCEL`).

### Validated (replayed the road session offline)
- Reference channels are real and varied: lateral −4.1…+8.8 m/s² (std 0.97), longitudinal −3.5…+3.4 m/s² (std 0.90).
- **Longitudinal** surfaced several plausible powertrain IDs correlating with acceleration (0x191 r=0.70, 0xC9 r=0.69, 0x1C3/0x3D1 r=0.68) — moderate, as expected since d/dt of 1 Hz GPS speed is coarse/noisy. Worth verifying in-app.
- **Lateral** found nothing ≥0.5 — same story as yaw rate: this 2008 Impala doesn't broadcast lateral dynamics on the OBD-II HS-CAN. The tool correctly reports nothing rather than a false positive.
- Note: longitudinal fidelity is limited by 1 Hz GPS; smoothing the speed or using the (calibrated) BNO085 linear accel could improve it later.

## 2026-06-12 (session 31 — fix per-frame timestamp wrap (stray graph lines))

### Fixed
- **Non-monotonic frame timestamps → stray graph lines + degraded correlation timing.** The dongle stamps each frame/IMU line with a 16-bit `millis() % 65536` counter; the phone reconstructed per-entry times by mapping the **numerical max** to "now". That counter wraps every 65.536 s, so across a wrap the numerical max isn't the temporally-latest entry — post-wrap frames got shoved up to ~65 s into the past. Analysis of a real session showed **1141 backward time jumps of ~43–65 s** (plus hundreds of smaller ones); the in-window ones are the stray lines on the live graph, and all of them blur the discovery correlation's time alignment. Fixed with **signed circular offsets** from a pivot (entries in a packet are within a few hundred ms, far under the ±32768 ms unambiguous range), so the latest entry maps to `now` and ordering is preserved across wraps. Verified the math against a wrap-straddling packet.
- **Defensive chart hardening** (`SignalGraphScreen.drawLineChart`): draw strictly left-to-right (sort only if needed) and break the path on any non-finite value, so a stray timestamp/sample can never render a line jumping across the chart.

### Changed
- **Settings: "Recent commits" (up to 4 rows) → "Last commit" (single row).** The commit list was pushing the Pull/Push from Git actions off-screen; showing only the latest commit keeps them visible.

### Verified
- New on-device recording (~60 s, crossing a counter wrap) shows **0 backward timestamp jumps** (old data had 1141 large + thousands of small).

## 2026-06-12 (session 30 — Add-to-DBC hand-off + already-mapped indicator)

### Added
- **"Add" on each Discover hit opens the signal editor pre-filled** (Phase 11.7) — no more hand-copying `SG_` lines. The hit's bit window, endianness, signedness, factor/offset, unit, suggested name (`<CHANNEL>_<startBit>`), message name and DLC are stashed (`CanBusViewModel.SignalPrefill` / `setPendingPrefill` / `takePendingPrefill`) and seeded into `SignalEditorScreen`'s form for a new signal. Writes go through the existing editor → `DbcWriter` to the **active** DBC (one DBC per vehicle). "Copy SG_" is still there as a fallback. Add one at a time; after saving you return to the results to add the next.
- **Already-mapped indicator.** Each hit now shows whether its bits re-detect an existing signal: `≈ <SignalName> (already mapped)` (de-emphasized) vs `new in <MESSAGE>` for undecoded bits of a known message vs nothing for an unknown ID. `ScanHit` gained `overlapsSignal` (best-overlap defined signal, ≥25% of the candidate's bits) and `dlc`. This resolves the "I went from 4 to 12 speed hits and can't tell which 8 are already mapped" problem.

### Verified
- Compiles; 6 test files green (33 tests); APK installed. On-device promotion flow to be confirmed.

## 2026-06-12 (session 29 — fix frozen IMU quaternion)

### Fixed
- **Frozen IMU quaternion (firmware v1.5.2).** The orientation quaternion was constant across a whole recording while gyro/accel streamed normally. Root cause, confirmed with an LED report-streaming probe: enabling three *fused* reports at 50 Hz (rotation vector + linear accel + gyro) starves the BNO085's scheduler and the rotation-vector report dies after the first sample. Fix: run the **game rotation vector at 10 Hz** (`kImuRvReportMs`) while gyro/accel stay 50 Hz — the probe showed the report streams reliably again. Also switched plain Rotation Vector → **Game Rotation Vector** (gyro+accel fusion, no magnetometer) along the way, which is more robust in a car and still gravity-referenced (all the yaw projection / future accel-frame rotation need). Orientation changes slowly, so 10 Hz is plenty; yaw rate still comes from the 50 Hz gyro.
- This unblocks the lateral/longitudinal **accel** channels (11.6), which need live orientation. It didn't affect yaw (gravity projection ≈ raw gz at small tilt).

### Changed
- `scripts/build_and_stage_firmware.sh` paths made rename-proof (derive from the project dir, not a hardcoded `phone_companion`) — fixes staging after the `dbc-forge` rename.

## 2026-06-12 (session 28 — Discover scans known messages; first road-data validation)

### Changed
- **Discover now scans all CAN IDs, not just unknown ones.** Previously the full "Discover" sweep skipped any ID already in the DBC, so signals living in *partially-decoded* messages were invisible. It now scans known messages too but **suppresses candidates that overlap already-defined signal bits** (`SignalCorrelator.bitsCovered` made public; >50%-covered windows dropped), so it surfaces *new/undecoded* signals without re-reporting what's mapped. Hits on known IDs are tagged `in <MESSAGE> (undecoded bits)` in the results. Per-ID "Find signal (this ID)" still shows everything (incl. overlapping bits) so a misplaced definition can be corrected against ground truth.

### Validated (first real drive — 2008 Impala HS-CAN)
- Correlator confirmed against ground truth: vehicle-speed message `0x3E9` hit **r=0.999 at bit 7 / len 16 / Motorola**, exactly matching the DBC's `VehicleSpeed : 7|16@0+`. GPS-speed discovery surfaced 4 hits on the live drive.
- **Yaw rate is not on this bus.** Ran the correlator over the recorded IMU yaw (good data: −41…+31 °/s, real turns) against all 50 captured IDs — nothing above ~0.55, and those at 1.6–2.0 s lag (spurious). Notably **485/489 (the DBC's guessed `YawRate` locations) do not correlate** — consistent with those messages' own "verify with moving vehicle / may be wrong" comments. The Impala's yaw/ESP data is likely on a bus not reachable via the OBD-II HS-CAN pins.

### Known issue
- **BNO085 rotation-vector (quaternion) is frozen** in recordings — same value every sample, while gyro/accel update normally. Harmless for yaw here (the gravity projection ≈ raw gz at small mount tilt), but it must be fixed before the lateral/longitudinal **accel** channels (11.6), which depend on live orientation. To investigate: whether `enableRotationVector` is being starved/disabled or `getQuat*` is returning a stale cache.

### Note
- Project directory renamed `phone_companion` → `dbc-forge`; updated `android_app/local.properties` `sdk.dir`. CLAUDE.md tool-path table and `scripts/` still reference the old name — update when convenient.

## 2026-06-12 (session 27 — fix git sync from the phone)

### Fixed
- **Git push from the phone never worked — "merge conflict during sync".** Root cause: `GitRepository.sync()` did stash → pull → stash-apply, but JGit's `stashCreate()` **silently drops untracked files**. New DBCs / vehicle profiles / sidecars created on the phone are untracked, so the stash captured nothing, `stashApply()` then failed, and that failure was mis-reported as a "merge conflict" — followed by a hard reset, so the push never ran. Re-cloning/pulling never helped because the next edit hit the same path.
  - **Fix:** replaced the stash dance with a **commit-first** flow — stage (new/modified **and** deletions) → commit (captures untracked files correctly) → fetch + merge the remote → push. Content conflicts resolve in favour of the phone's edit (`ContentMergeStrategy.OURS`) since the user just made it intentionally; non-conflicting remote changes still merge in; only an unresolvable structural conflict surfaces an error (and the local commit is preserved, nothing lost).
- Commits now set an explicit committer identity (not just author).

### Verified
- New `GitRepositoryTest` exercises the real flow against a local bare remote: (1) **a brand-new untracked file pushes cleanly** — the exact bug; (2) a non-conflicting remote change merges and both sides survive; (3) a content conflict resolves to the phone's version on the remote. Full suite green (33 tests). APK installed.

## 2026-06-12 (session 26 — fix top-bar crowding; Dongle status screen)

### Fixed
- **Record/Stop button was being squeezed out of the live view.** The live IMU/GPS status text added in session 24/25 was packed into the same Row as the filter chips and the `weight(1f)` box holding Record/Stop; once connected/recording the unbounded text consumed the row and collapsed the button to zero width (it rendered as an empty tall pill, and Stop vanished entirely while recording). Removed the inline status text from that row — the action row is now just `Known | Unknown | Diag | … | Record/Stop` and the button always has room.

### Changed
- **Firmware screen → "Dongle" status screen.** Repurposed `FirmwareUpdateScreen` (Settings tile renamed "Firmware Update" → "Dongle") into a consolidated device view: connection (name/RSSI/fps), live **IMU** (mount-tolerant yaw rate + accel magnitude), live **GPS** (speed/accuracy), and the existing firmware version + update. This is the new home for the live sensor fields — glanceable for a pre-drive sensor check without crowding the live view.
- **GPS preview** — added `CanBusViewModel.setGpsPreview()` so the Dongle screen streams GPS for display while open (stops on exit unless a recording owns the source), letting you confirm a fix before driving rather than only during a recording.

### Verified
- Compiles; 30 unit tests green; APK installed. On-device: live view shows a properly-sized Record button when connected; the Dongle screen shows live IMU (ω/accel) and GPS status + firmware 1.5.0 "up to date".

## 2026-06-12 (session 25 — yaw-rate channel + discover-all workflow)

### Added
- **Yaw-rate correlation channel (mount-tolerant).** `ImuSample.verticalYawRateDegPerSec` projects the gyro vector onto the gravity direction (from the fusion quaternion), recovering the true vertical-axis yaw rate regardless of how the dongle is tilted in the OBD-II port — **no calibration step**. Added `ReferenceChannel` (GPS_SPEED, IMU_YAW_RATE) so the correlator runs against either; yaw rate targets `YAW_RATE`/`STEERING_ANGLE`. Unit test (`ImuSampleTest`) proves a 90°-tilted mount still reads vehicle yaw correctly.
- **Discover-all workflow (hands-off review).** New **"Discover"** button in the log browser runs the correlator across **every unknown CAN ID against every recorded channel** (speed + yaw rate) in one pass, with a progress indicator, and presents a single ranked list (`DiscoveryRun`/`ScanHit`) tagged by CAN ID + channel, each with a **Copy SG_** line. The per-ID long-press action ("Find signal (this ID)") now also runs all channels. This is the "drive, then review at the end" step — no need to investigate IDs one at a time.
- **Keep-screen-on while recording.** LiveScreen holds the screen awake during a recording so a solo, hands-off drive isn't interrupted by the app being backgrounded when the screen sleeps.

### Verified
- `compileDebugKotlin` clean; `testDebugUnitTest` green (30 tests, incl. 2 new yaw-projection cases); APK builds and installs. On-road discovery quality pending a real drive with turns + speed changes.

### Notes
- The OBD-II mount angle is a non-issue for **yaw** (gravity projection handles it). The **lateral/longitudinal accel** channels still need full orientation calibration (11.6) — they need horizontal heading, not just vertical.

## 2026-06-12 (session 24 — Phase 11.3 BNO085 IMU ingestion)

### Added
- **Firmware v1.5.0 — BNO085 IMU on the dongle.** Integrated the `SparkFun_BNO08x` library into `esp32_can_ble_bridge`: I2C @ `0x4B` (AD0 strapped high), SDA=D4/SCL=D5, 100 kHz (clock-stretch), **poll mode** (INT=-1 so `getSensorEvent()` never stalls the TWAI/NimBLE loop). Rotation vector + linear accel + gyro enabled at 50 Hz; cached and emitted as a combined `IMU,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,<ts>` NUS line at 50 Hz, sharing the existing `millis()%65536` per-line timestamp scheme so IMU and CAN align on one clock.
- **Android IMU pipeline.** `ImuSample` model; `parseImuLine` splits `IMU,` lines in `handleNotification` (frames and IMU share one packet `maxFwTs` for timestamp reconstruction); routed via `imuChannel` and drained in the frame processor. `imuStatus` StateFlow drives a live "IMU ω=±N°/s a=…" readout in LiveScreen (updates whenever connected — move the dongle to see yaw rate change). Recording writes `imu.log`; `SessionMeta.imuSampleCount` shows in the log header; `LogPlayerViewModel` loads it (`imuSamples`).
- **Bundled OTA firmware restaged to v1.5.0** (`assets/firmware/firmware.bin`).

### Fixed
- **IMU `begin()` is now retried from `loop()` instead of once in `setup()`.** A single-shot `imu.begin()` raced the BNO085's power-on boot and silently failed → no IMU on the dongle (the LED-flicker probe had masked this by retrying every second). `tryInitImu()` now retries at 1 Hz until the sensor answers; this was the actual fix that got IMU streaming to the phone.

### Verified
- **End-to-end confirmed on device.** Phone connected to `CAN-DONGLE` over BLE; LiveScreen shows the live `IMU ω=±N°/s a=…` readout and it tracks dongle movement (yaw rate swings, accel reacts). BNO085 bring-up also confirmed on-bench via an LED-flicker probe (sensor at 0x4B, all three reports streaming). Firmware 55% flash; Android app builds clean; APK installed via adb.

### Notes
- Captured two reusable gotchas: `docs/hardware/bno085_9dof_imu.md` now has the verified C6 config + poll-mode rationale; `docs/tooling/arduino-cli.md` documents the C6 USB-serial silence (`cdc_on_boot=0` → Serial on UART0; HWCDC drops TX with no host) — use a heartbeat/LED for headless bring-up.
- Remaining Phase 11: 11.6 orientation calibration (rotate IMU into vehicle frame) to unlock lateral/longitudinal accel correlation; gyro-Z yaw-rate correlation can come sooner since it's roughly orientation-tolerant for a flat-mounted dongle.

## 2026-06-12 (session 23 — Phase 11 synthetic validation + scope trim)

### Added
- **Synthetic end-to-end validation for the correlator** — `SignalCorrelatorTest` (4 cases) fabricates a "drive": a known speed profile encoded into a frame bit field plus a matching GPS speed channel, and asserts the engine (1) recovers the field and a scale that **decodes back to true speed** within ~1 km/h, (2) recovers a deliberate **400 ms GPS clock skew** via lag search (r>0.99 only reachable if it shifts to ~the true lag), (3) returns `NO_MATCH` for an unrelated monotonic field vs a symmetric speed hump, and (4) reports `REFERENCE_STATIC` for constant speed. Full unit suite green.

### Changed
- **Correlator ranking now prefers the widest window on near-ties** (`SignalCorrelator`). Candidate correlations are bucketed to ~0.001 and, within a bucket, the longer field wins the overlap-suppression contest. A sub-slice that correlates as well as the full field is a less complete and less accurately-scaled decode, so it should lose — this surfaces the true field instead of an off-by-N fragment. (Found via the validation harness.)
- **Dropped the external/USB GPS path — phone GNSS only.** Per decision, no module gets plugged into the phone. ROADMAP 11.2 retired, hardware note + `sensor_correlation_plan.md` updated; `docs/hardware/adafruit_ultimate_gps.md` retained as a repo-wide reference but marked not-used-here. A dongle UART GPS remains a future fallback if 1 Hz proves too coarse.

### Verified
- `testDebugUnitTest` passes (28 tests, incl. the 4 new correlator cases). On-vehicle accuracy still pending a real drive, but the decode/scale/lag math is now validated against synthetic ground truth.

## 2026-06-12 (session 22 — Phase 11 GPS correlation build: 11.1, 11.4, 11.5, 11.7)

### Added
- **GPS reference channel capture (Phase 11.1).** New `GpsSource` wraps the framework `LocationManager` GPS_PROVIDER directly (no Google Play Services dependency, works on non-GMS devices). During a recording session it streams Doppler speed-over-ground / course / accuracy into a per-session `gps.log` sidecar (frames.log stays pure CAN). `GpsSample` model + `SessionMeta.gpsSampleCount`. Started/stopped with recording in `CanBusViewModel`; stopped in `onCleared`.
- **Brute-force signal correlator (Phase 11.4/11.5).** New `SignalCorrelator` sweeps candidate bit fields `(startBit, length∈{8,12,16}, Intel/Motorola, signed/unsigned)` over one CAN ID's frames, zero-order-hold-resamples onto a uniform grid alongside the linearly-interpolated reference, runs a ±2 s **lag-search** cross-correlation (absorbs the phone↔dongle clock skew + BLE jitter), and for matches above |r| ≥ 0.8 fits raw→physical by least squares to infer `factor`/`offset`. Window-fit and variance guards; overlapping windows suppressed; results ranked by |r|. Reuses `SignalDecoder.extractRaw`.
- **Candidate review UI (Phase 11.7).** Long-press an unknown frame in the log browser → "Find signal (GPS speed)" runs the correlator on the offline log path (`LogPlayerViewModel.runSpeedCorrelation`). Results dialog lists ranked candidates (bit window, signed r, lag, inferred scale) each with a "Copy SG_" button that puts a ready-to-paste DBC `SG_` line on the clipboard. Status messages for no-GPS / static-speed / no-overlap / no-match cases.
- **Live GPS feedback.** LiveScreen shows a "GPS N km/h" / "GPS fix…" / "GPS no fix" status while recording; session header shows the recorded GPS-fix count.

### Changed
- **`ACCESS_FINE_LOCATION` no longer capped at SDK 30.** It's now needed at runtime on all SDKs for the GPS reference channel (previously `maxSdkVersion=30`, BLE-scan-only). `MainActivity` requests it alongside the Bluetooth permissions. `BLUETOOTH_SCAN` keeps `neverForLocation`.
- **Deviation from plan:** used the framework `LocationManager` instead of `FusedLocationProvider` to avoid a Google Play Services dependency — `Location.getSpeed()` already exposes the GNSS Doppler speed we correlate against. Docs updated to match.

### Verified
- `assembleDebug` builds the debug APK clean. On-vehicle correlation accuracy not yet validated (needs a recorded drive with speed variation).

### Notes
- Phase 11.7 stops short of one-tap signal-editor prefill (cross-VM navigation) — the clipboard `SG_` line is the v1 promote path. IMU ingestion (11.3), orientation calibration (11.6), and the USB-GPS bridge-chip check (11.2) remain.

## 2026-06-12 (session 21 — IMU + GPS correlation planning)

### Added
- **Roadmap Phase 11 — Sensor-aided signal discovery (IMU + GPS correlation).** LibreCAN-style auto-identification of unknown CAN signals: record a ground-truth physical reference (speed/yaw/accel) alongside CAN, brute-force candidate bit-field decodings of unknown IDs, and rank by Pearson correlation with the matching reference channel — reusing the Phase 7.3 OBD-II cross-reference engine. Added rows 11.1–11.7 plus an "onboard sensors" hardware note and firmware backlog F.8/F.9.
- **`docs/sensor_correlation_plan.md`** — full design: architecture (IMU on dongle for single-clock alignment, GPS on phone with lag-search alignment), reference-channel → CAN-target map, brute-force candidate-decode + lag-search + scale-inference algorithm, data-model additions, and the sensor hardware decision.
- **`docs/hardware/bno085_9dof_imu.md`** — BNO085 reference: SH-2 fusion reports, PS1/PS0 strapping, I2C 0x4A @ 100 kHz clock-stretch gotcha, UART-RVC fallback. Recommended IMU (on hand) for the dynamic channels.
- **`docs/hardware/adafruit_ultimate_gps.md`** — Ultimate GPS USB reference: MTK3339 10 Hz, NMEA/PMTK config, and the USB-host constraint (C6 is USB-device-only and cannot host it → must attach to the phone via OTG). Documents why no F9P is needed (correlation uses velocity, not position).

### Decision
- **GPS:** start with the phone's internal `FusedLocationProvider` (free, Doppler speed), upgrade to the on-hand Adafruit Ultimate GPS USB via phone OTG for 10 Hz. **No ZED-F9P purchase** — RTK position accuracy is irrelevant to velocity/yaw correlation. **IMU:** use the on-hand Adafruit BNO085 on the dongle (on-chip fusion solves the mounting-orientation problem).

## 2026-06-11 (session 20 — C6-only platform cleanup)

### Changed
- **Firmware: removed S3 board support** — platform is now C6-only. Removed the commented S3 pin block and board-selection instructions from `esp32_can_ble_bridge.ino`; header comment updated to reference only the C6. `partitions_s3.csv` left in place for reference but is no longer used.
- **Bundled OTA firmware rebundled for C6** — `assets/firmware/firmware.bin` was previously compiled against `XIAO_ESP32S3` (different pins, different partition table, would have bricked a C6 on OTA). Recompiled with `XIAO_ESP32C6` FQBN and replaced the bundled binary. Version remains 1.4.0.
- **CLAUDE.md example flash command** updated from `XIAO_ESP32S3` to `XIAO_ESP32C6`.

## 2026-06-11 (session 19 — UI stability fixes: BLE picker and Unknowns screen)

### Fixed
- **BLE device picker: list no longer jumps while scanning** — `onScanResult` was calling `.sortedByDescending { it.rssi }` on every scan callback, reordering the list every time RSSI updated (which is continuous). `deviceMap` is a `LinkedHashMap` so its insertion order is stable (first-seen device stays at the top). Removed the sort; RSSI is still displayed per-device as signal strength info. Devices now appear and stay in first-seen order for the duration of a scan.
- **Unknowns screen: rows no longer bounce** — `processBatch` was emitting `freshUnknown.sortedByDescending { it.updateRateHz }`, causing every CAN frame batch to potentially reorder the entire list. Changed to `sortedBy { it.id }` (ascending CAN ID). The list is now deterministic — a row is always at the same position once it appears, so it can be tapped without first hitting Pause.

## 2026-06-11 (session 18 — XIAO ESP32-C6 firmware support)

### Changed
- **Firmware: XIAO ESP32-C6 pin support** — added commented toggle block in `esp32_can_ble_bridge.ino` so the firmware can be quickly flipped between XIAO ESP32-S3 and XIAO ESP32-C6 targets. C6 active, S3 commented out. Pin differences: CAN TX D0=GPIO0, CAN RX D1=GPIO1, S-pin D2=GPIO2, LED=GPIO15 (vs S3: GPIO1/2/3/21). FQBN for C6: `esp32:esp32:XIAO_ESP32C6`.
- **Partition tables split by board** — `partitions.csv` was sized for the S3's 8 MB flash; the C6 only has 4 MB, causing a boot loop ("partition 3 invalid — exceeds flash chip size"). Added `partitions_s3.csv` (8 MB, two 3.3 MB OTA partitions) and `partitions_c6.csv` (4 MB, two 1.75 MB OTA partitions). `partitions.csv` is the active copy — `cp partitions_c6.csv partitions.csv` or `cp partitions_s3.csv partitions.csv` before compiling for the matching board. C6 confirmed advertising as `CAN-DONGLE` (MAC 58:E6:C5:14:F5:BA) after reflash.

## 2026-06-11 (session 17 — Impala DBC signal fixes)

### Fixed (dbc_companion repo, `chevy_impala_2008_hscan.dbc`)
- **0xC1 gmlan_sensors_1**: renamed message (spaces in name broke DBC parsers); replaced bogus `engine coolant temp : 0|11@1+` signal (was capturing rolling counter bits, decoding ~264°C) with correct `RollingCounter : 4|2@1+`. Actual coolant temp is in 0x4C1 `ECMEngineCoolantTemp`.
- **0x1A1 AcceleratorPedalPos**: start bit corrected from 48 → 0 (bit 48 is out-of-bounds for a 3-byte DLC frame; signal decoded 0 silently before the fix).
- **0x1E5 SteeringWheelAngle**: start bit corrected from 7 → 15 (B0 is a status byte; old position read it as the MSB of a signed 16-bit value, decoding −2016° while parked; B1–B2 = 0x0000 parked is correct).
- **0x34A RLWheelDir / RRWheelDir**: removed (bit positions 37 and 34 are out-of-bounds for the 4-byte DLC; both decoded 0 silently). Need to confirm actual positions with a moving-vehicle recording before re-adding.
- Added NOTE comments on `YawRate` (decodes ~64 deg/s parked, start bit 51 suspect) and `AmbientTemp` (decodes −34°C from cold recording, byte position unverified) flagging them for running-engine verification.
- Corrected mislabelled comment on 0x1265 (was "Engine oil and ambient temperatures", now "Fuel rail pressure").

All fixes identified via Python static analysis against a key-on engine-off recording (`frames_54e71e98.log`, 2008 Chevy Impala).

## 2026-06-11 (session 16 — diag frame persistence + review mode diag support)

### Fixed
- **Live view: diag frames no longer scroll off screen** — OBD2 frames (0x7DF, 0x7E0–0x7EF) were going into the shared 200-slot rolling `liveBuffer`, where they'd be pushed out within seconds on a busy CAN bus. They now accumulate in a separate `diagAccum` (cap 500) that is never trimmed by regular traffic. `LiveScreen` merges `liveFrames` and `diagFrames` by `seq` order when `showDiag` is on, so diag codes pile up chronologically for the entire session.
- **Review mode: diag frames now visible and filterable** — `LogPlayerViewModel.visibleFrames` previously only checked `showKnown`/`showUnknown`, lumping diag frames into the Unknown bucket. Added `_showDiag: StateFlow<Boolean>` and a 4-way `combine` so diag frames are independently toggled. `LogBrowserScreen` gained a "Diag" FilterChip. `LogFrameRow` now detects OBD2 IDs, renders them in `ColorDiag` (purple), and formats them via `formatObd2Frame` (same Mode-01 decode logic as the live view: REQ/RSP prefix, service, PID, decoded value where available).

## 2026-06-09 (session 15 — starter DBC assets)

### Added
- **Starter DBC bundle** — Added `app/src/main/assets/starter_dbcs/` containing 8 real-world DBC files sourced from the comma.ai opendbc repository, plus a `LICENSE` file (MIT, Comma.ai). Files included: `acura_ilx_2016_nidec.dbc` (5.1 KB), `gm_global_a_lowspeed.dbc` (3.3 KB), `ford_fusion_2018_pt.dbc` (4.8 KB), `bmw_e9x_e8x.dbc` (24.6 KB), `mazda_2017.dbc` (28.7 KB), `tesla_model3_vehicle.dbc` (23.8 KB), `hyundai_i30_2014.dbc` (22.8 KB), `toyota_2017_ref_pt.dbc` (66.6 KB). These give users real DBC definitions to explore on first launch without needing to import their own file. Note: no Honda or Subaru DBC files exist in the opendbc repo at this time; those slots were filled with the closest available alternatives (Hyundai i30 2014 and Toyota 2017 ref PT).

## 2026-06-09 (session 14 — F.3 baud rate, F.4 bus-off recovery, OTA MTU fix)

### Fixed
- **OTA MTU negotiation** — `onConnectionStateChange` now calls `gatt.requestMtu(517)` before service discovery instead of proceeding at the BLE default (23-byte) ATT MTU. `onMtuChanged` stores the negotiated MTU and then calls `discoverServices()`. OTA chunk size is now computed as `(negotiatedMtu - 3).coerceIn(20, 512)` instead of a hardcoded 512, preventing GATT error 133 on phones that don't auto-negotiate a high MTU. This was the primary risk for bricking a device on a day-1 OTA update.

### Added
- **F.3 — Runtime baud rate (firmware + Android)** — Firmware `v1.4.0` parses `BAUD:<rate>` commands on the NUS RX characteristic. Supported rates: 125000, 250000, 500000, 1000000. `reinitCan()` selects the appropriate `TWAI_TIMING_CONFIG_*` macro from `gReinitBaudRate`; reinit preserves current TX-enabled state. Android: `setBaudRate(baud)` in `CanBusViewModel` sends the NUS command, updates `_activeBaudRate` StateFlow, and persists to SharedPreferences (`can_baud_rate`). Baud rate selector added to `SettingsScreen` as a `FilterChip` row (125k / 250k / 500k / 1M); shows live selection, applies immediately when connected.
- **F.4 — Bus-off recovery (firmware)** — `loop()` calls `twai_get_status_info()` every 500 ms. On `TWAI_STATE_BUS_OFF`, triggers `reinitCan(gTxEnabled)` with a 5-second cooldown between recovery attempts. Prevents the dongle from going permanently silent after a bus error burst.

### Changed
- Firmware bumped to **v1.4.0**; bundled binary and `assets/firmware/version.txt` updated to match.

## 2026-06-09 (session 13 — Phase 8 Generic Signal Graph)

### Added
- **`CanBusViewModel` Phase 8 backend** — `_pinnedSignalKeys: MutableStateFlow<List<String>>`, `_signalSeries: MutableStateFlow<Map<String, List<Pair<Long,Double>>>>`, `_thresholds: MutableStateFlow<Map<String,Double>>`, `_thresholdAlerts: MutableSharedFlow<Pair<String,Double>>` (capacity 8). Public methods: `pinSignal(key)`, `unpinSignal(key)`, `setThreshold(key, value)`, `clearThreshold(key)`. Constants: `MAX_PINNED_SIGNALS = 4`, `GRAPH_MAX_WINDOW_MS = 60_000L`, `GRAPH_BUFFER_SLOTS = 6_000`.
- **Rolling series accumulation in `processBatch()`** — For each decoded signal value, if the key is in `pinnedNow`, appends `(timestampMs, value)` to an `ArrayDeque` in `signalSeriesData`. Keeps the last 6000 points and drops points older than 60 s. Emits a snapshot to `_signalSeries` every batch (only when not frozen).
- **Threshold crossing detection in `processBatch()`** — Compares current side (above/below) to previous; on a crossing, enqueues `(key, value)` in a per-batch `crossings` list, then `tryEmit`s each to `_thresholdAlerts` from `Dispatchers.Main`.
- **`SignalGraphScreen.kt`** (new) — Full-screen rolling chart view. Window selector chips (5/15/30/60 s). Individual `SignalChartCard` per pinned signal: Canvas line chart with 25/50/75 % grid lines, dashed threshold line, threshold dialog (set/clear), min/max/current readout strip, Y-axis labels. Multi-signal `OverlayChartCard` with normalized traces and color legend when "Overlay" is toggled. Empty-state message when nothing is pinned. `SnackbarHost` at bottom; `LaunchedEffect` collects `thresholdAlerts` and shows `"⚠ <sigName> crossed threshold: <value>"` snackbar.
- **Pin/unpin button in `SignalRow`** (`SignalsScreen.kt`) — `Icons.AutoMirrored.Filled.ShowChart` icon button; tinted primary when pinned, dim when not. Tapping calls `vm.pinSignal` / `vm.unpinSignal` via `onTogglePin` callback threaded through `MessageRow`.
- **"Graph (N signals pinned)" navigation banner** (`SignalsScreen.kt`) — Appears between the health warning banner and the search bar whenever `pinnedSignalKeys` is non-empty. Tapping calls `onGraphScreen()`.
- **`signal_graph` route in `AppNavigation.kt`** — `composable("signal_graph")` renders `SignalGraphScreen`; `SignalsScreen` receives `onGraphScreen = { navController.navigate("signal_graph") }`.

### Why
- Rolling chart enables watching a signal evolve in real time — useful for confirming a newly decoded signal behaves as expected (e.g., correlates with pedal movement) and for threshold-based spotting of anomalies.
- Overlay mode lets you visually correlate up to 4 signals without needing to export to a desktop tool.

## 2026-06-09 (session 12 — Phase 7.3 Native ↔ OBD-II cross-reference)

### Added
- **`OdbCrossRef` data class** — Public state type: `signalKey` (`"msgName/sigName"`), `pid`, `pidName`, `correlation` (Pearson r), `sampleCount`.
- **`obdCrossRefs: StateFlow<Map<String, OdbCrossRef>>`** in `CanBusViewModel` — emits confirmed matches (|r| ≥ 0.85) keyed by `"msgName/sigName"`. Updated unconditionally (not subject to freeze).
- **Pearson correlation engine in `processBatch()`** — For each Mode 01 OBD-II response frame (SVC `0x41`) in a batch, decodes the PID value and samples all currently-decoded native signals. Accumulates up to 30 `(obd_value, native_value)` pairs per `(pid, signalKey)` in `crossRefObdSamples`/`crossRefNativeSamples`. After ≥ 10 samples, computes Pearson r; if |r| ≥ 0.85, promotes to `_obdCrossRefs`. Entry removed if r drops below threshold. Buffers cleared on DBC change and on BLE disconnect.
- **"OBD ✓" badge in `SignalRow`** — Green monospace label `"OBD ✓  Engine RPM  r=0.97"` shown below the signal name when `obdCrossRefs` contains a confirmed entry for that signal. Correlation coefficient shown for transparency.

### Why
- Scale-invariant correlation handles the common case where a native signal is a raw count at a different scaling factor than the OBD-II physical value (e.g., raw ticks → rpm). The Pearson r approach finds matches without requiring unit knowledge.
- Stuck-signal guard: if either signal is constant, the denominator is ≈0 and r is returned as 0, preventing false positives on parked-vehicle data.

## 2026-06-09 (session 11 — Phase 7.2 OBD-II PID decoder + app scope decision)

### Added
- **`Obd2PidTable` (Phase 7.2)** — `data/obd2/Obd2PidTable.kt`. 63-entry hardcoded J1979 Mode 01 PID table covering: engine RPM, vehicle speed, coolant/intake/ambient/oil temps, MAF, throttle/accel pedal positions, fuel trim (STFT/LTFT), fuel level/pressure/rate, MAP, timing advance, O2 sensors (banks 1–2, sensors 1–8), catalyst temps, barometric pressure, EGR, run time, MIL distance/time, module voltage, torque, ethanol content. Each entry: `pid`, `name`, `unit`, `minBytes`, `decode: (ByteArray) -> Double`.
- **Mode 01 response decoding in live view** — `formatObd2Frame()` in `LiveScreen.kt` now detects SVC `0x41` responses and calls `Obd2PidTable.decode(pid, valueBytes)`. Decoded frames show as `RSP1  SVC:41 PID:0C  Engine RPM: 1726.25 rpm` instead of raw hex. Falls back to raw hex for unknown PIDs. Non-Mode-01 frames unchanged.

### Changed
- **App scope narrowed** — Active OBD-II diagnostics (PID polling, DTC read/clear, ISO-TP, freeze frame, Mode 06) moved to a future separate diagnostic app. `phone_companion` stays a DBC tool. Phase 8 redesigned as a generic signal graph; old O2-specific monitor and active polling phases removed from roadmap.

## 2026-06-09 (session 10 — Android NUS RX plumbing + Phase 7.1 OBD-II frame detection)

### Added
- **NUS RX write channel in `CanBusViewModel`** — Discovers the `6E400002` write characteristic in `onServicesDiscovered` and stores it as `nusCmdChar`. `enableTx()` sends `TX_ENABLE`; `disableTx()` sends `TX_DISABLE` via a new `writeNusCommand()` helper (uses `WRITE_TYPE_NO_RESPONSE`). `_txEnabled: MutableStateFlow<Boolean>` tracks the current TX state; resets to `false` on both user-initiated and unexpected disconnect.
- **TX active indicator in top bar** — A green `"TX"` badge appears in the `TopAppBar` actions area when `txEnabled == true`. Implemented by collecting `canBusVm.txEnabled` in `AppNavigation`.
- **Phase 7.1 — Diagnostic frame detection** — `CanBusViewModel.isObd2Diagnostic(id)` returns `true` for `0x7DF` and `0x7E0–0x7EF`. `_showDiagInLive: MutableStateFlow<Boolean>` (default `true`) and `setShowDiag()` added; OBD-II frames are now a separate bucket from Known/Unknown in the live filter.
- **"Diag" filter chip in `LiveScreen`** — Third `FilterChip` alongside Known/Unknown. OBD-II frames (identified by ID) are shown in light purple and formatted with `formatObd2Frame()`: `REQ SVC:01 PID:0C` for requests (`0x7DF`/`0x7E0–0x7E7`), `RSP1 SVC:41 PID:0C  XX XX XX XX` for responses (`0x7E8–0x7EF`). Request/response prefixes include ECU index for responses.

### Why
- The NUS RX channel is the control path for all future TX-mode features (Phase 7.2+). Making it solid now — with auto-revert on disconnect — means every future OBD-II polling screen can use `enableTx()`/`disableTx()` via `DisposableEffect` without touching the BLE layer again.
- Diagnostic frame detection (7.1) requires no hardware and lets us validate which OBD-II IDs are active on the bus before writing any polling code.

## 2026-06-09 (session 9 — Firmware v1.3.0: F.1/F.2 NUS RX + S-pin GPIO control)

### Added
- **NUS RX write characteristic (F.1)** — UUID `6E400002` added to the NUS service. Accepts UTF-8 commands from the phone. Currently recognised: `TX_ENABLE` and `TX_DISABLE`.
- **S-pin GPIO control (F.2)** — `kCanSPin = GPIO_NUM_3` (D2). Driven `LOW` in `setup()` before any CAN or BLE init — transceiver is in listen-only mode from the very first millisecond. `TX_ENABLE` drives the pin `HIGH` and reinits TWAI in `TWAI_MODE_NORMAL`; `TX_DISABLE` drives it `LOW` and reinits as `TWAI_MODE_LISTEN_ONLY`.
- **Auto-revert to listen-only on disconnect** — `ServerCallbacks::onDisconnect` schedules `TX_DISABLE` via the deferred reinit flags, so the bus is always safe when no phone is connected.
- **Deferred reinit pattern** — BLE callbacks set `volatile bool gReinitRequested` + `gReinitTxTarget`; `loop()` applies the reinit at a safe point (between TWAI receive calls) to avoid driver races.
- **Firmware staged for OTA** — `firmware.bin` and `version.txt` (`1.3.0`) updated in Android assets.

### Changed
- **Firmware version → 1.3.0**
- **`kFirmwareVersion`** bumped; `Serial` startup log now prints S-pin state.
- **`initBle()`** — NUS RX characteristic added before NUS TX in service init order.
- **Hardware comment in file header** updated to document D2 (GPIO3) S-pin wiring.

### Hardware
- S-pin pulldown resistor removed from TJA1051 breakout.
- S-pin wired to XIAO ESP32-S3 D2 (GPIO3).

## 2026-06-09 (session 8 — Phase 6 completion: sparklines, correlation plot, playback mode)

### Added
- **Signal sparklines (Phase 6.2)** — `LogPlayerViewModel` computes a `List<SparklineSeries>` immediately after loading frames. Each series samples 200 evenly-spaced time slots across the session and normalises values to `[0, 1]`. Rendered as a horizontally scrollable `LazyRow` of `SparklineCard` composables (110×~60dp each) above the scrubber. Each card shows signal name, message name, and a Canvas line chart. Tapping a card toggles that signal's selection for correlation (up to 4). Selected cards are highlighted with a colored border + tinted background. Color palette shared with the correlation plot.
- **Multi-signal correlation plot (Phase 6.5)** — "Plot (N)" button appears in the filter chip row when any sparkline is selected (or `sparklines` non-empty for first-time picker). Tapping opens `SignalPickerDialog` — a checkbox list of all decoded signals with colored dots. After confirming, a full-screen `Dialog` shows `CorrelationPlotDialog`: a Canvas-based multi-line plot with one Y-scale per signal (independent scaling), horizontal grid lines, and a time axis footer. Signal legend at top shows name + color for each selected signal. Y-axis labels show max/min for each signal. Plot rendered from `allFrames` filtered to the current scrubber range.
- **Playback mode (Phase 6.6)** — Play/pause `IconButton` added to the left of the scrubber row. When playing, a coroutine in `LogPlayerViewModel` advances `_rangeEnd` by `wallElapsed × playbackSpeed` every 16 ms. Playback stops automatically when `_rangeEnd` reaches `1f` or when the user drags the scrubber. Restarts from current `_rangeEnd` on play (or from `_rangeStart` if already at the end). Speed selector `TextButton` (right of scrubber) offers 0.25×, 0.5×, 1×, 2×, 4× via a `DropdownMenu`.

### Changed
- **`LogPlayerViewModel`** — Added `SparklineSeries` data class, `sparklines` / `selectedSignalKeys` / `isPlaying` / `playbackSpeed` StateFlows, `startPlayback()` / `stopPlayback()` / `setPlaybackSpeed()` / `toggleSignalSelection()` / `clearSignalSelection()` methods, `computeSparklines()` private function. `loadSession()` resets all new state and computes sparklines after frame load. `MAX_CORRELATION_SIGNALS = 4` exposed as companion constant.
- **Phase 6 marked complete** in roadmap.

## 2026-06-09 (session 7 — Phase 6.1/6.3/6.4: timeline scrubber, reference anchor, stimulus-response diff)

### Added
- **Timeline range scrubber (Phase 6.1)** — `RangeSlider` (Material3, two-thumb) pinned to the bottom of `LogBrowserScreen`. Dragging the thumbs sets a normalised `[0, 1]` window in `LogPlayerViewModel`; `visibleFrames` is re-filtered reactively. Label row above the slider shows session start (`00:00`), total duration, and selected sub-range when non-trivial. Scrubber only renders once the session has loaded.
- **Reference event anchor (Phase 6.3)** — Long-press any frame row in `LogBrowserScreen` opens a context menu with "Set as T=0 reference". When set, all frame timestamps switch from `+MM:SS.mmm` (session-relative) to `T+N.Ns` / `T-N.Ns` (reference-relative). An amber star chip (`★ T=0`) in the status row clears the reference on tap.
- **Stimulus-response diff (Phase 6.4)** — Long-press any frame row → "Mark as A" or "Mark as B" bookmarks that moment. When both are set, a "Diff (N changed)" button appears in the filter chip row. Tapping opens `DiffDialog` listing all decoded signals sorted by magnitude of change: signal name (monospace), message name, values at A and B (`a → b`), and a `↑`/`↓` delta indicator. Changed signals are highlighted in `ColorActive`; "appeared" / "disappeared" signals handled. The 2 s snapshot window is shared with the ViewModel's `SNAPSHOT_WINDOW_MS` constant. Bookmark chips (blue for A, green for B) appear below the filter chips and can be cleared individually.
- **`SignalDiff` data class** in `LogPlayerViewModel` — `signalName`, `messageName`, `valueAtA?`, `valueAtB?`, computed `delta?` property.
- **`allFrames` public StateFlow** on `LogPlayerViewModel` — gives `LogBrowserScreen` the full unfiltered list for scrubber endpoint computation.
- **`clearBookmarkA()` / `clearBookmarkB()`** on `LogPlayerViewModel` — clear individual bookmarks and reset diffResult without touching the other bookmark.

### Changed
- **`visibleFrames` in `LogPlayerViewModel`** — updated from a 3-flow `combine` to a nested 3+3 combine that also filters by the selected time range before the known/unknown filter runs. Range is based on the first/last frame timestamps in `_allFrames`.
- **`loadSession()`** now resets `_rangeStart`, `_rangeEnd`, `_referenceTs`, `_bookmarkA`, `_bookmarkB`, and `_diffResult` on every new session load.
- **`LogBrowserScreen` layout** — `LazyColumn` now wrapped in a `Box(weight = 1f)` so the scrubber is anchored at the bottom. The `SessionHeader` extracted to its own composable. Empty-state message distinguishes "no frames at all" from "no frames in selected window".

## 2026-06-09 (session 6 — Phase 5 completion: canonical names, new DBC, copy signal)

### Added
- **`CanonicalSignals` catalog** — `CanonicalSignal` data class + `CanonicalSignals.ALL` list with 47 common automotive signals (engine RPM/torque/load, wheel speeds, O2 sensors, fuel trims, temperatures, dynamics, body signals) including unit, min, max, factor, offset, and description for each.
- **Canonical name autocomplete (Phase 5.5)** — `SignalEditorScreen` now shows a `DropdownMenu` below the signal name field when 2+ characters are typed. Each suggestion shows the canonical name (monospace) + unit and description. Selecting auto-fills `unit`, `min`, `max`, `factor`, and `offset`.
- **Create new DBC (Phase 5.6)** — `DbcListScreen` has a FAB (+ icon) that opens a "New DBC" dialog. The name field filters to letters, digits, `_`, and `-`. Duplicate names are blocked with an inline error. `SettingsViewModel.createDbc(name)` writes an empty DBC and refreshes the list. Empty-state text updated to mention the FAB.
- **Copy signal from another DBC (Phase 5.7)** — "Copy from…" button in `SignalEditorScreen` opens a two-level `AlertDialog` picker: level 1 lists available DBC files; level 2 lists signals from the selected DBC (sorted by message, showing message name + unit + min/max). Selecting a signal copies all fields including bit layout (startBit, length, byteOrder, signed, factor, offset, unit, min, max, comment, valueDescriptions) into the form. DBC loading runs on `Dispatchers.IO` via `LaunchedEffect`.

### Changed
- **Phase 5 marked mostly complete** in roadmap (5.3 rate-of-change backlog; everything else done).
- **Phase 6 marked active** in roadmap.

## 2026-06-09 (session 5 — Phase 5 signal health + Phase 11.1 help tile)

### Added
- **ROADMAP.md refreshed** — Current state section rewritten to reflect firmware v1.2.0, completed Phase 4 (1.3/1.4 now actually done), hardware-blocked path clearly noted (Phase 7.2+ require S-pin mod), Phase 11 Documentation & help added as a proper phase, Firmware backlog updated with F.6 ✓ and new F.7 (TX safety model). User help note converted to structured items 11.1–11.3.
- **Stuck signal detection (Phase 5.1)** — `CanBusViewModel` tracks a rolling window (50 frames / ~5 s) of decoded values per signal. A signal is flagged `isStuck` when all window values are equal. History is tracked in `signalValueHistory: MutableMap<String, ArrayDeque<Double>>` populated in `processBatch`.
- **Pegged signal detection (Phase 5.2)** — A signal is flagged `isPegged` when all window values are at or beyond the DBC `min` or `max` (only checked when `min < max` to avoid false positives on unconfigured ranges). Indicates wrong scaling, startBit, or length.
- **`SignalHealth` data class** — `data class SignalHealth(val isStuck: Boolean, val isPegged: Boolean)` exposed as `canBusVm.signalHealth: StateFlow<Map<String, SignalHealth>>`. Updated every processing batch; gated by the freeze flag alongside other UI StateFlows.
- **Health flags in SignalsScreen** — Amber banner row at top of Signals tab shows "N signals with health warnings" and navigates to the health panel on tap. Individual `SignalRow` shows amber `Warning` icon next to the value and a sub-label (`Stuck`, `Pegged`, or `Stuck+Pegged`) under the signal name.
- **Signal health panel screen (Phase 5.4)** — New `SignalHealthScreen` at `signal_health` route. Lists all currently flagged signals with message name, CAN ID, and `STUCK`/`PEGGED` chips. Empty state explains the 5-second observation window needed before flags appear. Accessible from the amber banner on the Signals tab.
- **Help & Documentation tile (Phase 11.1)** — New tile at bottom of SettingsScreen opens the project GitHub URL in the system browser. Uses `LocalContext` + `Intent.ACTION_VIEW`. Unobtrusive placement — settings screen only.

### 2026-06-09 (session 4 — infrastructure fixes from external review)

### Added
- **Connection persistence** — Last device address/name persisted in SharedPreferences. On app start, the BLE button in the top bar glows green and triggers immediate reconnect to the saved device (no scan required). On unintentional disconnect, auto-reconnect is attempted after 3 s, then 15 s (up to 3 attempts). User-initiated `disconnect()` suppresses this. `lastKnownDevice` StateFlow exposed for UI.
- **Operational telemetry** — `CanStats` data class exposed as `canBusVm.canStats` StateFlow, tracking: total frames processed, decode-out-of-range events (signals extending past DLC), parse errors, and BLE notification count. Updates every processing batch.
- **DBC mux signal awareness** — `DbcSignal.muxIndicator` field added (`"M"` = selector, `"m<N>"` = muxed at slot N, `null` = plain). Parser captures and preserves the indicator. Writer emits it in DBC output. Decoder in `processBatch` now: decodes the mux selector first, then skips muxed signals whose slot doesn't match the current selector value. Prevents garbage values when loading opendbc files with multiplexed signals.
- **Per-frame firmware timestamps (v1.2.0)** — Firmware `appendFrame` now appends `,<millis_mod_65536>` to each frame line. Android parser extracts this into `CanFrame.firmwareTimestampMs`. `handleNotification` uses intra-packet firmware offsets to reconstruct per-frame timestamps anchored to BLE receipt time, eliminating batch-level timestamp compression where all frames in a notification shared a single receive time.

### Changed
- **Session notes surfacing** — `SessionRow` in VehicleDetailScreen now shows `notes` below the frame/duration line. Log browser header shows notes if non-empty.
- **LogPlayerViewModel** — now uses `SignalDecoder.decodeOrNull()` for replay decoding; out-of-range signals are excluded from the displayed map rather than showing 0.
- **Roadmap Phase 4** — Updated status: items 4.1–4.12 marked complete.
- **Firmware v1.1.0 → v1.2.0** — Adds per-frame millis timestamp to BLE packet format. Backward compatible: old app ignores the extra field; new app uses it when present. New firmware.bin staged in Android assets for OTA delivery.

## 2026-06-09 (session 3 — gap fixes)

### Added
- **Signal deletion** (Item 1): Delete icon on each signal row in SignalsScreen. Confirms with AlertDialog before removing from DBC.
- **Message deletion** (Item 1): Three-dot overflow menu on MessageRow header with "Delete message" option and AlertDialog confirmation.
- **Editable DLC on new message** (Item 2): DLC field shown in SignalEditorScreen when creating a new message; defaults to observed frame size from live data. BitGrid and bounds checks now respect the user-entered or observed DLC instead of hardcoding 8.
- **Search / filter — Unknowns screen** (Item 3): TextField filter bar on UnknownsScreen filters by hex ID string (uppercase prefix match). Clear button resets filter; trigger window subsection filters too.
- **Search / filter — Signals screen** (Item 3): TextField filter bar on SignalsScreen filters by message name and CAN ID hex string.
- **Coverage badges** (Item 4): "N / M messages seen this session" summary row at top of SignalsScreen. Message header rows show a green (seen) or dim (not seen) dot indicator.
- **Frame inspector for known messages** (Item 5): FrameInspectorScreen now accepts both unknown and known CAN IDs. Known messages show recent-frame history (ring buffer added to MessageState) and overlay defined signal byte ranges with distinct tint colors and a legend. Inspect action added to SignalsScreen message overflow menu.
- **Signal comment field** (Item 6): Multiline "Notes / comment" field in SignalEditorScreen. Written to DBC `CM_` block.
- **VAL_ editor** (Item 7): Collapsible "Value descriptions" section in SignalEditorScreen. Add/delete raw-value → label entries via dialog; persisted to DBC `VAL_` block.
- **Session notes at record time** (Item 9): Notes field added to vehicle picker dialog in LiveScreen. Passed through to SessionMeta on recording start.

### Changed
- **Trigger window survives freeze** (Item 8): `processBatch()` now always runs trigger bookkeeping and recording; only StateFlow UI emissions are gated by the frozen flag. Frames processed while frozen retain correct `triggeredInWindow` state.
- **DLC guard in signal decoder** (Item 10): `SignalDecoder.decodeOrNull()` returns null when a signal's bit range extends past the frame DLC. `CanBusViewModel.processBatch()` now uses `decodeOrNull()` — out-of-range signals are excluded from the decoded map rather than silently returning 0. `SignalEditorScreen` shows a warning when the signal extends past the observed frame DLC.

## 2026-06-09

### Changed
- `readme.md`: corrected "Frame format" section — each BLE notification carries one or more packed frames (not one), clarified hex byte encoding, and added the EXT ID example. Added OTA GATT service UUIDs. Expanded firmware flash section to document `build_and_stage_firmware.sh` and wireless OTA update path.

---

## 2026-06-09

### Changed
- Updated hardware target for the BLE companion dongle from ESP32-C6 to ESP32-S3 (Seeed Studio XIAO ESP32-S3). README updated to reflect the new board target.

### Verified
- OTA firmware update confirmed working end-to-end over BLE. Device reports v1.1.0 via version characteristic; app shows "Device is up to date." Dongle no longer requires USB cable for firmware updates.

---

### Added
- BLE OTA firmware update support — dongle can now be updated wirelessly after the initial USB flash.
  - `partitions.csv`: custom dual-OTA partition table for 8 MB flash (app0/app1 at 3.2 MB each).
  - OTA GATT service (UUIDs `6E410001–6E410005`): control (write), data (write-no-response), status (notify), version (read) characteristics.
  - CAN frame streaming pauses automatically during OTA to avoid BLE congestion.
  - `esp_ota_mark_app_valid_cancel_rollback()` called on boot so the bootloader does not roll back a healthy image.
- `scripts/build_and_stage_firmware.sh`: compiles firmware, stages `firmware.bin` + `version.txt` into Android assets, and optionally flashes via USB in one command.
- **Settings → Firmware Update** screen in the Android app: shows device version vs. bundled version, streams chunks with a progress bar, and reports errors clearly. Firmware version is read from the OTA version characteristic on connect.
- Vehicle detail screen (Settings → Vehicles → tap vehicle): shows vehicle info card, Edit Profile button, and a list of all recordings for that vehicle sorted by date.
- Log browser screen: tap any recording to open a full frame-by-frame view with relative timestamps (`+MM:SS.mmm`), Known/Unknown filter chips, decoded signal values for known IDs, and raw hex for unknowns.
- `LogPlayerViewModel`: loads a session's `frames.log`, re-decodes against the DBC that was active at record time, and exposes filtered frames as a StateFlow.

### Changed
- Vehicle list row tap now navigates to vehicle detail instead of directly to the edit screen. Edit is accessible via the detail screen.
- `SettingsViewModel` now exposes `sessionRepository` and `loadSessionsForVehicle()` / `vehicleSessions` StateFlow for the vehicle detail screen.
- OTA GATT characteristic discovery and CCCD setup chained in `onDescriptorWrite` so UART and OTA notifications are enabled sequentially without GATT operation conflicts.
- CLAUDE.md updated: changelog entries are now required for all `phone_companion/` changes.

## 2026-06-08 (session 2)

### Added
- Frame packing in ESP32 firmware: multiple CAN frames batched per BLE notification (10 ms flush window, 400-byte early-flush threshold) to avoid frame drops on busy buses.
- `DisplayFrame.seq` monotonic counter to fix LazyColumn duplicate-key crashes when the same CAN ID fires multiple times per millisecond.
- Frame Inspector screen (`inspector/{canId}`) showing byte-level diff grid for unknown CAN IDs with `+Δms` row deltas and a "Define signal" shortcut.
- Long-press verification marking on signal rows (VERIFIED / SUSPECT / UNVERIFIED with optional notes), backed by sidecar JSON.
- Record / Stop button on Live screen with vehicle picker dialog.
- Freeze button in top app bar (Pause/Play) — suppresses StateFlow updates without interrupting internal processing or recording.
- Unknown ID persistence: IDs stay visible for 10 s of silence instead of being evicted each 100 ms batch tick.
- Connect / Bluetooth icon button in top app bar when disconnected — lets the user re-open the device scanner at any time.
- Pull from Git tile in Settings (fetches remote changes, refreshes DBC and vehicle lists).
- INTERNET and ACCESS_NETWORK_STATE permissions required for JGit clone/push.
- `CLAUDE.md` at repo root documenting arduino-cli and JAVA_HOME tool paths.

### Changed
- BLE connect dialog can now be dismissed (Skip button / tap-outside) without connecting — app is fully navigable without a dongle.
- "Sync to Git" renamed to "Push to Git" throughout Settings UI for clarity.
- Pause button tint set to white (was inheriting an invisible color on dark backgrounds).
- `stopScan()` no longer clears the scanned device list; new `dismissScanDialog()` does both, preventing the picker from reappearing after dismissal.

### Fixed
- `AcceleratorPedalPos` (0x1A1) startBit corrected from `48|8@1+` (byte 6, past end of 3-byte frame) to `16|8@1+` (byte 2).
- `SteeringWheelAngle` (0x1E5) startBit corrected from `7|16@0-` (bytes 0–1, constant) to `47|16@0-` (bytes 5–6, Motorola MSB — confirmed against steering sweep capture).
- `YawRate` (0x1E9) startBit corrected from `51|12@0-` (decoded 64 grad/s at rest) to `35|12@0-` (bytes 4–5, 0 grad/s at rest — matches opendbc reference).
- Session `appendFrame()` wrapped in try-catch to handle close race condition.

## 2026-06-08

### Added
- Bootstrapped Android app at `android_app/` to scan/connect over BLE and display incoming CAN frame text.
- Added ESP32 firmware at `firmware/esp32_can_ble_bridge/esp32_can_ble_bridge.ino` for CAN listen-only capture and BLE NUS notifications.
- Added local dependency bootstrap script `scripts/setup_android_env.sh` for JDK/Gradle/Android SDK command-line setup.
- Added project README instructions for setup, build, flash, and runtime usage.

### Changed
- Installed and used repository Arduino CLI (`/home/daniel/documents/workspace/robo-services/bin/arduino-cli`) for firmware compile/upload.
- Corrected hardware target from C6 to S3 during flashing after chip ID mismatch.
- Updated BLE advertising setup to explicitly advertise as `CAN-DONGLE` and auto-restart advertising on disconnect/idle checks.
- Adjusted firmware behavior to keep BLE advertising even if CAN init fails.

### Verified
- Android debug app builds and installs on device.
- Firmware flashes successfully to connected ESP32-S3.
- BLE advertisement is detectable as `CAN-DONGLE` in external scan.

### Known issue
- End-to-end phone connection/data flow remains unresolved in current session despite successful flashing and observable BLE advertising.
