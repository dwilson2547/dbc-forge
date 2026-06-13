# Sensor-aided signal discovery (IMU + GPS correlation)

Status: **planning** · Owner: Daniel · Created 2026-06-12

## Goal

Auto-identify unknown CAN signals the way [LibreCAN](https://github.com/lscandroid/librecan)
does: record a ground-truth physical reference (vehicle speed, yaw rate, lateral/
longitudinal acceleration) while capturing CAN, then brute-force candidate bit-field
decodings of unknown IDs and rank them by how well each correlates with the matching
reference channel. A high-correlation candidate is a probable signal — the tool then
infers scale/offset and offers to promote it to a DBC signal.

This stays inside the **passive DBC reverse-engineering** scope of the app — the phone
and dongle only *read* sensors and the bus. No active diagnostics.

## What already exists (reuse, don't rebuild)

The OBD-II cross-reference feature is the same algorithm with a different reference
source. We already have:

- `pearson()` + windowed `(reference, native)` sample accumulation in
  `CanBusViewModel.processBatch()` (`CanBusViewModel.kt:746`)
- |r| thresholding + best-match promotion → `OdbCrossRef` StateFlow → green badge in
  `SignalRow`
- `CanonicalSignals.ALL` already defines every target: `VEHICLE_SPEED`,
  `WHEEL_SPEED_FL/FR/RL/RR`, `YAW_RATE`, `LATERAL_ACCEL`, `LONGITUDINAL_ACCEL`,
  `STEERING_ANGLE`
- Offline multi-signal correlation plotting in `LogPlayerViewModel`
- Per-frame capture timestamps from the dongle's monotonic `millis()` clock

**The two genuinely new pieces:**

1. A **reference-channel source** (IMU + GPS) recorded alongside CAN frames.
2. A **brute-force candidate decoder** that slides `(startBit, length, endianness,
   signed)` windows across *unknown* IDs and correlates each — today correlation only
   runs on already-decoded *known* signals.

## Architecture decision — where the sensors live

The key insight: the dongle already timestamps every CAN frame from one monotonic
clock. **Any sensor read by the same MCU shares that clock**, which eliminates the
BLE-latency time-alignment problem for that sensor. So:

- **IMU → on the dongle (ESP32-C6).** Yaw rate and acceleration are high-rate, dynamic
  signals where tight time alignment to CAN matters most. Co-locating the IMU with CAN
  capture gives single-clock alignment for free, and the BNO085's on-chip fusion solves
  the orientation problem (below).
- **GPS → on the phone (to start).** Speed-over-ground is a slow, smooth signal that
  tolerates a looser, lag-searched alignment across clock domains. The phone already has
  a GNSS receiver, so this costs nothing to start.

This split is actually optimal: the channels that *need* tight timing (yaw/accel) get
the single-clock path; the channel that doesn't (speed) takes the free phone receiver.

```
  ESP32-C6 dongle                              Android phone
  ┌────────────────────────┐                   ┌──────────────────────────┐
  │ TWAI (CAN) ──┐         │   BLE NUS         │  NUS parser              │
  │              ├─ shared ├──────────────────▶│   ├─ CAN frames          │
  │ BNO085 ──I2C─┘  millis │   STD/EXT + IMU   │   └─ IMU samples         │
  │                clock    │   lines           │                          │
  └────────────────────────┘                   │  FusedLocationProvider   │
                                                │   └─ GPS speed/course    │
                                                │                          │
                                                │  Correlation engine      │
                                                └──────────────────────────┘
```

---

## Sensor hardware — what to use

### IMU: Adafruit BNO085 — **yes, use it**

The BNO085 (on hand) is the right part and needs no purchase. It runs CEVA/Hillcrest
SH-2 sensor fusion on-chip, so it outputs *calibrated, fused* quantities directly rather
than raw MEMS data we'd have to filter ourselves:

| BNO085 report | Maps to CAN canonical |
|---|---|
| Calibrated gyro **Z** (yaw rate) | `YAW_RATE` |
| Linear acceleration **Y** (gravity removed) | `LATERAL_ACCEL` |
| Linear acceleration **X** (gravity removed) | `LONGITUDINAL_ACCEL` (also tracks brake/throttle) |
| Rotation vector (absolute quaternion) | mounting-orientation calibration |

**Why fusion matters here:** raw phone/MEMS accelerometers read in the *sensor* frame and
are dominated by gravity and mounting tilt. The BNO085's rotation vector lets us capture
the static mount quaternion once, then rotate every sample into the *vehicle* frame —
so "lateral" really means lateral. Linear-acceleration reports already have gravity
removed. This is the single biggest accuracy win and it's why we prefer the BNO085 over
the phone's own IMU.

**Connection:** I2C is the primary path (addr `0x4A` default, `0x4B` with SA0 high).
Pin budget on the XIAO C6 is fine — CAN uses 2 GPIO, the TX S-pin 1, leaving SDA/SCL free.

**Gotcha — I2C clock stretching:** the BNO08x stretches the I2C clock; keep the bus at
100 kHz and honour the RST/INT lines. If the ESP32 I2C path proves flaky, fall back to
**UART-RVC mode** (strap PS1=1/PS0=1): a dead-simple fixed 100 Hz, 115200-baud binary
stream of yaw/pitch/roll + 3-axis accel. RVC gives heading rather than gyro rate, so
yaw rate becomes the time-derivative of yaw — fine for correlation. Document both paths;
start with I2C full reports.

→ Full pin map / strapping / library notes: `docs/hardware/bno085_9dof_imu.md`.

### GPS — recommendation, and the USB-variant problem

**For correlation, you do not need RTK / cm-level position.** We correlate against
*velocity* (and optionally heading rate), not position. GNSS Doppler-derived
speed-over-ground is accurate to roughly ±0.1 m/s even on a cheap MTK3339 — RTK improves
*position* by orders of magnitude but barely moves *velocity* accuracy. **So: do not buy
another ZED-F9P for this.** The F9P stays on the RTK base station where its position
accuracy is actually used.

That leaves the two GPS units on hand and the phone:

| Option | Rate | Effort | Clock domain | Verdict |
|---|---|---|---|---|
| **A. Phone internal GNSS** (framework `LocationManager`) | ~1 Hz | none | phone | **Implemented** — free, Doppler speed is good enough; no Play Services dep |
| ~~B. Adafruit Ultimate GPS USB → phone OTG~~ | 10 Hz | medium | phone | **Dropped** — not worth plugging a module into the phone |
| C. UART GPS module on the dongle | 10 Hz | medium + small buy | dongle (single clock) | Only if A proves insufficient |

**Decision (session 23): phone GNSS only.** The phone's built-in receiver is the GPS
source, full stop — no external module, no USB-OTG dongle to deal with. The 1 Hz Doppler
speed it provides is plenty for speed correlation, and avoiding the USB GPS removes the
bridge-chip/OTG complexity entirely. If 1 Hz ever proves too coarse, the fallback is a
UART GPS on the dongle (Option C), not a phone-side USB module.

> **Implemented (session 22):** Option A via the framework `android.location.LocationManager`
> (`GPS_PROVIDER`), not `FusedLocationProvider` — this avoids a Google Play Services
> dependency and works on non-GMS devices, while `Location.getSpeed()` still gives the
> GNSS Doppler speed-over-ground we correlate against. See `GpsSource.kt`.

---

## Reference channel → CAN target map

| Reference channel | Sensor | Primary CAN targets | Notes |
|---|---|---|---|
| Speed over ground | phone GPS | `VEHICLE_SPEED`, `WHEEL_SPEED_*` | ✅ implemented; orientation-immune |
| Yaw rate | BNO085 gyro · gravity | `YAW_RATE`, `STEERING_ANGLE` | ✅ implemented; **mount-tolerant** via gravity projection (no calibration) |
| Lateral accel | BNO085 lin-accel Y | `LATERAL_ACCEL` | needs frame rotation (11.6) |
| Longitudinal accel | BNO085 lin-accel X | `LONGITUDINAL_ACCEL` | needs frame rotation (11.6); also tracks brake/throttle |

Start the engine with **speed only** — it's orientation-immune (no calibration needed),
has the cleanest signal, and exercises the whole pipeline end-to-end before the IMU
rotation machinery is added.

## Correlation algorithm (the new decoder)

For each unknown CAN ID, over a recorded window:

```
for startBit in 0 .. (dlc*8 - 1):
  for length in {8, 12, 16}:           # widen later if needed
    for endianness in {Intel, Motorola}:
      for signed in {false, true}:
        series = [ extractRaw(candidate, frame.data) for frame in id_frames ]
        resample series + reference onto a common time grid
        r = max over lag ∈ [-T, +T] of pearson(shift(series, lag), reference)   # lag-search
        if |r| ≥ threshold:
          (factor, offset) = linregress(series → reference)   # slope, intercept
          emit Candidate(id, startBit, length, endianness, signed, r, factor, offset)
rank candidates by |r|; suppress overlapping bit-windows; keep top-N per channel
```

New components:

- `CandidateDecoder` — enumerates and extracts candidate windows (reuses
  `SignalDecoder.extractRaw`).
- **Lag-search cross-correlation** — slides ±T to find the alignment peak. This is what
  absorbs the GPS cross-clock offset and any BLE jitter; without it a true match can hide
  behind a timing skew. (LibreCAN does the same.)
- **Linear regression** for scale/offset — `slope → factor`, `intercept → offset`,
  pre-filling the signal editor.
- **Orientation calibration** — capture the static BNO085 quaternion at rest, rotate
  every IMU sample into the vehicle frame before correlating accel channels.

Build it on the **offline log path first** (`LogPlayerViewModel`) — record the reference
channels into the session format now so logs are replay-ready, iterate the algorithm on
recorded data, and only later consider a live mode.

## Data model additions

- Session recording gains time-stamped reference rows. Sketch (CSV/line form, matching
  the existing frame log style):
  ```
  IMU,<ts_ms>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gz>
  GPS,<ts_ms>,<speed_mps>,<course_deg>,<hdop>,<fix>
  ```
- Firmware NUS stream gains an `IMU,...` line type alongside `STD,`/`EXT,`, sharing the
  existing intra-packet timestamp reconstruction. (GPS stays phone-side, so no GPS line
  from the dongle in the recommended design.) A dedicated BLE characteristic is an
  alternative if we want to decouple the parser; inline matches the current pattern and
  is the v1 choice.

## Open questions / risks

- **BNO085 I2C clock-stretching** on ESP32 — UART-RVC fallback documented if it bites.
- **Mounting** — the dongle/IMU must be rigidly fixed to the vehicle; a loose unit adds
  vibration noise. Orientation calibration assumes the mount doesn't move mid-session.
- **BLE bandwidth** — IMU at 100 Hz × ~40 B ≈ 4 KB/s on top of CAN. Within NUS framed-
  packing headroom, but down-sample to 50 Hz if it crowds CAN throughput.
