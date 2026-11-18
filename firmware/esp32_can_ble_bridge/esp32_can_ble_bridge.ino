/*
 * esp32_can_ble_bridge.ino
 * ------------------------
 * ESP32-S3 CAN listener that bridges decoded CAN frames over BLE NUS UART.
 *
 * Hardware target:
 * - Seeed Studio XIAO ESP32-C6  — D0=GPIO0, D1=GPIO1, D2=GPIO2, LED=GPIO15
 *   FQBN: esp32:esp32:XIAO_ESP32C6
 * - TJA1051 CAN transceiver: TX->D0, RX<-D1, S<-D2
 *   S is held LOW (transceiver Normal) at all times, since 1.8.1. Listen mode is the TWAI
 *   controller's LISTEN_ONLY alone. The unmarked "TJA1051" breakouts in use stop receiving
 *   with S HIGH (a genuine TJA1051 would only turn its transmitter off).
 * - u-blox MAX-M10S GPS (1.8.0, optional): module TX->D8 (GPIO19), RX<-D9 (GPIO20),
 *   TIMEPULSE->D10 (GPIO18, input only). No module: no GPS lines; the config is retried
 *   (5 tries, then every 30 s) without blocking the loop.
 *
 * BLE services:
 *   NUS (Nordic UART Service) — CAN frame streaming + command channel
 *     TX notify:  6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (frames → phone)
 *     RX write:   6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (commands → dongle)
 *       Commands: "ACK:AUTO" — listen, then acknowledge once the bus is proven (default,
 *                              restored on every connect)
 *                 "ACK:OFF"  — listen only
 *                 "ACK:ON"   — acknowledge now, without the proof
 *                 "TX_ENABLE" / "TX_DISABLE" — the same as ACK:ON / ACK:OFF
 *                 "BAUD:<rate>" — 125000, 250000, 500000 or 1000000; back to listen
 *                 "FILTER:<hex>,<hex>,..." — forward only these CAN ids (max 64);
 *                 "FILTER:" forwards every id again. Reset on every connect.
 *   TX notify also carries a diagnostics line every 5 s (1.6.2+):
 *     STAT,imu_lines,imu_drop,imu_events,imu_resets,imu_ready,can,can_drop,notify_fail,max_pass_ms,
 *          notifies,notify_bytes,can_mode,can_repeats,bus_errors
 *     notifies and notify_bytes since 1.6.3; the last three since 1.7.0:
 *     can_mode 0 = listen, 1 = ack, 2 = listen after an ack fallback; can_repeats = frames
 *     identical to the frame just before (id, data, same ms); bus_errors in the window.
 *   TX notify also carries one GPS line per navigation epoch, 10 Hz (1.8.0+):
 *     GPS,fix,sats,lat_deg,lon_deg,alt_m,speed_mps,heading_deg,hacc_m,utc_ms,pps_ts,ts
 *     fix 0 = none (or not gnssFixOK), 2 = 2D, 3 = 3D; alt_m above mean sea level;
 *     speed_mps ground speed; heading_deg of motion; utc_ms Unix ms, 0 until date and
 *     time are valid; pps_ts millis() % 65536 at the last PPS edge, -1 before the first;
 *     ts millis() % 65536 when the line was built (the clock of the other lines).
 *   USB serial accepts "GPSCFG" (bench): re-run the GPS configuration.
 *
 *   OTA service — firmware update over BLE
 *     Control write:  6E410002-B5A3-F393-E0A9-E50E24DCCA9E
 *       Commands: "START:<decimal_bytes>", "COMMIT", "ABORT"
 *     Data write:     6E410003-B5A3-F393-E0A9-E50E24DCCA9E
 *       Raw binary chunks, max ATT_MTU-3 bytes each
 *     Status notify:  6E410004-B5A3-F393-E0A9-E50E24DCCA9E
 *       Responses: "READY", "PROGRESS:<offset>", "VERIFYING", "OK", "IDLE",
 *                  "ERROR:<reason>"
 *     Version read:   6E410005-B5A3-F393-E0A9-E50E24DCCA9E
 *       Returns kFirmwareVersion as a UTF-8 string
 *
 * Frame packing:
 * - Multiple CAN frames are batched into a single BLE notification.
 * - Loop drains the TWAI queue non-blocking, then waits up to kFlushIntervalMs
 *   before flushing. A notification is sent when nearly full or when its oldest line
 *   is kMaxBufferAgeMs old, not on every pass. It is never longer than the negotiated ATT MTU
 *   allows (MTU - 3, at most kBufSize): the stack would cut a longer one short. A
 *   line that would not fit flushes the buffer first.
 * - CAN processing is paused during OTA to avoid BLE congestion.
 *
 * CAN modes (1.7.0):
 * - Listen: TWAI LISTEN_ONLY (the transceiver stays Normal, S LOW, since 1.8.1). Every boot,
 *   every disconnect and every BAUD change starts here.
 * - Ack: TWAI NORMAL. A sender resends a frame until some node
 *   acknowledges it; on a port with no other listener (a 2024 Blazer, 2026-11-04) a
 *   listen-only dongle records every resend. No CAN frames are transmitted by this
 *   firmware (twai_transmit is never called), so ack mode only drives ACK bits and, on
 *   errors, error flags.
 * - Auto switches listen -> ack after kAckProofFrames valid frames with fewer than one
 *   bus error per kAckProofRatio of them. A wrong bitrate gives almost no valid frames,
 *   so the dongle never joins a bus it can't read. Errors in ack mode (warning level,
 *   bus-off, or errors above that ratio) fall back to listen until the next connect.
 *
 * OTA safety:
 * - esp_ota_end() validates the image magic and SHA256 before committing.
 * - The inactive partition is written; the active partition is unchanged until
 *   esp_ota_set_boot_partition() is called after successful verification.
 * - COMMIT is verified and applied from loop(), after its write has been acknowledged
 *   (1.7.1; before, the dongle restarted inside the write and COMMIT was never acked).
 * - A new image is marked valid only once a phone has connected over BLE (1.7.1;
 *   before, at the start of setup()). An image that boots but can't serve BLE stays
 *   pending, and the bootloader rolls back to the previous one on the next reset.
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "driver/twai.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

// ── Firmware version ──────────────────────────────────────────────────────────

constexpr char kFirmwareVersion[] = "1.8.1";

// ── Hardware config ───────────────────────────────────────────────────────────

namespace {

// ── Pin config (XIAO ESP32-C6) ────────────────────────────────────────────────
constexpr gpio_num_t kCanTxPin = GPIO_NUM_0;  // D0
constexpr gpio_num_t kCanRxPin = GPIO_NUM_1;  // D1
constexpr gpio_num_t kCanSPin  = GPIO_NUM_2;  // D2 — transceiver S pin, held LOW (see header)
constexpr uint8_t    kLedPin   = 15;
constexpr uint32_t   kBaud     = 500000;

// BNO085 IMU (Phase 11.3) — I2C on the XIAO C6 default bus (SDA=D4, SCL=D5).
// AD0 strapped high → address 0x4B. Poll mode (no INT dependency in the hot loop).
constexpr uint8_t  kImuAddr        = 0x4B;
constexpr uint32_t kImuI2cHz       = 100000;   // BNO08x stretches the clock — keep at 100 kHz
constexpr uint16_t kImuReportMs    = 20;       // 50 Hz — gyro + linear accel
constexpr uint16_t kImuRvReportMs  = 100;      // 10 Hz — game rotation vector (see note in enableImuReports)
constexpr uint32_t kImuEmitMs      = 20;       // throttle combined IMU lines to 50 Hz
constexpr int      kImuDrainMax    = 8;        // max sensor events serviced per loop pass

// u-blox MAX-M10S GPS (1.8.0) on UART1: module TX -> D8, module RX <- D9, PPS -> D10.
constexpr int kGpsRxPin  = 19;   // D8, ESP RX
constexpr int kGpsTxPin  = 20;   // D9, ESP TX
constexpr int kGpsPpsPin = 18;   // D10, input only (see initGps)

// Frame packing
constexpr uint32_t kFlushIntervalMs = 10;
constexpr int      kBufSize         = 512;    // largest notification, used when the MTU allows
constexpr int      kMaxLineLen      = 128;    // longest line (IMU); flush once less than this is left
constexpr uint16_t kPreferredMtu    = 517;    // asked for at init; the phone may settle lower
constexpr int      kMaxFilterIds    = 64;     // FILTER command capacity
// A partly filled notification is sent once its oldest line is this old. Flushing on
// every loop pass sent ~210 notifications/s of ~100 bytes, and phones drop some at
// that rate (measured: up to 15% lost); this keeps them near full at ~45/s.
constexpr uint32_t kMaxBufferAgeMs  = 50;

// OTA progress notification interval (bytes between PROGRESS notifications)
constexpr size_t kOtaProgressInterval = 16 * 1024;

// ── Runtime state ─────────────────────────────────────────────────────────────

NimBLEServer*         gServer           = nullptr;
NimBLECharacteristic* gTxCharacteristic = nullptr;   // NUS TX (frames → phone)
NimBLECharacteristic* gOtaStatusChar    = nullptr;
NimBLEAdvertising*    gAdvertising      = nullptr;

bool     gClientConnected = false;
bool     gCanReady        = false;
bool     gTxEnabled       = false;    // current mode: true = ack (NORMAL), false = listen

// Ack policy (see "CAN modes" in the header). Set by commands from the BLE task, read by loop().
enum class AckPolicy : uint8_t { AUTO, OFF, ON };
volatile AckPolicy gAckPolicy        = AckPolicy::AUTO;
volatile bool      gAckPolicyChanged = false;
bool               gAckFellBack      = false;  // ack mode tripped this connection; stay listening
constexpr uint32_t kAckProofFrames   = 200;
constexpr uint32_t kAckProofRatio    = 10;     // at most one bus error per 10 valid frames
constexpr uint32_t kAckCheckMs       = 250;
constexpr uint32_t kAckErrorWarn     = 96;     // TWAI error-warning limit
uint32_t gModeFrames      = 0;   // valid frames since the driver (re)started
uint32_t gLastAckCheckMs  = 0;
uint32_t gAckCheckFrames  = 0;   // gModeFrames and bus errors at the previous check
uint32_t gAckCheckErrors  = 0;
uint32_t gLastAdvCheckMs  = 0;

// ── IMU state (Phase 11.3) ────────────────────────────────────────────────────
BNO08x   gImu;
bool     gImuReady     = false;
uint32_t gLastImuEmitMs = 0;
uint32_t gLastImuTryMs  = 0;
// Latest cached reports (sensor frame). Quaternion is unitless; accel m/s²; gyro rad/s.
float    gQw = 1, gQx = 0, gQy = 0, gQz = 0;
float    gAx = 0, gAy = 0, gAz = 0;
float    gGx = 0, gGy = 0, gGz = 0;
bool     gImuHasData = false;          // at least one report cached since boot

// Frame pack buffer (NUS TX)
char gTxBuf[kBufSize];
int  gTxBufLen = 0;

// Deferred CAN reinit — set by BLE callback, applied in loop() at a safe point
volatile bool     gReinitRequested = false;
volatile bool     gReinitTxTarget  = false;
volatile uint32_t gReinitBaudRate  = 500000;   // default; updated by BAUD command

// Largest notification payload the connection carries: ATT MTU - 3. The stack
// silently truncates a longer notification, so the TX buffer is filled only to this.
// 20 until the MTU exchange (default ATT MTU 23).
volatile int      gNotifyMax       = 20;

// CAN id allowlist from the FILTER command; empty = forward every frame. Written by
// the BLE task into gPendingFilter, applied by loop() (same hand-off as gReinit*).
uint32_t          gFilterIds[kMaxFilterIds];
int               gFilterCount     = 0;
uint32_t          gPendingFilter[kMaxFilterIds];
volatile int      gPendingFilterCount = 0;
volatile bool     gFilterRequested = false;

// Counters printed every kStatIntervalMs as one "[STAT]" serial line, then reset.
uint32_t gStatImuLines = 0, gStatImuDropped = 0, gStatImuEvents = 0, gStatImuResets = 0;
uint32_t gStatCanFrames = 0, gStatCanDropped = 0, gStatNotifyFails = 0, gStatMaxPassMs = 0;
uint32_t gStatNotifies = 0, gStatNotifyBytes = 0;
uint32_t gStatCanRepeats = 0, gStatBusErrorsAt = 0;   // repeats in the window; bus errors at its start
twai_message_t gPrevMsg = {};   // the frame received just before, for the repeat count
uint32_t       gPrevMsgMs = 0;
uint32_t gLastStatMs = 0;
uint32_t gBufFirstLineMs = 0;   // when the oldest line in gTxBuf was added
constexpr uint32_t kStatIntervalMs = 5000;

void flushBuf();
bool appendLine(const char* line, int len);
void serviceImu();
void appendImu();

// Bus-off recovery timing
uint32_t gLastBusOffCheckMs    = 0;
uint32_t gLastBusOffRecoveryMs = 0;

// ── OTA state ─────────────────────────────────────────────────────────────────

enum class OtaPhase { IDLE, RECEIVING, COMMITTING };

OtaPhase               gOtaPhase         = OtaPhase::IDLE;
esp_ota_handle_t       gOtaHandle        = 0;
const esp_partition_t* gOtaPartition     = nullptr;
size_t                 gOtaExpectedBytes = 0;
size_t                 gOtaBytesWritten  = 0;
size_t                 gOtaLastProgress  = 0;
volatile bool          gOtaCommitRequested = false;  // set by the COMMIT write, applied by loop()
volatile bool          gMarkValidRequested = false;  // set on a BLE connect, applied by loop()

// ── Helpers ───────────────────────────────────────────────────────────────────

inline void ledOn()  { digitalWrite(kLedPin, LOW);  }
inline void ledOff() { digitalWrite(kLedPin, HIGH); }

void otaNotify(const char* msg) {
    if (gOtaStatusChar && gClientConnected) {
        gOtaStatusChar->setValue(msg);
        gOtaStatusChar->notify();
    }
}

// Verify the received image, switch the boot partition and restart (from loop()).
void otaCommit() {
    otaNotify("VERIFYING");
    Serial.printf("[OTA] Verifying — %u / %u bytes\n", gOtaBytesWritten, gOtaExpectedBytes);
    esp_err_t err = esp_ota_end(gOtaHandle);
    gOtaHandle = 0;
    if (err != ESP_OK) {
        char msg[40];
        snprintf(msg, sizeof(msg), "ERROR:verify_failed_%d", err);
        otaNotify(msg);
        gOtaPhase = OtaPhase::IDLE;
        return;
    }
    err = esp_ota_set_boot_partition(gOtaPartition);
    if (err != ESP_OK) { otaNotify("ERROR:set_boot_failed"); gOtaPhase = OtaPhase::IDLE; return; }
    otaNotify("OK");
    Serial.println("[OTA] Success — restarting");
    delay(500);  // let OK reach the phone
    esp_restart();
}

// A phone reached this image over BLE: keep it (cancels the bootloader's rollback).
// Works only because verifyRollbackLater() below stops the Arduino core from marking
// every pending image valid at startup (initArduino, esp32-hal-misc.c).
void markAppValid() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (running && esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        Serial.println("[OTA] New image confirmed by a BLE connection");
    }
}

void otaAbort() {
    if (gOtaPhase == OtaPhase::RECEIVING) {
        esp_ota_abort(gOtaHandle);
        gOtaHandle = 0;
    }
    gOtaPhase = OtaPhase::IDLE;
    gOtaPartition = nullptr;
    gOtaExpectedBytes = 0;
    gOtaBytesWritten = 0;
    gOtaLastProgress = 0;
}

// Stop the TWAI driver and restart in the requested mode/baud.
// ack = true: TWAI NORMAL. false: LISTEN_ONLY. The S pin stays LOW either way.
// Must only be called from loop() — not from BLE callback context.
void reinitCan(bool enableTx) {
    if (gCanReady) {
        twai_stop();
        twai_driver_uninstall();
        gCanReady = false;
    }


    twai_general_config_t gCfg = TWAI_GENERAL_CONFIG_DEFAULT(
        kCanTxPin, kCanRxPin,
        enableTx ? TWAI_MODE_NORMAL : TWAI_MODE_LISTEN_ONLY);
    gCfg.rx_queue_len = 64;

    twai_timing_config_t tCfg;
    switch (gReinitBaudRate) {
        case 125000:   tCfg = TWAI_TIMING_CONFIG_125KBITS();  break;
        case 250000:   tCfg = TWAI_TIMING_CONFIG_250KBITS();  break;
        case 1000000:  tCfg = TWAI_TIMING_CONFIG_1MBITS();    break;
        default:       tCfg = TWAI_TIMING_CONFIG_500KBITS();  break;
    }
    twai_filter_config_t fCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&gCfg, &tCfg, &fCfg) == ESP_OK &&
        twai_start() == ESP_OK) {
        gCanReady  = true;
        gTxEnabled = enableTx;
        gModeFrames = gAckCheckFrames = gAckCheckErrors = gStatBusErrorsAt = 0;
        Serial.printf("[CAN] Mode: %s  Baud: %lu\n",
                      enableTx ? "ACK (NORMAL)" : "LISTEN (LISTEN_ONLY)", gReinitBaudRate);
    } else {
        twai_driver_uninstall();
        gTxEnabled = false;
        Serial.println("[CAN] reinitCan failed");
    }
}

// ── OTA GATT callbacks ────────────────────────────────────────────────────────

class OtaCtrlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        const std::string val = pChar->getValue();
        if (val.empty()) return;

        if (val.rfind("START:", 0) == 0) {
            if (gOtaPhase != OtaPhase::IDLE) otaAbort();
            const size_t size = static_cast<size_t>(atol(val.c_str() + 6));
            if (size == 0) { otaNotify("ERROR:invalid_size"); return; }

            gOtaPartition = esp_ota_get_next_update_partition(NULL);
            if (!gOtaPartition) { otaNotify("ERROR:no_ota_partition"); return; }

            const esp_err_t err = esp_ota_begin(gOtaPartition, OTA_WITH_SEQUENTIAL_WRITES, &gOtaHandle);
            if (err != ESP_OK) {
                char msg[40];
                snprintf(msg, sizeof(msg), "ERROR:begin_failed_%d", err);
                otaNotify(msg);
                return;
            }
            gOtaExpectedBytes = size;
            gOtaBytesWritten  = 0;
            gOtaLastProgress  = 0;
            gOtaPhase         = OtaPhase::RECEIVING;
            otaNotify("READY");
            Serial.printf("[OTA] START — expecting %u bytes\n", size);

        } else if (val == "COMMIT") {
            if (gOtaPhase != OtaPhase::RECEIVING) { otaNotify("ERROR:not_receiving"); return; }
            // Verify and restart from loop(): returning here lets the stack acknowledge the write
            gOtaPhase = OtaPhase::COMMITTING;
            gOtaCommitRequested = true;

        } else if (val == "ABORT") {
            otaAbort();
            otaNotify("IDLE");
            Serial.println("[OTA] Aborted");
        }
    }
};

class OtaDataCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        if (gOtaPhase != OtaPhase::RECEIVING) return;
        const std::string val = pChar->getValue();
        if (val.empty()) return;

        const esp_err_t err = esp_ota_write(gOtaHandle, val.data(), val.size());
        if (err != ESP_OK) {
            char msg[40];
            snprintf(msg, sizeof(msg), "ERROR:write_failed_%d", err);
            otaAbort();
            otaNotify(msg);
            return;
        }
        gOtaBytesWritten += val.size();
        if (gOtaBytesWritten - gOtaLastProgress >= kOtaProgressInterval ||
            gOtaBytesWritten >= gOtaExpectedBytes) {
            gOtaLastProgress = gOtaBytesWritten;
            char msg[32];
            snprintf(msg, sizeof(msg), "PROGRESS:%u", gOtaBytesWritten);
            otaNotify(msg);
        }
    }
};

// ── NUS RX callback (F.1 / F.2) ──────────────────────────────────────────────

class NusRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        const std::string val = pChar->getValue();
        if (val == "ACK:AUTO" || val == "ACK:OFF" || val == "ACK:ON" ||
            val == "TX_ENABLE" || val == "TX_DISABLE") {
            Serial.printf("[CMD] %s received\n", val.c_str());
            gAckPolicy = (val == "ACK:AUTO") ? AckPolicy::AUTO
                       : (val == "ACK:ON" || val == "TX_ENABLE") ? AckPolicy::ON
                       : AckPolicy::OFF;
            gAckPolicyChanged = true;
        } else if (val.rfind("BAUD:", 0) == 0) {
            const uint32_t baud = static_cast<uint32_t>(atol(val.c_str() + 5));
            if (baud == 125000 || baud == 250000 || baud == 500000 || baud == 1000000) {
                Serial.printf("[CMD] BAUD:%lu received\n", baud);
                gReinitBaudRate  = baud;
                gReinitTxTarget  = false;   // a new bitrate is proven again before acking
                gReinitRequested = true;
            } else {
                Serial.printf("[CMD] BAUD: unsupported rate %lu\n", baud);
            }
        } else if (val.rfind("FILTER:", 0) == 0) {
            // FILTER:<hex id>,<hex id>,...  forward only these CAN ids
            // FILTER:  (empty)              forward every id again
            int n = 0;
            const char* p = val.c_str() + 7;
            while (*p && n < kMaxFilterIds) {
                char* end = nullptr;
                const unsigned long id = strtoul(p, &end, 16);
                if (end == p) break;
                gPendingFilter[n++] = static_cast<uint32_t>(id);
                p = (*end == ',') ? end + 1 : end;
            }
            gPendingFilterCount = n;
            gFilterRequested    = true;
            Serial.printf("[CMD] FILTER: %d id(s)\n", n);
        } else {
            Serial.printf("[CMD] Unknown command: %s\n", val.c_str());
        }
    }
};

// ── NUS server callbacks ──────────────────────────────────────────────────────

int notifyMaxFor(uint16_t mtu) {
    const int payload = static_cast<int>(mtu) - 3;
    return payload < kBufSize ? payload : kBufSize;
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
        (void)server;
        gClientConnected = true;
        gTxBufLen = 0;
        gNotifyMax = notifyMaxFor(connInfo.getMTU());
        gPendingFilterCount = 0;  // every client starts unfiltered
        gFilterRequested    = true;
        gAckPolicy          = AckPolicy::AUTO;  // and with the default ack policy
        gAckPolicyChanged   = true;
        gMarkValidRequested = true;             // a phone can reach this image: keep it
        ledOn();
        Serial.println("[BLE] Client connected");
    }

    void onMTUChange(uint16_t mtu, NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        gNotifyMax = notifyMaxFor(mtu);
        Serial.printf("[BLE] MTU %u -> notifications up to %d bytes\n", mtu, (int)gNotifyMax);
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        (void)server; (void)connInfo; (void)reason;
        gClientConnected = false;
        gNotifyMax = 20;
        if (gOtaPhase != OtaPhase::IDLE) {
            otaAbort();
            Serial.println("[OTA] Aborted — client disconnected");
        }
        // Back to listen on disconnect: nobody is recording, so stay fully passive
        gAckPolicy = AckPolicy::OFF;
        gAckPolicyChanged = true;
        Serial.println("[CAN] Client disconnected — back to listen");
        ledOff();
        Serial.println("[BLE] Client disconnected");
    }
};

// ── BLE init ──────────────────────────────────────────────────────────────────

void initBle() {
    NimBLEDevice::init("CAN-DONGLE");
    NimBLEDevice::setPower(9);
    // NimBLE's default preferred MTU is ~255, which caps a notification at ~252 bytes.
    NimBLEDevice::setMTU(kPreferredMtu);

    gServer = NimBLEDevice::createServer();
    gServer->setCallbacks(new ServerCallbacks());
    gServer->advertiseOnDisconnect(true);

    // ── NUS service ──
    NimBLEService* nusSvc = gServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

    // RX: phone → dongle commands (F.1)
    NimBLECharacteristic* nusRxChar = nusSvc->createCharacteristic(
        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::WRITE);
    nusRxChar->setCallbacks(new NusRxCallbacks());

    // TX: dongle → phone CAN frames
    gTxCharacteristic = nusSvc->createCharacteristic(
        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::NOTIFY);

    nusSvc->start();

    // ── OTA service ──
    NimBLEService* otaSvc = gServer->createService("6E410001-B5A3-F393-E0A9-E50E24DCCA9E");

    NimBLECharacteristic* otaCtrlChar = otaSvc->createCharacteristic(
        "6E410002-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::WRITE);
    otaCtrlChar->setCallbacks(new OtaCtrlCallbacks());

    NimBLECharacteristic* otaDataChar = otaSvc->createCharacteristic(
        "6E410003-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::WRITE_NR);
    otaDataChar->setCallbacks(new OtaDataCallbacks());

    gOtaStatusChar = otaSvc->createCharacteristic(
        "6E410004-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::NOTIFY);

    NimBLECharacteristic* otaVersionChar = otaSvc->createCharacteristic(
        "6E410005-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::READ);
    otaVersionChar->setValue(kFirmwareVersion);

    otaSvc->start();

    gAdvertising = NimBLEDevice::getAdvertising();
    gAdvertising->setName("CAN-DONGLE");
    gAdvertising->addServiceUUID(nusSvc->getUUID());
    gAdvertising->enableScanResponse(true);
    gAdvertising->start();
    Serial.println("[BLE] Advertising as CAN-DONGLE");
}

// ── CAN init ──────────────────────────────────────────────────────────────────

bool initCan() {
    // Starts LISTEN_ONLY: the controller never drives TX, so the transceiver stays recessive
    twai_general_config_t gCfg =
        TWAI_GENERAL_CONFIG_DEFAULT(kCanTxPin, kCanRxPin, TWAI_MODE_LISTEN_ONLY);
    gCfg.rx_queue_len = 64;
    twai_timing_config_t tCfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t fCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&gCfg, &tCfg, &fCfg) != ESP_OK) return false;
    if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
    return true;
}

// ── Frame packing helpers ─────────────────────────────────────────────────────

bool appendFrame(const twai_message_t& msg) {
    char line[72];
    int len;
    if (msg.extd) {
        len = snprintf(line, sizeof(line), "EXT,0x%08lX,%u", msg.identifier, msg.data_length_code);
    } else {
        len = snprintf(line, sizeof(line), "STD,0x%03lX,%u", msg.identifier, msg.data_length_code);
    }
    for (uint8_t i = 0; i < msg.data_length_code && len < (int)sizeof(line) - 8; i++) {
        len += snprintf(line + len, sizeof(line) - len, ",%02X", msg.data[i]);
    }
    len += snprintf(line + len, sizeof(line) - len, ",%u", (unsigned)(millis() % 65536));
    line[len++] = '\n';
    const bool ok = appendLine(line, len);
    if (ok) gStatCanFrames++; else gStatCanDropped++;
    return ok;
}

// Every received frame, before the filter: the ack proof's frame count, and the repeat
// count (a frame identical to the one just before, in the same ms, is almost always a
// resend of an unacknowledged frame).
void countFrame(const twai_message_t& msg) {
    gModeFrames++;
    const uint32_t now = millis();
    if (now == gPrevMsgMs && msg.identifier == gPrevMsg.identifier && msg.extd == gPrevMsg.extd &&
        msg.data_length_code == gPrevMsg.data_length_code &&
        memcmp(msg.data, gPrevMsg.data, msg.data_length_code) == 0) {
        gStatCanRepeats++;
    }
    gPrevMsg = msg;
    gPrevMsgMs = now;
}

bool canIdWanted(uint32_t id) {
    if (gFilterCount == 0) return true;
    for (int i = 0; i < gFilterCount; i++) {
        if (gFilterIds[i] == id) return true;
    }
    return false;
}

// Append one whole line. A notification must fit the negotiated MTU, or the stack
// cuts it short, so flush first when the line would not fit. Returns false (line
// dropped) only if the previous notification could not be sent yet.
bool appendLine(const char* line, int len) {
    if (gTxBufLen + len > gNotifyMax) flushBuf();
    if (gTxBufLen + len > gNotifyMax) return false;
    if (gTxBufLen == 0) gBufFirstLineMs = millis();
    memcpy(gTxBuf + gTxBufLen, line, len);
    gTxBufLen += len;
    return true;
}

void flushBuf() {
    if (gTxBufLen == 0 || !gClientConnected || gTxCharacteristic == nullptr) {
        gTxBufLen = 0;
        return;
    }
    gTxCharacteristic->setValue(reinterpret_cast<uint8_t*>(gTxBuf), gTxBufLen);
    // Keep the buffer if the stack has no room for it yet; the next flush retries.
    if (gTxCharacteristic->notify()) {
        gStatNotifies++;
        gStatNotifyBytes += gTxBufLen;
        gTxBufLen = 0;
    } else {
        gStatNotifyFails++;
    }
}

// ── IMU (BNO085) ──────────────────────────────────────────────────────────────

void enableImuReports() {
    // Game rotation vector (gyro+accel fusion, NO magnetometer): gravity-referenced
    // orientation — all we need for the yaw projection and vehicle-frame accel rotation.
    // Run it at 10 Hz, NOT 50 Hz: three fused reports at 50 Hz starve the BNO085's
    // scheduler and the rotation-vector report dies after one sample (observed: frozen
    // quaternion while gyro/accel kept streaming; confirmed it streams again at 10 Hz).
    // Orientation changes slowly, so 10 Hz is plenty; yaw rate still comes from 50 Hz gyro.
    gImu.enableGameRotationVector(kImuRvReportMs);  // 10 Hz orientation quaternion
    gImu.enableLinearAccelerometer(kImuReportMs);   // 50 Hz gravity-removed accel (m/s²)
    gImu.enableGyro(kImuReportMs);                  // 50 Hz calibrated rate (rad/s); yaw = Z
}

// Attempt I2C begin + report enable. Returns true once the sensor is up. Retried from
// loop() until it succeeds — the BNO085 can need >100 ms after power-on to respond.
bool tryInitImu() {
    if (gImuReady) return true;
    if (!gImu.begin(kImuAddr, Wire, -1, -1)) {
        Serial.println("[IMU] BNO085 not found @0x4B (will retry)");
        return false;
    }
    gImuReady = true;
    enableImuReports();
    Serial.println("[IMU] BNO085 @0x4B ready");
    return true;
}

void initImu() {
    Wire.begin();                  // XIAO C6 default: SDA=D4 (GPIO22), SCL=D5 (GPIO23)
    Wire.setClock(kImuI2cHz);
    // First attempt; if the BNO085 hasn't finished its power-on boot yet this fails and
    // loop() retries via tryInitImu() (poll mode, INT=-1, so getSensorEvent never blocks).
    tryInitImu();
}

// Drain pending sensor events into the latest-value cache (non-blocking, bounded).
void serviceImu() {
    if (!gImuReady) return;
    if (gImu.wasReset()) {
        gStatImuResets++;
        enableImuReports();
    }
    for (int i = 0; i < kImuDrainMax && gImu.getSensorEvent(); i++) {
        gStatImuEvents++;
        switch (gImu.getSensorEventID()) {
            case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
                gQw = gImu.getGameQuatReal(); gQx = gImu.getGameQuatI();
                gQy = gImu.getGameQuatJ();    gQz = gImu.getGameQuatK();
                gImuHasData = true;
                break;
            case SENSOR_REPORTID_LINEAR_ACCELERATION:
                gAx = gImu.getLinAccelX(); gAy = gImu.getLinAccelY(); gAz = gImu.getLinAccelZ();
                gImuHasData = true;
                break;
            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
                gGx = gImu.getGyroX(); gGy = gImu.getGyroY(); gGz = gImu.getGyroZ();
                gImuHasData = true;
                break;
        }
    }
}

// Emit an IMU line on a fixed 50 Hz schedule. The schedule advances by the period,
// not to "now", so time spent draining CAN delays one line without stretching every
// interval after it (resetting to now made the rate fall to ~40 Hz under CAN load).
// If it falls more than a period behind, it resyncs instead of bursting lines.
void emitImuIfDue() {
    if (!gImuReady) return;
    const uint32_t now = millis();
    if (now - gLastImuEmitMs < kImuEmitMs) return;
    gLastImuEmitMs += kImuEmitMs;
    if (now - gLastImuEmitMs >= kImuEmitMs) gLastImuEmitMs = now;
    serviceImu();  // freshest readings for this line
    appendImu();
}

// Append a combined IMU line (same trailing millis%65536 timestamp scheme as frames):
//   IMU,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,<ts>\n
void appendImu() {
    if (!gImuReady || !gImuHasData) return;
    char line[128];
    int len = snprintf(line, sizeof(line),
        "IMU,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%u\n",
        gQw, gQx, gQy, gQz, gAx, gAy, gAz, gGx, gGy, gGz,
        (unsigned)(millis() % 65536));
    if (len <= 0) return;
    if (appendLine(line, len)) gStatImuLines++; else gStatImuDropped++;
}

// ── GPS (u-blox MAX-M10S, 1.8.0) ──────────────────────────────────────────────
//
// The module boots with NMEA output and GPS+Galileo+BeiDou B1I, which caps
// the navigation rate at 3 Hz (data sheet UBX-20035208 R08, table 2). Its UART rate is
// 9600 per the data sheet, but the breakout on the bench dongle came at 38400 (measured
// 2026-11-17, a baud scan: NMEA only at 38400), so the baud change goes out at both. Configured on
// every boot, RAM layer only (nothing written to the module's flash or battery-backed
// RAM): 115200 baud, GPS+Galileo (10 Hz max), automotive dynamic model, NMEA off,
// UBX-NAV-PVT every epoch. Redone whenever no NAV-PVT has arrived for kGpsSilentMs, so a
// module that restarted on its own (or was plugged in late) comes back.

constexpr uint32_t kGpsBaudBoot  = 9600;    // u-blox default
constexpr uint32_t kGpsBaudAlt   = 38400;   // as this breakout shipped
constexpr uint32_t kGpsBaud      = 115200;
constexpr uint16_t kGpsRateMs    = 100;     // 10 Hz
constexpr uint32_t kGpsSilentMs  = 3000;

// UBX-CFG-VALSET keys (u-blox M10 SPG 5.10 interface description)
constexpr uint32_t kKeyUart1Baud    = 0x40520001;  // U4
constexpr uint32_t kKeyRateMeas     = 0x30210001;  // U2, ms
constexpr uint32_t kKeyDynModel     = 0x20110021;  // E1, 4 = automotive
constexpr uint32_t kKeyBdsEna       = 0x10310022;  // L
constexpr uint32_t kKeyGloEna       = 0x10310025;  // L
constexpr uint32_t kKeyGalEna       = 0x10310021;  // L
constexpr uint32_t kKeyGpsEna       = 0x1031001F;  // L
constexpr uint32_t kKeyUart1Ubx     = 0x10740001;  // L, UBX output on UART1
constexpr uint32_t kKeyUart1Nmea    = 0x10740002;  // L, NMEA output on UART1
constexpr uint32_t kKeyPvtUart1     = 0x20910007;  // U1, NAV-PVT per epoch on UART1

uint32_t gGpsLastPvtMs   = 0;
uint32_t gGpsConfigAtMs  = 0;
volatile uint32_t gPpsMs = 0;          // millis() at the last PPS rising edge
volatile bool     gPpsSeen = false;
uint32_t gStatGpsLines = 0, gStatGpsDropped = 0, gStatGpsBadCk = 0, gStatGpsAcks = 0,
         gStatGpsNaks = 0, gStatGpsPvts = 0;
volatile uint32_t gPpsCount = 0;
uint32_t gStatGpsRxBytes = 0, gStatGpsNmea = 0;   // raw UART bytes; '$' seen (NMEA still on)
uint8_t  gGpsFix = 0, gGpsSats = 0;     // latest NAV-PVT, for the serial log
float    gGpsHaccM = 0;

// UBX receive state
uint8_t  gUbxBuf[100];
int      gUbxPos = 0;
uint16_t gUbxLen = 0;

void IRAM_ATTR onPps() {
    gPpsMs = millis();
    gPpsSeen = true;
    gPpsCount++;
}

void ubxSend(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 6; i++) { a += hdr[i]; b += a; }
    for (uint16_t i = 0; i < len; i++) { a += payload[i]; b += a; }
    Serial1.write(hdr, 6);
    Serial1.write(payload, len);
    Serial1.write(a);
    Serial1.write(b);
}

// One CFG-VALSET into the RAM layer. Values are written little-endian at their key's size.
struct KeyVal { uint32_t key; uint32_t val; };
void gpsValset(const KeyVal* kv, int n) {
    uint8_t p[4 + 8 * 12];
    int len = 0;
    p[len++] = 0x00;   // version
    p[len++] = 0x01;   // layers: RAM
    p[len++] = 0; p[len++] = 0;
    for (int i = 0; i < n; i++) {
        memcpy(p + len, &kv[i].key, 4); len += 4;
        const int size = (kv[i].key >> 28) & 0x7;    // 1 = bit, 2 = 1 byte, 3 = 2, 4 = 4
        const int bytes = size == 4 ? 4 : size == 3 ? 2 : 1;
        memcpy(p + len, &kv[i].val, bytes); len += bytes;
    }
    ubxSend(0x06, 0x8A, p, len);
}

// Non-blocking: the loop can't stall (the TWAI receive queue holds 64 frames, a few tens
// of ms of a busy bus), so the steps are spaced out by gpsConfigStep() from serviceGps().
//   step 1: 38400 baud, send the baud change (and UBX output on).
//   step 2 (+20 ms, once those bytes are out): the same at 9600. A module at another rate
//           (or still at 115200 from before an ESP reset) sees noise.
//   step 3 (+60 ms): 115200, constellations. A GNSS signal change restarts the
//           receiver's tracking.
//   step 4: rate, dynamic model, NMEA off, NAV-PVT on.
// Steps 3 and 4 wait for ACK-ACK and resend after kGpsAckWaitMs. After the copies sent at
// the wrong baud the module ignores what it receives for a while (bench, 2026-11-17:
// first ACK 110 ms and 400 ms after reopening at 115200 on two runs; at once without
// the copies), so step 3 usually goes out more than once.
constexpr uint32_t kGpsAckWaitMs = 200;
constexpr int      kGpsSendMax   = 10;
int      gGpsCfgStep   = 0;     // 0 = idle
uint32_t gGpsCfgStepMs = 0;
int      gGpsCfgSends  = 0;     // sends of the current step's message
volatile bool gGpsAcked = false;   // ACK-ACK for a CFG-VALSET since the last send
uint32_t gGpsCfgTries  = 0;     // since the last NAV-PVT; after 5, retry every 30 s
char     gGpsCfgResult[24] = "none";   // outcome of the last config, for the serial log

// Reopen UART1 at a new rate. Serial1.updateBaudRate() is not used: after it, the module
// acknowledged nothing sent (bench, 2026-11-17, Arduino-ESP32 on the C6) although bytes
// from the module still read correctly; end() + begin() got ACK-ACK.
void gpsSetBaud(uint32_t baud) {
    Serial1.end();
    Serial1.setRxBufferSize(2048);
    Serial1.begin(baud, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
}

void sendCfgStep(int step) {
    gGpsAcked = false;
    gGpsCfgSends++;
    gGpsCfgStepMs = millis();
    if (step == 3) {
        const KeyVal sig[] = {{kKeyGpsEna, 1}, {kKeyGalEna, 1}, {kKeyBdsEna, 0}, {kKeyGloEna, 0}};
        gpsValset(sig, 4);
    } else {
        const KeyVal nav[] = {{kKeyRateMeas, kGpsRateMs}, {kKeyDynModel, 4},
                              {kKeyUart1Nmea, 0}, {kKeyPvtUart1, 1}};
        gpsValset(nav, 4);
    }
}

void configureGps() {
    gGpsCfgStep = 1;
    gGpsCfgStepMs = millis();
    gGpsConfigAtMs = millis();
    gGpsCfgTries++;
    gpsSetBaud(kGpsBaudAlt);
    const KeyVal baud[] = {{kKeyUart1Baud, kGpsBaud}, {kKeyUart1Ubx, 1}};
    gpsValset(baud, 2);
}

void gpsConfigStep() {
    const uint32_t now = millis();
    if (gGpsCfgStep == 1) {
        if (now - gGpsCfgStepMs < 20) return;
        gpsSetBaud(kGpsBaudBoot);
        const KeyVal baud[] = {{kKeyUart1Baud, kGpsBaud}, {kKeyUart1Ubx, 1}};
        gpsValset(baud, 2);
        gGpsCfgStep = 2;
        gGpsCfgStepMs = now;
    } else if (gGpsCfgStep == 2) {
        if (now - gGpsCfgStepMs < 60) return;
        gpsSetBaud(kGpsBaud);
        gGpsCfgStep = 3;
        gGpsCfgSends = 0;
        sendCfgStep(3);
    } else if (gGpsAcked) {
        if (gGpsCfgStep == 3) {
            gGpsCfgStep = 4;
            gGpsCfgSends = 0;
            sendCfgStep(4);
        } else {
            gGpsCfgStep = 0;
            snprintf(gGpsCfgResult, sizeof(gGpsCfgResult), "ok@%lums", (unsigned long)now);
            Serial.printf("[GPS] Configured (try %lu): 115200, GPS+GAL, 10 Hz, NAV-PVT\n", gGpsCfgTries);
        }
    } else if (now - gGpsCfgStepMs >= kGpsAckWaitMs) {
        if (gGpsCfgSends < kGpsSendMax) {
            sendCfgStep(gGpsCfgStep);
        } else {
            Serial.printf("[GPS] No ACK at step %d after %d sends (try %lu)\n",
                          gGpsCfgStep, gGpsCfgSends, gGpsCfgTries);
            snprintf(gGpsCfgResult, sizeof(gGpsCfgResult), "noack%d@%lums", gGpsCfgStep, (unsigned long)now);
            gGpsCfgStep = 0;
        }
    }
}

void initGps() {
    Serial1.setRxBufferSize(2048);
    Serial1.begin(kGpsBaud, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
    // PPS: plain input, no pull, never driven. TIMEPULSE is tied to SAFEBOOT_N inside
    // the module (1 kΩ); held low at the module's power-on it boots into safeboot.
    pinMode(kGpsPpsPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kGpsPpsPin), onPps, RISING);
    configureGps();
}

// Days since 1970-01-01 for a proleptic Gregorian date (Howard Hinnant's days_from_civil).
int64_t daysFromCivil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

template <typename T> T rd(const uint8_t* p) { T v; memcpy(&v, p, sizeof(T)); return v; }

// GPS line, one per NAV-PVT (same trailing millis%65536 timestamp as the other lines):
//   GPS,fix,sats,lat,lon,alt_m,speed_mps,heading_deg,hacc_m,utc_ms,pps_ts,<ts>\n
// fix = NAV-PVT fixType (0 none, 2 2D, 3 3D, ...) or 0 when gnssFixOK is clear.
// alt_m above mean sea level. utc_ms = Unix ms of the epoch, 0 until date and time are
// valid. pps_ts = millis()%65536 of the last PPS edge, -1 before the first.
void handlePvt(const uint8_t* p) {
    gGpsLastPvtMs = millis();
    gStatGpsPvts++;
    gGpsCfgTries = 0;
    const uint8_t valid = p[11], flags = p[21];
    const uint8_t fix = (flags & 0x01) ? p[20] : 0;
    uint64_t utcMs = 0;
    if ((valid & 0x03) == 0x03) {
        const int64_t days = daysFromCivil(rd<uint16_t>(p + 4), p[6], p[7]);
        const int64_t secs = days * 86400 + p[8] * 3600 + p[9] * 60 + p[10];
        const int64_t ms = secs * 1000 + rd<int32_t>(p + 16) / 1000000;
        if (ms > 0) utcMs = (uint64_t)ms;
    }
    gGpsFix = fix; gGpsSats = p[23]; gGpsHaccM = rd<uint32_t>(p + 40) / 1000.0f;
    const long ppsTs = gPpsSeen ? (long)(gPpsMs % 65536) : -1;
    char line[128];
    const int len = snprintf(line, sizeof(line),
        "GPS,%u,%u,%.7f,%.7f,%.2f,%.2f,%.1f,%.2f,%llu,%ld,%u\n",
        fix, p[23], rd<int32_t>(p + 28) * 1e-7, rd<int32_t>(p + 24) * 1e-7,
        rd<int32_t>(p + 36) / 1000.0, rd<int32_t>(p + 60) / 1000.0,
        rd<int32_t>(p + 64) * 1e-5, rd<uint32_t>(p + 40) / 1000.0,
        (unsigned long long)utcMs, ppsTs, (unsigned)(millis() % 65536));
    if (len <= 0 || !gClientConnected) return;
    if (appendLine(line, len)) gStatGpsLines++; else gStatGpsDropped++;
}

void handleUbx(uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len) {
    if (cls == 0x01 && id == 0x07 && len == 92) handlePvt(p);
    else if (cls == 0x05 && id == 0x01) {
        gStatGpsAcks++;
        if (len >= 2 && p[0] == 0x06 && p[1] == 0x8A) gGpsAcked = true;
        Serial.printf("[GPS] ACK for 0x%02X 0x%02X\n", len >= 2 ? p[0] : 0, len >= 2 ? p[1] : 0);
    }
    else if (cls == 0x05 && id == 0x00) {
        gStatGpsNaks++;
        Serial.printf("[GPS] NAK for 0x%02X 0x%02X\n", len >= 2 ? p[0] : 0, len >= 2 ? p[1] : 0);
    }
}

// Byte-wise UBX framing: B5 62 cls id lenL lenH payload ckA ckB. Anything else
// (leftover NMEA, noise at the wrong baud) is skipped.
void serviceGps() {
    int budget = 512;
    while (budget-- > 0 && Serial1.available()) {
        const uint8_t c = Serial1.read();
        gStatGpsRxBytes++;
        if (c == '$') gStatGpsNmea++;
        if (gUbxPos == 0) { if (c == 0xB5) gUbxBuf[gUbxPos++] = c; continue; }
        if (gUbxPos == 1) { gUbxPos = c == 0x62 ? 2 : 0; if (gUbxPos) gUbxBuf[1] = c; continue; }
        if (gUbxPos < 6) {
            gUbxBuf[gUbxPos++] = c;
            if (gUbxPos == 6) {
                gUbxLen = gUbxBuf[4] | (gUbxBuf[5] << 8);
                if (gUbxLen + 8 > (int)sizeof(gUbxBuf)) gUbxPos = 0;   // not a message we read
            }
            continue;
        }
        gUbxBuf[gUbxPos++] = c;
        if (gUbxPos == gUbxLen + 8) {
            uint8_t a = 0, b = 0;
            for (int i = 2; i < 6 + gUbxLen; i++) { a += gUbxBuf[i]; b += a; }
            if (a == gUbxBuf[6 + gUbxLen] && b == gUbxBuf[7 + gUbxLen]) {
                handleUbx(gUbxBuf[2], gUbxBuf[3], gUbxBuf + 6, gUbxLen);
            } else {
                gStatGpsBadCk++;
            }
            gUbxPos = 0;
        }
    }
    if (gGpsCfgStep != 0) { gpsConfigStep(); return; }
    const uint32_t now = millis();
    const uint32_t retryMs = gGpsCfgTries >= 5 ? 30000 : kGpsSilentMs;
    if (now - gGpsLastPvtMs >= kGpsSilentMs && now - gGpsConfigAtMs >= retryMs) {
        configureGps();
    }
}

} // namespace

// ── Arduino entry points ──────────────────────────────────────────────────────

// Overrides the core's weak default (false), which marks a pending OTA image valid
// before setup() runs. Returning true leaves that to markAppValid() (1.7.2).
extern "C" bool verifyRollbackLater() { return true; }

void setup() {
    // S LOW: transceiver Normal. TXD idles recessive until TWAI starts in LISTEN_ONLY.
    pinMode(kCanSPin, OUTPUT);
    digitalWrite(kCanSPin, LOW);

    pinMode(kLedPin, OUTPUT);
    ledOff();
    Serial.begin(115200);
    delay(300);

    // Not marked valid here: see "OTA safety" (markAppValid on the first BLE connect).

    initBle();
    gCanReady = initCan();
    if (!gCanReady) {
        Serial.println("[WARN] CAN init failed, BLE still advertising");
    } else {
        Serial.printf("[OK] CAN @ %lu baud, listen, firmware v%s\n",
                      kBaud, kFirmwareVersion);
    }
    initImu();
    initGps();
}

// Applies ack policy commands, switches listen -> ack once the bus is proven (AUTO),
// and drops back to listen when ack mode sees errors. Called from loop().
void updateAckMode() {
    if (gAckPolicyChanged) {
        gAckPolicyChanged = false;
        const AckPolicy policy = gAckPolicy;
        if (policy != AckPolicy::OFF) gAckFellBack = false;  // a connect or a command starts over
        if (policy == AckPolicy::OFF && gTxEnabled) reinitCan(false);
        if (policy == AckPolicy::ON && !gTxEnabled) reinitCan(true);
    }
    if (!gCanReady || millis() - gLastAckCheckMs < kAckCheckMs) return;
    gLastAckCheckMs = millis();
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) return;

    if (!gTxEnabled) {
        if (gAckPolicy == AckPolicy::AUTO && !gAckFellBack && gModeFrames >= kAckProofFrames &&
            st.bus_error_count * kAckProofRatio < gModeFrames) {
            Serial.printf("[CAN] Bus proven (%lu frames, %lu bus errors) — acknowledging\n",
                          gModeFrames, st.bus_error_count);
            reinitCan(true);
        }
        return;
    }
    const uint32_t frames = gModeFrames - gAckCheckFrames;
    const uint32_t errors = st.bus_error_count - gAckCheckErrors;
    gAckCheckFrames = gModeFrames;
    gAckCheckErrors = st.bus_error_count;
    if (st.state != TWAI_STATE_RUNNING || st.rx_error_counter >= kAckErrorWarn ||
        st.tx_error_counter >= kAckErrorWarn || (errors > 5 && errors * kAckProofRatio > frames)) {
        Serial.printf("[CAN] Ack fallback: state=%d rec=%lu tec=%lu errors=%lu frames=%lu — listening\n",
                      (int)st.state, st.rx_error_counter, st.tx_error_counter, errors, frames);
        gAckFellBack = true;
        reinitCan(false);
    }
}

// USB serial bench command: "GPSCFG" re-runs the GPS configuration.
void serviceSerialCommands() {
    static char cmd[16];
    static int n = 0;
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmd[n] = 0;
            if (strcmp(cmd, "GPSCFG") == 0) configureGps();
            n = 0;
        } else if (n < (int)sizeof(cmd) - 1) {
            cmd[n++] = c;
        }
    }
}

void printStats() {
    const uint32_t now = millis();
    if (now - gLastStatMs < kStatIntervalMs) return;
    gLastStatMs = now;
    Serial.printf("[STAT] imu_lines=%lu imu_drop=%lu imu_events=%lu imu_resets=%lu imu_ready=%d "
                  "can=%lu can_drop=%lu notify_fail=%lu max_pass_ms=%lu mtu_payload=%d filter=%d\n",
                  gStatImuLines, gStatImuDropped, gStatImuEvents, gStatImuResets, (int)gImuReady,
                  gStatCanFrames, gStatCanDropped, gStatNotifyFails, gStatMaxPassMs,
                  (int)gNotifyMax, gFilterCount);
    // Same counters to the phone, so they can be read while the dongle is on a bus
    // and not on USB: STAT,imu_lines,imu_drop,imu_events,imu_resets,imu_ready,
    // can,can_drop,notify_fail,max_pass_ms,notifies,notify_bytes,can_mode,can_repeats,bus_errors
    twai_status_info_t st = {};
    const bool haveStatus = gCanReady && twai_get_status_info(&st) == ESP_OK;
    const uint32_t busErrors = haveStatus ? st.bus_error_count - gStatBusErrorsAt : 0;
    if (haveStatus) gStatBusErrorsAt = st.bus_error_count;
    const int canMode = gTxEnabled ? 1 : (gAckFellBack ? 2 : 0);
    Serial.printf("[STAT] can_mode=%d can_repeats=%lu bus_errors=%lu\n", canMode, gStatCanRepeats, busErrors);
    const uint32_t pps = gPpsCount;
    gPpsCount = 0;
    Serial.printf("[GPS] rx_bytes=%lu nmea=%lu baud=%lu pvt=%lu lines=%lu drop=%lu bad_ck=%lu acks=%lu naks=%lu pps=%lu fix=%u sats=%u hacc_m=%.2f cfg=%s\n",
                  gStatGpsRxBytes, gStatGpsNmea, (unsigned long)Serial1.baudRate(), gStatGpsPvts, gStatGpsLines, gStatGpsDropped, gStatGpsBadCk, gStatGpsAcks,
                  gStatGpsNaks, pps, gGpsFix, gGpsSats, gGpsHaccM, gGpsCfgResult);
    gStatGpsRxBytes = gStatGpsNmea = gStatGpsPvts = gStatGpsLines = gStatGpsDropped = gStatGpsBadCk = gStatGpsAcks = gStatGpsNaks = 0;
    char line[192];
    const int len = snprintf(line, sizeof(line), "STAT,%lu,%lu,%lu,%lu,%d,%lu,%lu,%lu,%lu,%lu,%lu,%d,%lu,%lu\n",
                             gStatImuLines, gStatImuDropped, gStatImuEvents, gStatImuResets,
                             (int)gImuReady, gStatCanFrames, gStatCanDropped, gStatNotifyFails,
                             gStatMaxPassMs, gStatNotifies, gStatNotifyBytes,
                             canMode, gStatCanRepeats, busErrors);
    if (len > 0 && gClientConnected) appendLine(line, len);
    gStatImuLines = gStatImuDropped = gStatImuEvents = gStatImuResets = 0;
    gStatCanFrames = gStatCanDropped = gStatNotifyFails = gStatMaxPassMs = 0;
    gStatNotifies = gStatNotifyBytes = gStatCanRepeats = 0;
}

void loop() {
    const uint32_t passStart = millis();
    // Apply a FILTER command (or the reset on connect). Clear the flag before copying:
    // a command arriving mid-copy sets it again and is applied on the next pass.
    if (gFilterRequested) {
        gFilterRequested = false;
        const int n = gPendingFilterCount;
        memcpy(gFilterIds, gPendingFilter, n * sizeof(uint32_t));
        gFilterCount = n;
    }

    // Restart advertising if client dropped
    if (!gClientConnected && gAdvertising != nullptr &&
        millis() - gLastAdvCheckMs >= 2000) {
        gLastAdvCheckMs = millis();
        gAdvertising->start();
    }

    // Apply deferred CAN reinit (from BLE callback or disconnect handler)
    if (gReinitRequested) {
        gReinitRequested = false;
        reinitCan(gReinitTxTarget);
    }

    if (gMarkValidRequested) {
        gMarkValidRequested = false;
        markAppValid();
    }
    if (gOtaCommitRequested) {
        gOtaCommitRequested = false;
        otaCommit();  // restarts on success
    }

    // Pause CAN streaming during OTA
    if (gOtaPhase != OtaPhase::IDLE) {
        delay(10);
        return;
    }

    // Bus-off recovery — check every 500 ms, attempt reinit at most once per 5 s
    if (gCanReady && millis() - gLastBusOffCheckMs >= 500) {
        gLastBusOffCheckMs = millis();
        twai_status_info_t info;
        if (twai_get_status_info(&info) == ESP_OK &&
            info.state == TWAI_STATE_BUS_OFF &&
            millis() - gLastBusOffRecoveryMs >= 5000) {
            Serial.println("[CAN] Bus-off — attempting recovery");
            gLastBusOffRecoveryMs = millis();
            gAckFellBack = gAckFellBack || gTxEnabled;  // only ack mode can reach bus-off
            reinitCan(false);
        }
    }
    updateAckMode();

    if (gCanReady) {
        twai_message_t msg;
        while (twai_receive(&msg, 0) == ESP_OK) {
            countFrame(msg);
            if (canIdWanted(msg.identifier)) appendFrame(msg);
            if (gTxBufLen > gNotifyMax - kMaxLineLen) flushBuf();
            emitImuIfDue();  // a long CAN burst must not hold the IMU back
            serviceGps();
        }
        if (twai_receive(&msg, pdMS_TO_TICKS(kFlushIntervalMs)) == ESP_OK) {
            countFrame(msg);
            if (canIdWanted(msg.identifier)) appendFrame(msg);
        }
    } else {
        // No CAN pacing — give the IMU a steady cadence without busy-spinning.
        delay(kFlushIntervalMs);
    }

    // IMU: retry init until the sensor answers, then cache reports + emit at 50 Hz.
    if (!gImuReady && millis() - gLastImuTryMs >= 1000) {
        gLastImuTryMs = millis();
        tryInitImu();
    }
    serviceImu();
    emitImuIfDue();
    serviceGps();
    serviceSerialCommands();

    if (gTxBufLen > 0 && millis() - gBufFirstLineMs >= kMaxBufferAgeMs) flushBuf();
    const uint32_t pass = millis() - passStart;
    if (pass > gStatMaxPassMs) gStatMaxPassMs = pass;
    printStats();
}
