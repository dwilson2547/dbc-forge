/*
 * esp32_can_ble_bridge.ino
 * ------------------------
 * ESP32-S3 CAN listener that bridges decoded CAN frames over BLE NUS UART.
 *
 * Hardware target:
 * - Seeed Studio XIAO ESP32-C6  — D0=GPIO0, D1=GPIO1, D2=GPIO2, LED=GPIO15
 *   FQBN: esp32:esp32:XIAO_ESP32C6
 * - TJA1051 CAN transceiver: TX->D0, RX<-D1, S<-D2
 *   S HIGH = Silent (transmitter off), S LOW = Normal (NXP TJA1051 Rev. 9, 7.1).
 *   Held HIGH on boot; LOW only in ack mode (see "CAN modes").
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
 * - Listen: TWAI LISTEN_ONLY and the transceiver Silent. Passive twice over. Every boot,
 *   every disconnect and every BAUD change starts here.
 * - Ack: TWAI NORMAL and the transceiver Normal. A sender resends a frame until some node
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

constexpr char kFirmwareVersion[] = "1.7.1";

// ── Hardware config ───────────────────────────────────────────────────────────

namespace {

// ── Pin config (XIAO ESP32-C6) ────────────────────────────────────────────────
constexpr gpio_num_t kCanTxPin = GPIO_NUM_0;  // D0
constexpr gpio_num_t kCanRxPin = GPIO_NUM_1;  // D1
constexpr gpio_num_t kCanSPin  = GPIO_NUM_2;  // D2 — TJA1051 S pin (HIGH=Silent, LOW=Normal)
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

// Stop the TWAI driver, set the S pin, and restart in the requested mode/baud.
// ack = true: TWAI NORMAL, transceiver Normal (S LOW). false: LISTEN_ONLY, Silent (S HIGH).
// Must only be called from loop() — not from BLE callback context.
void reinitCan(bool enableTx) {
    if (gCanReady) {
        twai_stop();
        twai_driver_uninstall();
        gCanReady = false;
    }

    // Going to listen: the controller is already stopped, so silencing the transceiver
    // first is safe. Going to ack: the controller starts in NORMAL only after S is LOW.
    digitalWrite(kCanSPin, enableTx ? LOW : HIGH);
    delay(5); // let the transceiver settle after S-pin change

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
                      enableTx ? "ACK (NORMAL, S=LOW)" : "LISTEN (LISTEN_ONLY, S=HIGH)", gReinitBaudRate);
    } else {
        twai_driver_uninstall();
        digitalWrite(kCanSPin, HIGH);
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
    // S pin is already HIGH (set in setup): the transceiver is Silent before TWAI starts
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

} // namespace

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
    // S pin HIGH first (transceiver Silent) — must precede BLE and CAN init so the
    // transceiver can't drive the bus before the TWAI driver starts.
    pinMode(kCanSPin, OUTPUT);
    digitalWrite(kCanSPin, HIGH);

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
        Serial.printf("[OK] CAN @ %lu baud, listen (S=HIGH), firmware v%s\n",
                      kBaud, kFirmwareVersion);
    }
    initImu();
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

    if (gTxBufLen > 0 && millis() - gBufFirstLineMs >= kMaxBufferAgeMs) flushBuf();
    const uint32_t pass = millis() - passStart;
    if (pass > gStatMaxPassMs) gStatMaxPassMs = pass;
    printStats();
}
