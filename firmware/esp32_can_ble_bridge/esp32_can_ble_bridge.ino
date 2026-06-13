/*
 * esp32_can_ble_bridge.ino
 * ------------------------
 * ESP32-S3 CAN listener that bridges decoded CAN frames over BLE NUS UART.
 *
 * Hardware target:
 * - Seeed Studio XIAO ESP32-C6  — D0=GPIO0, D1=GPIO1, D2=GPIO2, LED=GPIO15
 *   FQBN: esp32:esp32:XIAO_ESP32C6
 * - TJA1051 CAN transceiver: TX->D0, RX<-D1, S<-D2
 *   S pin is held LOW (listen-only) on boot; driven HIGH to enable TX mode.
 *
 * BLE services:
 *   NUS (Nordic UART Service) — CAN frame streaming + command channel
 *     TX notify:  6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (frames → phone)
 *     RX write:   6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (commands → dongle)
 *       Commands: "TX_ENABLE"  — switch TWAI to NORMAL mode, assert S pin HIGH
 *                 "TX_DISABLE" — switch TWAI to LISTEN_ONLY, pull S pin LOW
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
 *   before flushing. Early flush fires when the buffer exceeds kFlushThreshold.
 * - CAN processing is paused during OTA to avoid BLE congestion.
 *
 * TX mode safety:
 * - S pin is LOW (listen-only) on every boot regardless of last state.
 * - TX_ENABLE must be sent explicitly by the phone each session.
 * - No CAN frames are transmitted by this firmware (TX mode reserved for Phase 7).
 *   TX_ENABLE only unlocks the transceiver hardware; the TWAI driver still has
 *   no frames queued to send.
 *
 * OTA safety:
 * - esp_ota_end() validates the image magic and SHA256 before committing.
 * - The inactive partition is written; the active partition is unchanged until
 *   esp_ota_set_boot_partition() is called after successful verification.
 * - esp_ota_mark_app_valid_cancel_rollback() is called in setup() so the
 *   bootloader does not roll back to the previous image on the next boot.
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "driver/twai.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

// ── Firmware version ──────────────────────────────────────────────────────────

constexpr char kFirmwareVersion[] = "1.5.2";

// ── Hardware config ───────────────────────────────────────────────────────────

namespace {

// ── Pin config (XIAO ESP32-C6) ────────────────────────────────────────────────
constexpr gpio_num_t kCanTxPin = GPIO_NUM_0;  // D0
constexpr gpio_num_t kCanRxPin = GPIO_NUM_1;  // D1
constexpr gpio_num_t kCanSPin  = GPIO_NUM_2;  // D2 — TJA1051 S pin (LOW=listen, HIGH=normal)
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
constexpr int      kBufSize         = 512;
constexpr int      kFlushThreshold  = 400;

// OTA progress notification interval (bytes between PROGRESS notifications)
constexpr size_t kOtaProgressInterval = 16 * 1024;

// ── Runtime state ─────────────────────────────────────────────────────────────

NimBLEServer*         gServer           = nullptr;
NimBLECharacteristic* gTxCharacteristic = nullptr;   // NUS TX (frames → phone)
NimBLECharacteristic* gOtaStatusChar    = nullptr;
NimBLEAdvertising*    gAdvertising      = nullptr;

bool     gClientConnected = false;
bool     gCanReady        = false;
bool     gTxEnabled       = false;    // current TWAI mode (false = listen-only)
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

// ── Helpers ───────────────────────────────────────────────────────────────────

inline void ledOn()  { digitalWrite(kLedPin, LOW);  }
inline void ledOff() { digitalWrite(kLedPin, HIGH); }

void otaNotify(const char* msg) {
    if (gOtaStatusChar && gClientConnected) {
        gOtaStatusChar->setValue(msg);
        gOtaStatusChar->notify();
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
// Must only be called from loop() — not from BLE callback context.
void reinitCan(bool enableTx) {
    if (gCanReady) {
        twai_stop();
        twai_driver_uninstall();
        gCanReady = false;
    }

    digitalWrite(kCanSPin, enableTx ? HIGH : LOW);
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
        Serial.printf("[CAN] Mode: %s  Baud: %lu\n",
                      enableTx ? "NORMAL (TX enabled)" : "LISTEN_ONLY", gReinitBaudRate);
    } else {
        twai_driver_uninstall();
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
            gOtaPhase = OtaPhase::COMMITTING;
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
            delay(300);
            esp_restart();

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
        if (val == "TX_ENABLE") {
            Serial.println("[CMD] TX_ENABLE received");
            gReinitTxTarget  = true;
            gReinitRequested = true;
        } else if (val == "TX_DISABLE") {
            Serial.println("[CMD] TX_DISABLE received");
            gReinitTxTarget  = false;
            gReinitRequested = true;
        } else if (val.rfind("BAUD:", 0) == 0) {
            const uint32_t baud = static_cast<uint32_t>(atol(val.c_str() + 5));
            if (baud == 125000 || baud == 250000 || baud == 500000 || baud == 1000000) {
                Serial.printf("[CMD] BAUD:%lu received\n", baud);
                gReinitBaudRate  = baud;
                gReinitTxTarget  = gTxEnabled;
                gReinitRequested = true;
            } else {
                Serial.printf("[CMD] BAUD: unsupported rate %lu\n", baud);
            }
        } else {
            Serial.printf("[CMD] Unknown command: %s\n", val.c_str());
        }
    }
};

// ── NUS server callbacks ──────────────────────────────────────────────────────

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
        (void)server; (void)connInfo;
        gClientConnected = true;
        gTxBufLen = 0;
        ledOn();
        Serial.println("[BLE] Client connected");
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        (void)server; (void)connInfo; (void)reason;
        gClientConnected = false;
        if (gOtaPhase != OtaPhase::IDLE) {
            otaAbort();
            Serial.println("[OTA] Aborted — client disconnected");
        }
        // Return to listen-only on disconnect — safety default
        if (gTxEnabled) {
            gReinitTxTarget  = false;
            gReinitRequested = true;
            Serial.println("[CAN] Client disconnected — scheduling TX_DISABLE");
        }
        ledOff();
        Serial.println("[BLE] Client disconnected");
    }
};

// ── BLE init ──────────────────────────────────────────────────────────────────

void initBle() {
    NimBLEDevice::init("CAN-DONGLE");
    NimBLEDevice::setPower(9);

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
    // S pin is already LOW (set in setup), ensuring listen-only on first init
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
    if (gTxBufLen + len > kBufSize) return false;
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
    gTxCharacteristic->notify();
    gTxBufLen = 0;
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
    if (gImu.wasReset()) enableImuReports();
    for (int i = 0; i < kImuDrainMax && gImu.getSensorEvent(); i++) {
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

// Append a combined IMU line (same trailing millis%65536 timestamp scheme as frames):
//   IMU,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,<ts>\n
void appendImu() {
    if (!gImuReady || !gImuHasData) return;
    char line[128];
    int len = snprintf(line, sizeof(line),
        "IMU,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%u\n",
        gQw, gQx, gQy, gQz, gAx, gAy, gAz, gGx, gGy, gGz,
        (unsigned)(millis() % 65536));
    if (len <= 0 || gTxBufLen + len > kBufSize) return;
    memcpy(gTxBuf + gTxBufLen, line, len);
    gTxBufLen += len;
}

} // namespace

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
    // S pin LOW first — must precede BLE and CAN init so the transceiver
    // is in listen-only mode before the TWAI driver starts.
    pinMode(kCanSPin, OUTPUT);
    digitalWrite(kCanSPin, LOW);

    pinMode(kLedPin, OUTPUT);
    ledOff();
    Serial.begin(115200);
    delay(300);

    esp_ota_mark_app_valid_cancel_rollback();

    initBle();
    gCanReady = initCan();
    if (!gCanReady) {
        Serial.println("[WARN] CAN init failed, BLE still advertising");
    } else {
        Serial.printf("[OK] CAN @ %lu baud, S=LOW (listen-only), firmware v%s\n",
                      kBaud, kFirmwareVersion);
    }
    initImu();
}

void loop() {
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
            reinitCan(gTxEnabled);
        }
    }

    if (gCanReady) {
        twai_message_t msg;
        while (twai_receive(&msg, 0) == ESP_OK) {
            appendFrame(msg);
            if (gTxBufLen >= kFlushThreshold) flushBuf();
        }
        if (twai_receive(&msg, pdMS_TO_TICKS(kFlushIntervalMs)) == ESP_OK) {
            appendFrame(msg);
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
    if (gImuReady && millis() - gLastImuEmitMs >= kImuEmitMs) {
        gLastImuEmitMs = millis();
        appendImu();
    }

    flushBuf();
}
