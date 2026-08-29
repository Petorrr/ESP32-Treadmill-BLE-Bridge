// main.cpp — ESP32 Treadmill BLE Bridge BUILD 12
// Changes from BUILD 12:
//      Added support for Dionysis RUN500 treadmill (FTMS + FitShow) — see platformio.ini for board selection.
//      Running on an ESP32-C3 SuperMini, the bridge exposes a BLE Speed and Cadence sensor profile to the watch (instead of FTMS proxy).

// Changes from BUILD 11:
//   [FIX] Step counter new-session detection improved.
//         BUILD 11 relied on "60s of zeros" but FitShow holds the last non-zero
//         value when treadmill stops — zeros never arrive.
//         New logic (two independent triggers):
//           1. rawSteps jumps to 0 from a non-overflow value (gLastRawSteps < 9500)
//              → immediate reset (new session started)
//           2. rawSteps value frozen for 60s → reset (treadmill stopped)
//         Overflow (9999→0→1) still detected: drop > 500 AND gLastRawSteps >= 9500.

#include <Arduino.h>
#include <NimBLEDevice.h>

#define ESP32_BUILD 13
#define LED_PIN     8

#define RSC_SERVICE_UUID        "1814"
#define RSC_MEASUREMENT_UUID    "2A53"
#define RSC_FEATURE_UUID        "2A54"
#define RSC_SENSOR_LOCATION_UUID "2A5D"

// ── Treadmill identity ──────────────────────────────────────────────────────
// EXILE TREX SPORT TX-400WP — random static BLE address
static const char* TREADMILL_MAC = "e3:97:52:37:c6:30";

// ── BLE UUIDs ────────────────────────────────────────────────────────────────
static const char* FTMS_SERVICE_UUID        = "00001826-0000-1000-8000-00805f9b34fb";
static const char* TREADMILL_DATA_CHAR_UUID = "00002acd-0000-1000-8000-00805f9b34fb";
static const char* CCCD_UUID                = "00002902-0000-1000-8000-00805f9b34fb";
// FitShow proprietary service — sends live data (incl. step count) on 0xFFF1
static const char* FITSHOW_SERVICE_UUID     = "0000fff0-0000-1000-8000-00805f9b34fb";
static const char* FITSHOW_NOTIFY_CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb";

// ── Global state ─────────────────────────────────────────────────────────────
// These flags are set in NimBLE callbacks (separate FreeRTOS task)
// and read in loop() (main task) — volatile prevents compiler optimizations
static NimBLEClient*         pClient     = nullptr;
static NimBLECharacteristic* p2acdChar   = nullptr;  // server-side char for watch notifications
static NimBLEAddress         treadmillAddress;        // copied from scan result (safe after rescan)
static volatile bool         doConnect          = false;
static volatile bool         treadmillConnected = false;
static volatile bool         watchConnected     = false;
static NimBLECharacteristic* rscMeasurementChar = nullptr;  // Garmin RSC notify char

// ── Treadmill data (written in callbacks, read in sendToWatch) ───────────────
static uint16_t gRawSpeed    = 0;   // 0.01 km/h resolution (from FTMS 0x2ACD)
static uint32_t gDistanceM   = 0;   // meters              (from FTMS 0x2ACD)
static uint16_t gElapsedSec  = 0;   // seconds             (from FTMS 0x2ACD)
static uint32_t gStepCount      = 0;  // cumulative total steps (survives mid-session overflows)
static uint32_t gStepOffset     = 0;  // steps accumulated from past 10000-step overflows
static uint16_t gLastRawSteps   = 0;  // last raw value from FitShow — used to detect overflow
static uint32_t gZeroStepsStartMs = 0; // millis() when FitShow first reported 0 steps;
                                        // if 0 for >60s → new session → reset counters

// ── Forward declarations ─────────────────────────────────────────────────────
void sendToWatch();
bool connectToTreadmill();

// =============================================================================
// Watch notification — 9-byte packet over FTMS proxy
//   Bytes 0-1: raw speed  (uint16 LE, 0.01 km/h)
//   Bytes 2-4: distance   (uint24 LE, meters)
//   Bytes 5-6: elapsed    (uint16 LE, seconds)
//   Bytes 7-8: step count (uint16 LE, total steps)  ← added in BUILD 8
// =============================================================================
void sendToWatch() {
    if (!watchConnected || p2acdChar == nullptr) return;

    uint8_t pkt[9];
    pkt[0] = (uint8_t)(gRawSpeed & 0xFF);
    pkt[1] = (uint8_t)((gRawSpeed >> 8) & 0xFF);
    pkt[2] = (uint8_t)(gDistanceM & 0xFF);
    pkt[3] = (uint8_t)((gDistanceM >> 8) & 0xFF);
    pkt[4] = (uint8_t)((gDistanceM >> 16) & 0xFF);
    pkt[5] = (uint8_t)(gElapsedSec & 0xFF);
    pkt[6] = (uint8_t)((gElapsedSec >> 8) & 0xFF);
    // Steps sent as uint16 — max 65535, sufficient for any desk treadmill session
    uint16_t stepsSend = (gStepCount > 0xFFFF) ? 0xFFFF : (uint16_t)gStepCount;
    pkt[7] = (uint8_t)(stepsSend & 0xFF);
    pkt[8] = (uint8_t)((stepsSend >> 8) & 0xFF);

    p2acdChar->setValue(pkt, sizeof(pkt));
    p2acdChar->notify();
}


void sendRscMeasurement() {
    if (!rscMeasurementChar) {
        return;
    }

    // FTMS speed: 0.01 km/h
    float speedKmh = gRawSpeed * 0.01f;

    // RSC speed: 1/256 m/s
    uint16_t speedRaw =
        (uint16_t)round((speedKmh / 3.6f) * 256.0f);

    // RSC total distance: 0.1 meter
    uint32_t distanceRaw = gDistanceM * 10;

    uint8_t data[8];

    // Flags:
    // bit 1 = Total Distance Present
    data[0] = 0x02;

    // Instantaneous Speed
    data[1] = speedRaw & 0xFF;
    data[2] = (speedRaw >> 8) & 0xFF;

    // Instantaneous Cadence
    data[3] = 0;

    // Total Distance
    data[4] = distanceRaw & 0xFF;
    data[5] = (distanceRaw >> 8) & 0xFF;
    data[6] = (distanceRaw >> 16) & 0xFF;
    data[7] = (distanceRaw >> 24) & 0xFF;

    rscMeasurementChar->setValue(data, sizeof(data));
    rscMeasurementChar->notify();
}

// =============================================================================
// FTMS 0x2ACD parser — extracts speed, distance, elapsed time from treadmill
// =============================================================================
// Packet format: flags(2 bytes) + variable fields based on flag bits.
// See PROJECT.md "FTMS 0x2ACD Packet Format" for full field table.
bool parseTreadmillData(uint8_t* p, size_t len) {
    if (len < 2) return false;

    uint16_t flags = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    size_t idx = 2;

    // Helper lambdas for little-endian reads with bounds safety.
    // On out-of-bounds: return 0 but ALWAYS advance idx so subsequent
    // fields parse at correct offsets (even if garbage).
    auto readU8  = [&]() -> uint8_t  {
        if (idx >= len) { idx++; return 0; }
        return p[idx++];
    };
    auto readU16 = [&]() -> uint16_t {
        if (idx + 1 >= len) { idx += 2; return 0; }
        uint16_t v = (uint16_t)p[idx] | ((uint16_t)p[idx+1] << 8);
        idx += 2;
        return v;
    };
    auto readS16 = [&]() -> int16_t  { return (int16_t)readU16(); };
    auto readU24 = [&]() -> uint32_t {
        if (idx + 2 >= len) { idx += 3; return 0; }
        uint32_t v = (uint32_t)p[idx] | ((uint32_t)p[idx+1]<<8) | ((uint32_t)p[idx+2]<<16);
        idx += 3;
        return v;
    };

    // Bit 0 = 0: Instantaneous Speed present (mandatory when bit is 0)
    if (!(flags & 0x0001)) { gRawSpeed   = readU16(); }
    // Bit 1: Average Speed
    if (flags & 0x0002) readU16();
    // Bit 2: Total Distance (uint24)
    if (flags & 0x0004) { gDistanceM  = readU24(); }
    // Bit 3: Inclination + Ramp Angle
    if (flags & 0x0008) { readS16(); readS16(); }
    // Bit 4: Elevation Gain (positive + negative)
    if (flags & 0x0010) { readU16(); readU16(); }
    // Bit 5: Instantaneous Pace
    if (flags & 0x0020) readU8();
    // Bit 6: Average Pace
    if (flags & 0x0040) readU8();
    // Bit 7: Expended Energy (total + per hour + per minute)
    if (flags & 0x0080) { readU16(); readU16(); readU8(); }
    // Bit 8: Heart Rate
    if (flags & 0x0100) readU8();
    // Bit 9: Metabolic Equivalent
    if (flags & 0x0200) readU8();
    // Bit 10: Elapsed Time
    if (flags & 0x0400) { gElapsedSec = readU16(); }

    return true;
}

// =============================================================================
// FTMS notification callback — called by NimBLE when treadmill sends data
// =============================================================================
void onTreadmillData(NimBLERemoteCharacteristic* pChar,
                     uint8_t* pData, size_t length, bool isNotify)
{
    if (!parseTreadmillData(pData, length)) return;

    Serial.printf("[B%d][FTMS] spd=%.2f dst=%u t=%u steps=%u watch=%s\n",
        ESP32_BUILD,
        gRawSpeed * 0.01f,
        gDistanceM,
        gElapsedSec,
        gStepCount,
        watchConnected ? "OK" : "waiting");

    sendToWatch();
}

// =============================================================================
// FitShow 0xFFF1 callback — fires every second with live session data.
// Packet layout (17 bytes, confirmed by log analysis):
//   [00]     = 0x02  header
//   [01]     = 0x51 or 0x04  subtype
//   [02]     = unknown constant
//   [03]     = uint8  speed x0.1 km/h
//   [04]     = 0x00
//   [05-06]  = uint16 LE  elapsed time (seconds)
//   [07-08]  = uint16 LE  distance (meters)
//   [09-10]  = uint16 LE  unknown
//   [11-12]  = uint16 LE  step count (total) ← the field we want
//   [13-14]  = uint16 LE  always 0x0000
//   [15]     = uint8  checksum
//   [16]     = 0x03  end marker
// =============================================================================
void onFitShowData(NimBLERemoteCharacteristic* pChar,
                   uint8_t* pData, size_t length, bool isNotify)
{
    if (length < 13) {
        // Short packets (5-6 bytes) appear after session ends — different packet type, ignored.
        return;
    }

    // Extract step count from bytes 11-12 (uint16 little-endian)
    uint16_t rawSteps = (uint16_t)pData[11] | ((uint16_t)pData[12] << 8);

    // 60-second zero-step timeout → new session, reset all step state.
    // This fires when the treadmill is stopped and stays at 0 for a full minute.
    // It handles the case where the treadmill keeps BLE up between sessions
    // (so BLE disconnect never resets the offset).
    if (rawSteps == 0) {
        if (gZeroStepsStartMs == 0) {
            gZeroStepsStartMs = millis();  // start timing
        } else if (millis() - gZeroStepsStartMs > 60000) {
            // 60s of zeros confirmed — start fresh
            gStepOffset = 0; gLastRawSteps = 0; gStepCount = 0; gZeroStepsStartMs = 0;
            Serial.printf("[B%d][FITSHOW] 60s zero — new session, counter reset.\n", ESP32_BUILD);
            sendToWatch();
        }
        return;  // don't update step count while at 0
    }

    // rawSteps > 0: cancel the zero timer
    gZeroStepsStartMs = 0;

    // Detect mid-session overflow: treadmill resets counter to 0 after 10000 steps.
    // A drop of more than 500 from the last known value means an overflow occurred.
    if (rawSteps + 500 < gLastRawSteps) {
        gStepOffset += gLastRawSteps;
        Serial.printf("[B%d][FITSHOW] *** Overflow! offset now=%u ***\n",
            ESP32_BUILD, gStepOffset);
    }
    gLastRawSteps = rawSteps;

    uint32_t totalSteps = gStepOffset + rawSteps;
    if (totalSteps != gStepCount) {
        gStepCount = totalSteps;
        Serial.printf("[B%d][FITSHOW] steps=%u (raw=%u offset=%u)\n",
            ESP32_BUILD, gStepCount, rawSteps, gStepOffset);
        sendToWatch();
    }
}

// =============================================================================
// GATT Server callbacks — watch (CIQ DataField) connects/disconnects
// =============================================================================
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server) override {
        watchConnected = true;
        Serial.printf("[B%d][GATT] *** Watch connected! ***\n", ESP32_BUILD);
        // Restart advertising so other devices can still discover ESP32
        NimBLEDevice::getAdvertising()->start();
    }
    void onDisconnect(NimBLEServer* server) override {
        watchConnected = false;
        Serial.printf("[B%d][GATT] Watch disconnected.\n", ESP32_BUILD);
        NimBLEDevice::getAdvertising()->start();
    }
};

// =============================================================================
// BLE Client callbacks — ESP32 <-> treadmill connection state
// =============================================================================
class TreadmillClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* client) override {
        Serial.printf("[B%d][FTMS] Connected to treadmill.\n", ESP32_BUILD);
        treadmillConnected = true;
    }
    void onDisconnect(NimBLEClient* client) override {
        Serial.printf("[B%d][FTMS] Disconnected from treadmill. Restarting scan...\n", ESP32_BUILD);
        treadmillConnected = false;

        // Reset FTMS data so watch shows zeros (= treadmill offline).
        // gStepCount stays frozen at last value for display.
        // gStepOffset/gLastRawSteps are NOT reset here — new-session detection
        // is time-based (60s zero) in onFitShowData, not BLE-disconnect-based.
        gRawSpeed = 0; gDistanceM = 0; gElapsedSec = 0;
        sendToWatch();

        // Restart scanning to find treadmill again
        NimBLEDevice::getScan()->start(0, nullptr, false);
    }
};

// =============================================================================
// BLE Scan callback — Find FTMS profile on treadmill, store address, and trigger connection
// =============================================================================
class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* device) override {

    // Look for a Fitness Machine advertising the standard FTMS service.
    if (!device->isAdvertisingService(NimBLEUUID(FTMS_SERVICE_UUID))) {
      return;
    }

    Serial.printf(
      "[B%d][SCAN] FTMS treadmill found: %s\n",
      ESP32_BUILD,
      device->getName().c_str()
    );

    Serial.printf(
      "[B%d][SCAN] Address: %s\n",
      ESP32_BUILD,
      device->getAddress().toString().c_str()
    );

    NimBLEDevice::getScan()->stop();

    // Store the address only for this connection attempt.
    treadmillAddress = device->getAddress();
    doConnect = true;
  }
};

// =============================================================================
// Connect to treadmill — GATT discovery + FTMS + FitShow subscriptions
// =============================================================================
bool connectToTreadmill() {
    // Clean up previous client to avoid NimBLE client pool exhaustion.
    // NimBLE has a pool of ~3 clients — without cleanup, after 3 reconnects
    // createClient() fails and ESP32 hangs.
    if (pClient != nullptr) {
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
    }

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new TreadmillClientCallbacks(), true);
    pClient->setConnectTimeout(5);  // 5 seconds — prevents infinite hang

    // Connect using stored address (safe across rescans)
    if (!pClient->connect(treadmillAddress)) {
        Serial.printf("[B%d][FTMS] ERROR: connect() failed.\n", ESP32_BUILD);
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }

    Serial.printf("[B%d][FTMS] Connected. MTU=%d\n", ESP32_BUILD, pClient->getMTU());

    // Wait for BLE connection parameters to settle.
    // Without this delay, service discovery sometimes fails silently.
    delay(1000);

    // ── FTMS service and Treadmill Data characteristic ────────────────────
    NimBLERemoteService* pSvc = pClient->getService(FTMS_SERVICE_UUID);
    if (!pSvc) {
        Serial.printf("[B%d][FTMS] ERROR: FTMS service not found.\n", ESP32_BUILD);
        pClient->disconnect();
        return false;
    }

    NimBLERemoteCharacteristic* pChar = pSvc->getCharacteristic(TREADMILL_DATA_CHAR_UUID);
    if (!pChar || !pChar->canNotify()) {
        Serial.printf("[B%d][FTMS] ERROR: 0x2ACD not found or not notifiable.\n", ESP32_BUILD);
        pClient->disconnect();
        return false;
    }

    // Subscribe via NimBLE API (registers local callback)
    pChar->subscribe(true, onTreadmillData);

    // CRITICAL: NimBLE 1.4.x subscribe() returns true but the treadmill
    // ignores it silently. Must manually write CCCD descriptor (0x2902)
    // with [0x01, 0x00] to actually enable notifications on the treadmill.
    NimBLERemoteDescriptor* pCCCD = pChar->getDescriptor(NimBLEUUID(CCCD_UUID));
    if (pCCCD) {
        uint8_t v[] = {0x01, 0x00};
        pCCCD->writeValue(v, 2, true);
        Serial.printf("[B%d][FTMS] CCCD written. Notifications enabled.\n", ESP32_BUILD);
    } else {
        Serial.printf("[B%d][FTMS] WARNING: CCCD descriptor not found!\n", ESP32_BUILD);
    }

    Serial.printf("[B%d][FTMS] Subscribed to 0x2ACD. Data flowing.\n", ESP32_BUILD);

    // ── FitShow 0xFFF1 — live step count ─────────────────────────────────
    // Fires every second during session. Bytes 11-12 = total step count.
    NimBLERemoteService* pFitShowSvc = pClient->getService(FITSHOW_SERVICE_UUID);
    if (pFitShowSvc) {
        NimBLERemoteCharacteristic* pFitShowChar =
            pFitShowSvc->getCharacteristic(FITSHOW_NOTIFY_CHAR_UUID);
        if (pFitShowChar && pFitShowChar->canNotify()) {
            pFitShowChar->subscribe(true, onFitShowData);
            NimBLERemoteDescriptor* pFitShowCCCD =
                pFitShowChar->getDescriptor(NimBLEUUID(CCCD_UUID));
            if (pFitShowCCCD) {
                uint8_t v[] = {0x01, 0x00};
                pFitShowCCCD->writeValue(v, 2, true);
                Serial.printf("[B%d][FITSHOW] Subscribed to 0xFFF1. Step count active.\n",
                    ESP32_BUILD);
            } else {
                Serial.printf("[B%d][FITSHOW] WARNING: 0xFFF1 CCCD not found. Subscribed anyway.\n",
                    ESP32_BUILD);
            }
        } else {
            Serial.printf("[B%d][FITSHOW] WARNING: 0xFFF1 not found or not notifiable.\n",
                ESP32_BUILD);
        }
    } else {
        Serial.printf("[B%d][FITSHOW] WARNING: FitShow service 0xFFF0 not found.\n",
            ESP32_BUILD);
    }

    return true;
}

// =============================================================================
// GATT Server setup — FTMS proxy for CIQ DataField on the watch
// =============================================================================
#if 0
void setupFtmsProxyServer() {
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // ── RSC service — Garmin sensor interface ─────────────────────────────
    NimBLEService* pRscSvc =
    pServer->createService(RSC_SERVICE_UUID);

    rscMeasurementChar = pRscSvc->createCharacteristic(
    RSC_MEASUREMENT_UUID,
    NIMBLE_PROPERTY::NOTIFY
    );

pRscSvc->start();
#if 0
    NimBLEService* pSvc = pServer->createService(FTMS_SERVICE_UUID);
    p2acdChar = pSvc->createCharacteristic(
        TREADMILL_DATA_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );

    // Initial value: 9 bytes of zeros (no data yet)
    uint8_t zero[9] = {0};
    p2acdChar->setValue(zero, 9);
    pSvc->start();
#endif

    // Advertise FTMS UUID so CIQ scan finds us via getServiceUuids()
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    //pAdv->addServiceUUID(NimBLEUUID(FTMS_SERVICE_UUID));
    pAdv->addServiceUUID(NimBLEUUID(RSC_SERVICE_UUID));

    // "TreadBLE" = 8 chars — fits in BLE scan response without truncation.
    // Longer names (e.g. "TreadmillBridge") get truncated to ~8 chars by the stack.
    NimBLEAdvertisementData scanResp;
    scanResp.setName("TreadBLE");
    pAdv->setScanResponseData(scanResp);

    pAdv->start();

    Serial.printf("[B%d][GATT] FTMS proxy started. Address: %s\n",
        ESP32_BUILD, NimBLEDevice::getAddress().toString().c_str());
}
#endif

// =============================================================================
// GATT Server setup — minimal RSC sensor for Garmin
// =============================================================================
void setupFtmsProxyServer() {
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // ── RSC Service ─────────────────────────────────────────────────────
    NimBLEService* pRscSvc =
        pServer->createService(RSC_SERVICE_UUID);

    // RSC Measurement — notifications to Garmin
    rscMeasurementChar = pRscSvc->createCharacteristic(
        RSC_MEASUREMENT_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // RSC Feature — minimal feature set
    NimBLECharacteristic* pRscFeature =
        pRscSvc->createCharacteristic(
            RSC_FEATURE_UUID,
            NIMBLE_PROPERTY::READ
        );

    uint8_t feature[2] = {
        0x00, 0x00
    };
    pRscFeature->setValue(feature, sizeof(feature));

    // Sensor Location — Foot
    NimBLECharacteristic* pSensorLocation =
        pRscSvc->createCharacteristic(
            RSC_SENSOR_LOCATION_UUID,
            NIMBLE_PROPERTY::READ
        );

    uint8_t location = 0x03;   // Foot
    pSensorLocation->setValue(&location, 1);

    pRscSvc->start();

    // ── Advertising ────────────────────────────────────────────────────
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();

    pAdv->addServiceUUID(NimBLEUUID(RSC_SERVICE_UUID));

    NimBLEAdvertisementData scanResp;
    scanResp.setName("TreadBLE");
    pAdv->setScanResponseData(scanResp);

    pAdv->start();

    Serial.printf(
        "[B%d][GATT] RSC sensor started. Address: %s\n",
        ESP32_BUILD,
        NimBLEDevice::getAddress().toString().c_str()
    );
}

void sendTestRscMeasurement() {
    if (!rscMeasurementChar) {
        return;
    }

    // 2.30 km/h = 0.6389 m/s
    // RSC resolution = 1/256 m/s
    uint16_t speedRaw = 164;

    uint8_t data[4];

    data[0] = 0x00;                  // Flags
    data[1] = speedRaw & 0xFF;       // Speed LSB
    data[2] = speedRaw >> 8;         // Speed MSB
    data[3] = 0;                     // Cadence

    rscMeasurementChar->setValue(data, sizeof(data));
    rscMeasurementChar->notify();
}

// =============================================================================
// setup() — runs once at boot
// =============================================================================
void setup() {
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(115200);
    delay(1000);

    // ── BUILD BANNER — always the first lines in the log ─────────────────
    Serial.println("================================================");
    Serial.printf( "=== Treadmill BLE Bridge ESP32 BUILD %d      ===\n", ESP32_BUILD);
    Serial.println("================================================");

    NimBLEDevice::init("TreadBLE");

    // Start GATT server first — watch can connect anytime
    setupFtmsProxyServer();

    // Start scanning for the treadmill by MAC address
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), false);
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);

    Serial.printf("[B%d][SCAN] Scanning for treadmill MAC %s ...\n",
        ESP32_BUILD, TREADMILL_MAC);
    pScan->start(0, nullptr, false);
}

// =============================================================================
// loop() — main loop, handles deferred connection
// =============================================================================
void loop() {
static uint32_t lastRsc = 0;

    digitalWrite(LED_PIN, HIGH);
 

    if (millis() - lastRsc >= 1000) {
        lastRsc = millis();

        if (watchConnected) {
            sendRscMeasurement();

            Serial.printf(
                "[B%d][RSC] speed=%.2f km/h distance=%lu m\n",
                ESP32_BUILD,
                gRawSpeed * 0.01f,
                (unsigned long)gDistanceM
            );
        }
    }

    // doConnect is set by ScanCallbacks::onResult() in the NimBLE task.
    // We handle the actual connection here in the main task to avoid
    // blocking the NimBLE stack during GATT discovery.
    if (doConnect) {
        doConnect = false;
        if (!connectToTreadmill()) {
            Serial.printf("[B%d][SCAN] Reconnect failed. Restarting scan...\n", ESP32_BUILD);
            NimBLEDevice::getScan()->start(0, nullptr, false);
        }
    }
    delay(10);
    digitalWrite(LED_PIN, LOW);
}
 
