// main.cpp — ESP32 Treadmill BLE Bridge BUILD 14
// Changes from BUILD 13:
//

// main.cpp — ESP32 Treadmill BLE Bridge BUILD 13
// Changes from BUILD 12:
//      Added support for FTMS + RSC sensor to Garmin watches
//      The bridge can now run on any treadmill that supports FTMS (assuming no FitShow)
//      Cadence is estimated and simulated based on treadmill speed and distance
//      Tested on Dionysis RUN500 treadmill, FTMS (no FIT) + ESP as proxy from FTMS data to Garmin RSC sensor
//      Running on an ESP32-C3 SuperMini, the bridge exposes a BLE Running Speed & Cadence sensor profile to the watch (as Footpod).
//      Blue LED flashes fast (5 Hz) when both treadmill and watch are disconnected (in scanning mode)
//      Blue LED flashes slow (2 Hz) when treadmill is connected but watch is not
//      Blue LED stays solid ON when bothtreadmill and watch are connected

// Original copy from BUILD 11 (Original from FitShow bridge fork):

#include <Arduino.h>
#include <NimBLEDevice.h>

// ── Build configuration ─────────────────────────────────────────────────────
#define ESP32_BUILD     14
#define DEBUG           0     // Set to 1 to enable detailed debug prints (very verbose)
#define LED_PIN         8     // Blue LED pin on ESP32-C3 SuperMini board (active LOW)
#define LED_ON          LOW   // Led is active LOW on ESP32-C3 SuperMini board
#define LED_OFF         HIGH  // Led off is HIGH on ESP32-C3 SuperMini board
#define CADENCE_FILTER  0.8f  // Filter or cadence smoothing, 0.8 of the new value
#define VIBRATION_PIN   3     // Pin for vibration sensor (optional)
#define DEBOUNCE_TIME   250   // Debounce time in milliseconds for vibration sensor
#define SENSOR_TIMEOUT  1500  // Timeout in milliseconds for sensor data

// ── Garmin RSC (Running Speed and Cadence) UUIDs ─────────────────────────
#define RSC_SERVICE_UUID         "1814"
#define RSC_MEASUREMENT_UUID     "2A53"
#define RSC_FEATURE_UUID         "2A54"
#define RSC_SENSOR_LOCATION_UUID "2A5D"

// ── BLE UUIDs ────────────────────────────────────────────────────────────────
static const char* FTMS_SERVICE_UUID        = "00001826-0000-1000-8000-00805f9b34fb";
static const char* TREADMILL_DATA_CHAR_UUID = "00002acd-0000-1000-8000-00805f9b34fb";
static const char* CCCD_UUID                = "00002902-0000-1000-8000-00805f9b34fb";

// ── Global state ─────────────────────────────────────────────────────────────
// These flags are set in NimBLE callbacks (separate FreeRTOS task)
// and read in loop() (main task) — volatile prevents compiler optimizations
static NimBLEClient*         pClient            = nullptr;
static NimBLEAddress         treadmillAddress;        // copied from scan result (safe after rescan)
static volatile bool         doConnect          = false;
static volatile bool         treadmillConnected = false;
static volatile bool         watchConnected     = false;
static NimBLECharacteristic* rscMeasurementChar = nullptr;  // Garmin RSC notify char

// ── Treadmill data (written in callbacks, read in sendToWatch) ───────────────
static uint16_t gRawSpeed    = 0;    // 0.01 km/h resolution  (from FTMS 0x2ACD)
static uint32_t gDistanceM   = 0;    // meters                (from FTMS 0x2ACD)
static uint16_t gElapsedSec  = 0;    // seconds               (from FTMS 0x2ACD)
static uint8_t  gHeartrate   = 0;    // bpm                   (from FTMS 0x2ACD)
static uint8_t  gPace        = 0;    // seconds per kilometer (from FTMS 0x2ACD)
static uint8_t  gEnergy      = 0;    // Energy kcal / min     (from FTMS 0x2ACD)

// ── Cadence estimation (from treadmill speed) ───────────────────────────────
static float    gCadence     = 0.0f; // Steps per minute (one leg)
static float    gCadenceFilt = 0.0f; // Filtered cadence for smoother RSC output
// ── Vibration sensor (optional) ─────────────────────────────────────────────
static volatile unsigned long lastStepTime    = 0;      // Last vibration pulse time in milliseconds
static volatile unsigned long pulseTimePassed = 0;     // Vibration pulse time
static volatile float         sensorCadence   = 0.0f;  // Calculated cadence from vibration sensor (steps per minute)
// Global lock for ISR and main loop access to vibration sensor data (lastStepTime, pulseTimePassed, sensorCadence)
static portMUX_TYPE vibrationMux = portMUX_INITIALIZER_UNLOCKED;

// ── Forward declarations, funtion prototypes ────────────────────────────────
float estimateStrideLength(float speedKmh, float cadenceSpm);
void  updateCadenceEstimate(float speedKmh);
void  sendRscMeasurement();
bool  parseTreadmillData(uint8_t* p, size_t len);
void  onTreadmillData(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
bool  connectToTreadmill();
void  setupFtmsProxyServer();

// ── Functions ───────────────────────────────────────────────────────────────

// Calculate stride length from speed and cadence.
// Output: stride length in metres.
float estimateStrideLength(float speedKmh, float cadenceSpm)
{
    if (cadenceSpm <= 0.0f)
        return 0.0f;

    return (speedKmh * 1000.0f) /
           (60.0f * cadenceSpm);
}


// Estimate cadence from treadmill speed based on scientific empirical data
// Called every second when treadmill is running and no cadence sensor data
void updateCadenceEstimate(float speedKmh) {

    // Default to 0 cadence if treadmill is stopped or speed is very low
    if (speedKmh < 1.0f)
        gCadence = 0.0f;
    // Walking: 1–6 km/h
    else if (speedKmh <= 6.0f)
        gCadence = 84.0f + 6.9f * speedKmh;
    // Walking → running transition: 6–7 km/h
    else if (speedKmh < 7.0f) {
        gCadence = 125.4f + 36.29f * (speedKmh - 6.0f);
    }
    // Running >= 7 km/h: cadence increases linearly with speed
    else
        gCadence = 150.0f + 1.67f * speedKmh;

    // Constrain cadence to a reasonable range
    gCadence = constrain(gCadence, 0.0f, 200.0f);
 }


// ============================================================================
// Notify all RSC sensor data to the watch (speed, cadence, distance)
// ============================================================================
void sendRscMeasurement() {
    uint8_t data[8];
    float speedKmh;
    uint16_t speedRaw;
    uint32_t distanceRaw;

    if (!rscMeasurementChar) {
        return;
    }

    // FTMS speed: 0.01 km/h
    speedKmh = gRawSpeed * 0.01f;
    
    // Calculate the estimated cadence based on treadmill speed
    updateCadenceEstimate(speedKmh);
    // If we have a vibration sensor, use its cadence instead of the estimated one
    portENTER_CRITICAL(&vibrationMux);
    sensorCadence = (pulseTimePassed > 0) ? (60000.0f / pulseTimePassed) : 0.0f;
    portEXIT_CRITICAL(&vibrationMux);

    // RSC speed: 1/256 m/s
    speedRaw = (uint16_t)round((speedKmh / 3.6f) * 256.0f);
    // RSC total distance: 0.1 meter
    distanceRaw = gDistanceM * 10;

    // Flags: 0x06 = Total distance is present, Running flag on
    data[0] = 0x06;

    // Instantaneous Speed
    data[1] = speedRaw & 0xFF;
    data[2] = (speedRaw >> 8) & 0xFF;

    // Check if we have Cadence from the sensor input or need to estimate 
    if (sensorCadence > 0.0f) {
        // Apply filter to smooth cadence changes
        gCadenceFilt = CADENCE_FILTER * sensorCadence + (1.0f - CADENCE_FILTER) * gCadenceFilt;
    } else {
        // Apply filter to smooth cadence changes
        gCadenceFilt = CADENCE_FILTER * gCadence + (1.0f - CADENCE_FILTER) * gCadenceFilt;
    }
    // Send the cadence estimated from treadmill speed (one leg, steps per minute)
    data[3] = (uint8_t)roundf(gCadenceFilt/2.0f);

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
    if (flags & 0x0020) { gPace = readU8(); }
    // Bit 6: Average Pace
    if (flags & 0x0040) readU8();
    // Bit 7: Expended Energy (total + per hour + per minute)
    if (flags & 0x0080) { readU16(); readU16(); gEnergy = readU8(); }
    // Bit 8: Heart Rate
    if (flags & 0x0100) { gHeartrate = readU8(); }
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

    Serial.printf("[B%d][FTMS] spd=%.2f dst=%u t=%u hr=%u kcal=%u watch=%s\n",
        ESP32_BUILD,
        gRawSpeed * 0.01f,
        gDistanceM,
        gElapsedSec,
        gHeartrate,
        gEnergy,
        watchConnected ? "OK" : "waiting");
}


// =============================================================================
// GATT Server callbacks — watch (RSC sensor) connects/disconnects
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
        gRawSpeed = 0; gCadence = 0.0f; gCadenceFilt = 0.0f;
        sendRscMeasurement();
        
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
    if (pClient == nullptr) {
        Serial.printf("[B%d][FTMS] Unable to create client.\n", ESP32_BUILD);
        return false;
    }
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

    return true;
}


// =============================================================================
// GATT Server setup — Running, Speed and Cadence (RSC) sensor for Garmin
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

// =============================================================================
// Background interrupt service routine for vibration sensor (optional)
// Calculates cadence based on time between vibration pulses (one pulse per step)
// =============================================================================
void IRAM_ATTR onVibrationDetected() {
    unsigned long currentmillis = millis();

    portENTER_CRITICAL_ISR(&vibrationMux);
    unsigned long elapsed = currentmillis - lastStepTime;

    // Check on first pulse to initialize lastStepTime
    if (lastStepTime == 0) {
        lastStepTime = currentmillis;
    }
   // Filter out micro-vibrations/chatter from a single impact
    else if (elapsed > DEBOUNCE_TIME) {
        pulseTimePassed = elapsed;  // Update only for valid pulse
        lastStepTime = currentmillis;
    }

    portEXIT_CRITICAL_ISR(&vibrationMux);
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

    pinMode(VIBRATION_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(VIBRATION_PIN), onVibrationDetected, FALLING);

    NimBLEDevice::init("TreadBLE");

    // Start GATT server first — watch can connect anytime
    setupFtmsProxyServer();

    // Start scanning for the treadmill by MAC address
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), false);
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);

    Serial.printf("[B%d][SCAN] Scanning for treadmill and watch...\n",
        ESP32_BUILD);
    pScan->start(0, nullptr, false);
}

// =============================================================================
// loop() — main loop, handles deferred connection
// =============================================================================
void loop() {
static uint32_t lastRsc = 0;

    // First handle the LED state based on connection status. This gives immediate visual feedback.
    if (watchConnected && treadmillConnected) {
        // Solid LED when watch and treadmill are connected
        digitalWrite(LED_PIN, LED_ON);
    } else {
        if (treadmillConnected) {
            // Blink LED 1 Hz asynchronously when treadmill connected but watch not connected
            if (millis() % 1000 < 800) {
                digitalWrite(LED_PIN, LED_OFF);
            } else {
                digitalWrite(LED_PIN, LED_ON);
            }
        } else if (watchConnected) {
            // Blink LED 1 Hz asynchronously when treadmill connected but watch not connected
            if (millis() % 1000 < 200) {
                digitalWrite(LED_PIN, LED_OFF);
            } else {
                digitalWrite(LED_PIN, LED_ON);
            }
        } else {
            // Blink Led really fast when both treadmill and watch not connected
            if (millis() % 200 < 100) {
                digitalWrite(LED_PIN, LED_OFF);
            } else {
                digitalWrite(LED_PIN, LED_ON);
            }
        }
    }

    // Check for sensor timeouts to reset to 0 cadence if no vibration pulses are detected for a while
    portENTER_CRITICAL(&vibrationMux);
    if (lastStepTime > 0 && (millis() - lastStepTime > SENSOR_TIMEOUT)) {
        lastStepTime = 0;
        pulseTimePassed = 0;
        sensorCadence = 0.0f;
        gCadenceFilt = 0.0f;
    }
    portEXIT_CRITICAL(&vibrationMux);

    // Send RSC measurement to watch every second if connected
    if (millis() - lastRsc >= 1000) {
        lastRsc = millis();

        if (watchConnected) {
            sendRscMeasurement();

            Serial.printf(
                "[B%d][RSC] speed=%.2f km/h distance=%lu m, cadence=%u spm\n",
                ESP32_BUILD,
                gRawSpeed * 0.01f,
                (unsigned long)gDistanceM,
                (uint16_t)(roundf(gCadenceFilt))
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
    
    // Delay a bit to avoid busy looping and allow other tasks to run
    delay(10);
}
 
