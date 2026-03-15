/*************************************************
 * Water Tank Monitoring System
 * DEVICE  : ESP32-C3 (SENDER)
 * VERSION : 8.1
 *
 * CHANGES FROM V8.0:
 *  - Night sleep: 22:00-04:00 (was 21:00)
 *  - Sleep duration: 3 min (was 5 min)
 *  - Wake duration: 2 min (unchanged)
 *  - Auto-restart if no receiver response for 5 min
 *
 * FEATURES:
 *  - Single direct ultrasonic read per cycle (no filter)
 *  - Temperature-compensated sound speed
 *  - BME280: Temp, Humidity, Pressure
 *  - SH1106 OLED 4-screen rotation (5 sec each):
 *      Screen 0: Tank Data
 *      Screen 1: Weather
 *      Screen 2: System Status
 *      Screen 3: Comm + Sleep countdown
 *  - ESP-NOW full sync protocol
 *  - Deep sleep NIGHT ONLY 22:00-04:00
 *  - OTA, MQTT optional, Calibration
 *************************************************/

#define FIRMWARE_VERSION "8.1"

/* ================= OTA CONFIG ================= */
#define OTA_ENABLE       true
#define OTA_RUN_AT_BOOT  true
#define OTA_WINDOW_MS    15000
#define OTA_VERSION_URL  "https://raw.githubusercontent.com/yourabdulwahabmobile-oss/attendance-web-app/main/esp32-ota/WaterTank_Sender/version.txt"
#define OTA_FIRMWARE_URL "https://raw.githubusercontent.com/yourabdulwahabmobile-oss/attendance-web-app/main/esp32-ota/WaterTank_Sender/firmware.bin"

/* ================= WIFI ================= */
#define WIFI_SSID "Mirza Gee 2"
#define WIFI_PASS "password2026"

/* ================= NTP ================= */
#define NTP_SERVER          "pool.ntp.org"
#define GMT_OFFSET_SEC      18000
#define DAYLIGHT_OFFSET_SEC 0

/* ================= MQTT (optional) ================= */
#define MQTT_ENABLE         false
#define MQTT_BROKER         "broker.hivemq.com"
#define MQTT_PORT           1883
#define MQTT_TOPIC_TELEMETRY "waterTank/house1/sender/telemetry"
#define MQTT_TOPIC_STATUS    "waterTank/house1/sender/status"
#define MQTT_RETRY_MS        5000
#define WIFI_START_DELAY_MS  10000

/* ================= INCLUDES ================= */
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#if MQTT_ENABLE
#include <PubSubClient.h>
#endif

/* ================= PIN CONFIG ================= */
#define TRIG_PIN  5
#define ECHO_PIN  4
#define CAL_BTN   6
#define SDA_PIN   8
#define SCL_PIN   9

/* ================= OLED ================= */
#define SCREEN_W            128
#define SCREEN_H             64
#define OLED_RESET           -1
#define OLED_ADDR           0x3C
#define SCREEN_DISPLAY_TIME 5000UL   // 5 sec per OLED screen

/* ================= TANK CONFIG ================= */
const float TANK_CAPACITY_GAL = 200.0;
const float GAL_TO_LITER      =   3.78541;

/* ================= SENSOR CONFIG ================= */
const float WATER_HEIGHT_FULL_CM = 110.5;
const float SENSOR_MIN_CM        =  10.0;
const float SENSOR_MAX_CM        = 400.0;

/* ================= READ TIMING ================= */
#define READ_INTERVAL_MS  5000UL   // read sensors every 5 sec

/* ================= DEEP SLEEP — NIGHT ONLY ================= */
#define DEEP_SLEEP_ENABLE       true
#define NIGHT_SLEEP_START_HOUR  22   // 10:00 PM
#define NIGHT_SLEEP_END_HOUR     4   //  4:00 AM
#define WAKE_DURATION_NIGHT_MS  (2UL * 60 * 1000)   // 2 min awake
#define SLEEP_DURATION_NIGHT_MS (3UL * 60 * 1000)   // 3 min sleep
#define MOTOR_STATUS_TIMEOUT_MS  15000UL

/* ================= NO-COMM WATCHDOG ================= */
#define NO_COMM_RESTART_MS (5UL * 60 * 1000)   // restart if no comms for 5 min

/* ================= DRY FAULT ================= */
#define DRY_FAULT_CLEAR_TIME_MS    (15UL * 60 * 1000)
#define DRY_FAULT_MIN_SAFE_LEVEL    50.0
#define DRY_FAULT_MOTOR_BLOCK_LEVEL 40.0

/* ================= SLEEP PERMISSION ================= */
#define SLEEP_PERMISSION_TIMEOUT_MS 5000

/* ================= HEARTBEAT ================= */
#define HEARTBEAT_INTERVAL_MS 10000

/* ================= RTC MEMORY ================= */
RTC_DATA_ATTR bool     rtc_dryFaultActive   = false;
RTC_DATA_ATTR uint32_t rtc_dryFaultStartSec = 0;
RTC_DATA_ATTR uint32_t rtc_totalUptimeSec   = 0;
RTC_DATA_ATTR uint32_t rtc_bootCount        = 0;

/* ================= RECEIVER MAC ================= */
uint8_t receiverMAC[] = {0xEC, 0xE3, 0x34, 0x22, 0x35, 0xA8};

/* ================= MESSAGE TYPE IDs ================= */
#define MSG_TANK_DATA        0x10
#define MSG_MOTOR_STATUS     0x20
#define MSG_SLEEP_PERMISSION 0x40
#define MSG_HEARTBEAT        0x50
#define MSG_SENSOR_HEALTH    0x60
#define MSG_SLEEP_INFO       0xAA

/* ================= STRUCTS ================= */
typedef struct {
  uint8_t  msgType;
  float    distance_cm;
  float    level_percent;
  float    water_gallons;
  float    water_liters;
  float    temperature_c;
  float    humidity_percent;
  float    pressure_hpa;
  float    empty_dist_cm;
  float    full_dist_cm;
} TankData;

typedef struct {
  uint8_t  msgType;
  bool     motor_on;
  bool     dry_fault;
  unsigned long timestamp;
} MotorStatus;

typedef struct {
  uint8_t  msgType;
  bool     allow_sleep;
  uint32_t max_sleep_ms;
} SleepPermission;

typedef struct {
  uint8_t  msgType;
  uint32_t uptime_sec;
  float    battery_voltage;
  uint8_t  wifi_channel;
} Heartbeat;

typedef struct {
  uint8_t  msgType;
  uint16_t total_reads;
  uint16_t bad_reads;
  bool     sensor_fault;
  float    min_distance;
  float    max_distance;
} SensorHealth;

typedef struct {
  uint8_t  msgType;
  uint32_t sleepDurationMs;
  float    lastLevelPercent;
  char     sleepTime[12];
} SleepInfo;

/* ================= OBJECTS ================= */
Adafruit_BME280  bme;
Adafruit_SH1106G display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);
Preferences      prefs;
#if MQTT_ENABLE
WiFiClient   mqttNet;
PubSubClient mqttClient(mqttNet);
#endif

/* ================= CALIBRATION ================= */
float EMPTY_DISTANCE_CM = 137.5;
float FULL_DISTANCE_CM  =  27.5;

/* ================= STATE VARIABLES ================= */
TankData    sendData;
MotorStatus motorStatus;

bool          espnowSendSuccess       = false;
bool          displayInitialized      = false;
bool          timeInitialized         = false;
bool          firstReadingDone        = false;
bool          motorStatusReceived     = false;
bool          sleepPermissionReceived = false;
bool          sleepAllowed            = false;
uint32_t      receiverMaxSleepMs      = 0;

unsigned long lastMotorStatusTime  = 0;
unsigned long lastHeartbeatTime    = 0;
unsigned long wakeStartTime        = 0;
unsigned long lastReadMs           = 0;
unsigned long lastEspNowSendMs     = 0;
unsigned long lastUpdateMs         = 0;

uint8_t       currentScreen    = 0;
unsigned long lastScreenSwitch = 0;

uint16_t totalReadCount  = 0;
uint16_t badReadCount    = 0;
float    minDistanceSeen = 999.0;
float    maxDistanceSeen =   0.0;

float humidity = 0.0;
float pressure = 0.0;

#if MQTT_ENABLE
String   mqttClientId;
uint32_t lastMqttAttemptMs   = 0;
bool     wifiStarted         = false;
bool     mqttEverConnected   = false;
uint32_t wifiStartDeadlineMs = 0;
#endif

/* ================= ESP-NOW CALLBACKS ================= */
void onSend(const wifi_tx_info_t*, esp_now_send_status_t s) {
  espnowSendSuccess = (s == ESP_NOW_SEND_SUCCESS);
  if (espnowSendSuccess) lastEspNowSendMs = millis();
}

void onReceiveFromReceiver(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t type = data[0];
  if (type == MSG_MOTOR_STATUS && len >= (int)sizeof(MotorStatus)) {
    memcpy(&motorStatus, data, sizeof(MotorStatus));
    lastMotorStatusTime = millis();
    motorStatusReceived = true;
    Serial.printf("[RX] Motor:%s%s\n", motorStatus.motor_on ? "ON" : "OFF",
                  motorStatus.dry_fault ? " FAULT" : "");
  } else if (type == MSG_SLEEP_PERMISSION && len >= (int)sizeof(SleepPermission)) {
    SleepPermission perm;
    memcpy(&perm, data, sizeof(SleepPermission));
    sleepPermissionReceived = true;
    sleepAllowed            = perm.allow_sleep;
    receiverMaxSleepMs      = perm.max_sleep_ms;
    Serial.printf("[RX] Sleep:%s\n", sleepAllowed ? "ALLOWED" : "DENIED");
  }
}

/* ================= ULTRASONIC ================= */
float readDistanceCM(float tempC) {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long us = pulseIn(ECHO_PIN, HIGH, 30000);
  if (us == 0) return -1.0;
  float soundSpeed = 331.0 + (0.6 * tempC);
  return (us / 1000000.0) * soundSpeed * 100.0 / 2.0;
}

/* ================= TIME HELPERS ================= */
String getCurrentTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "--:--";
  int h = timeinfo.tm_hour;
  String ap = "AM";
  if (h == 0) h = 12; else if (h == 12) ap = "PM"; else if (h > 12) { h -= 12; ap = "PM"; }
  char buf[12]; sprintf(buf, "%d:%02d %s", h, timeinfo.tm_min, ap.c_str());
  return String(buf);
}

String getFullTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "--:--:--";
  int h = timeinfo.tm_hour;
  String ap = "AM";
  if (h == 0) h = 12; else if (h == 12) ap = "PM"; else if (h > 12) { h -= 12; ap = "PM"; }
  char buf[16]; sprintf(buf, "%d:%02d:%02d %s", h, timeinfo.tm_min, timeinfo.tm_sec, ap.c_str());
  return String(buf);
}

bool isNightMode() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return false;
  int h = timeinfo.tm_hour;
  return (h >= NIGHT_SLEEP_START_HOUR || h < NIGHT_SLEEP_END_HOUR);
}

/* ================= DRY FAULT ================= */
void checkDryFaultAutoClear() {
  if (motorStatus.dry_fault && !rtc_dryFaultActive) {
    rtc_dryFaultActive   = true;
    rtc_dryFaultStartSec = rtc_totalUptimeSec + (millis() / 1000);
  }
  if (rtc_dryFaultActive) {
    uint32_t elapsed = (rtc_totalUptimeSec + millis() / 1000) - rtc_dryFaultStartSec;
    if (!motorStatus.motor_on && sendData.level_percent >= DRY_FAULT_MIN_SAFE_LEVEL &&
        (elapsed >= DRY_FAULT_CLEAR_TIME_MS / 1000 || !motorStatus.dry_fault)) {
      rtc_dryFaultActive = false;
      Serial.printf("[FAULT] Cleared after %lu min\n", elapsed / 60);
    }
  }
}

/* ================= SLEEP HELPERS ================= */
bool canEnterDeepSleep() {
  if (!DEEP_SLEEP_ENABLE)                               return false;
  if (!isNightMode())                                   return false;
  if (motorStatus.motor_on)                             return false;
  if (!motorStatusReceived)                             return false;
  if (millis() - lastMotorStatusTime > MOTOR_STATUS_TIMEOUT_MS) return false;
  if (sendData.level_percent < 50.0)                    return false;
  if (rtc_dryFaultActive || motorStatus.dry_fault)      return false;
  return true;
}

void enterDeepSleep() {
  unsigned long sleepMs = SLEEP_DURATION_NIGHT_MS;
  if (sleepPermissionReceived && receiverMaxSleepMs > 0 && receiverMaxSleepMs < sleepMs)
    sleepMs = receiverMaxSleepMs;

  SleepInfo si;
  si.msgType = MSG_SLEEP_INFO; si.sleepDurationMs = sleepMs;
  si.lastLevelPercent = sendData.level_percent;
  String t = getCurrentTime(); strncpy(si.sleepTime, t.c_str(), 11); si.sleepTime[11] = '\0';
  esp_now_send(receiverMAC, (uint8_t*)&si, sizeof(si)); delay(100);

  Serial.printf("\n[SLEEP] Night sleep %.1f min @ %s\n", sleepMs / 60000.0, getCurrentTime().c_str());

  if (displayInitialized) {
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
    display.setCursor(18, 0);  display.println(">>> SLEEPING <<<");
    display.drawLine(0, 10, 127, 10, SH110X_WHITE);
    display.setCursor(0, 14); display.print("Sleep : "); display.print(sleepMs/60000); display.println(" min");
    display.setCursor(0, 24); display.print("Level : "); display.print(sendData.level_percent, 1); display.println("%");
    display.setCursor(0, 34); display.print("Time  : "); display.println(getCurrentTime());
    display.setCursor(0, 44); display.print("Boot# : "); display.println((int)rtc_bootCount);
    display.setCursor(0, 54); display.print("TX:"); display.print(espnowSendSuccess ? "OK" : "--");
    display.display(); delay(1500);
    display.clearDisplay(); display.display();
    display.oled_command(SH110X_DISPLAYOFF);
  }
  rtc_totalUptimeSec += millis() / 1000;
  esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
  esp_deep_sleep_start();
}

/* ================= HEARTBEAT / SENSOR HEALTH ================= */
void sendHeartbeat() {
  Heartbeat hb; hb.msgType = MSG_HEARTBEAT;
  hb.uptime_sec = rtc_totalUptimeSec + (millis() / 1000);
  hb.battery_voltage = 0.0; hb.wifi_channel = 1;
  esp_now_send(receiverMAC, (uint8_t*)&hb, sizeof(hb));
  Serial.printf("[HB] Uptime:%lu sec\n", hb.uptime_sec);
}

void sendSensorHealth() {
  SensorHealth h; h.msgType = MSG_SENSOR_HEALTH;
  h.total_reads = totalReadCount; h.bad_reads = badReadCount;
  h.sensor_fault = (badReadCount > totalReadCount / 2 && totalReadCount > 5);
  h.min_distance = (minDistanceSeen < 900.0) ? minDistanceSeen : 0.0;
  h.max_distance = maxDistanceSeen;
  esp_now_send(receiverMAC, (uint8_t*)&h, sizeof(h));
  totalReadCount = 0; badReadCount = 0; minDistanceSeen = 999.0; maxDistanceSeen = 0.0;
}

void requestSleepPermission() {
  sendHeartbeat(); sleepPermissionReceived = false;
}

/* ================= OLED INIT ================= */
void initDisplay() {
  if (displayInitialized) return;
  delay(250);
  if (display.begin(OLED_ADDR, true)) {
    displayInitialized = true;
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
    display.setCursor(14, 0); display.println("WATER TANK v8.1");
    display.drawLine(0, 10, 127, 10, SH110X_WHITE);
    display.setCursor(0, 20); display.println("Initializing...");
    display.display(); delay(1500);
  }
}

/* ================= OLED SCREENS ================= */
// Screen 0: TANK DATA
void displayTankData() {
  if (!displayInitialized) return;
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(28, 0); display.println("[ TANK DATA ]");
  display.drawLine(0, 9, 127, 9, SH110X_WHITE);
  // Large level
  display.setTextSize(2);
  String lvl = String(sendData.level_percent, 1) + "%";
  int lx = max(0, (128 - (int)(lvl.length() * 12)) / 2);
  display.setCursor(lx, 13); display.print(lvl);
  // Details
  display.setTextSize(1);
  display.setCursor(0, 33); display.print("Dist: "); display.print(sendData.distance_cm, 1); display.println("cm");
  display.setCursor(0, 43); display.print("Vol : "); display.print(sendData.water_gallons, 1); display.print("gal "); display.print(sendData.water_liters, 0); display.println("L");
  display.setCursor(0, 54);
  if (motorStatus.dry_fault || rtc_dryFaultActive) { display.print("Motor:DRY FAULT!"); }
  else { display.print("Motor:"); display.print(motorStatus.motor_on ? "ON " : "OFF"); display.print(" TX:"); display.print(espnowSendSuccess ? "OK" : "--"); }
  display.display();
}

// Screen 1: WEATHER
void displayWeatherData() {
  if (!displayInitialized) return;
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(22, 0); display.println("[ WEATHER ]");
  display.drawLine(0, 9, 127, 9, SH110X_WHITE);
  display.setCursor(0, 13); display.print("Temp    : "); display.print(sendData.temperature_c, 1); display.println(" C");
  display.setCursor(0, 23); display.print("Humidity: "); display.print(humidity, 1); display.println(" %");
  display.setCursor(0, 33); display.print("Pressure: "); display.print(pressure, 0); display.println("hPa");
  display.setCursor(0, 43); display.print("Time : "); display.println(getCurrentTime());
  display.setCursor(0, 54); display.print("Boot#"); display.print((int)rtc_bootCount); display.print(" "); display.print(isNightMode() ? "NIGHT" : "DAY");
  display.display();
}

// Screen 2: SYSTEM STATUS
void displaySystemStatus() {
  if (!displayInitialized) return;
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(16, 0); display.println("[ SYS STATUS ]");
  display.drawLine(0, 9, 127, 9, SH110X_WHITE);
  display.setCursor(0, 13); display.print("Level : "); display.print(sendData.level_percent, 1); display.println("%");
  display.setCursor(0, 23); display.print("T:"); display.print(sendData.temperature_c, 1); display.print("C H:"); display.print(humidity, 0); display.println("%");
  display.setCursor(0, 33);
  if (lastUpdateMs > 0) {
    unsigned long ago = (millis() - lastUpdateMs) / 1000;
    display.print("Update: "); display.print(ago < 60 ? ago : ago / 60); display.println(ago < 60 ? "s ago" : "m ago");
  } else { display.println("Update: waiting"); }
  display.setCursor(0, 43);
  if (motorStatus.dry_fault || rtc_dryFaultActive) display.println("Motor: DRY FAULT!");
  else { display.print("Motor: "); display.println(motorStatus.motor_on ? "ON" : "OFF"); }
  display.setCursor(0, 54);
  if (espnowSendSuccess) {
    unsigned long ago = lastEspNowSendMs > 0 ? (millis() - lastEspNowSendMs) / 1000 : 0;
    display.print("ESP-NOW OK "); display.print(ago); display.print("s");
  } else { display.print("ESP-NOW: FAIL"); }
  display.display();
}

// Screen 3: COMM + SLEEP COUNTDOWN
void displayCommAndSleep() {
  if (!displayInitialized) return;
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(16, 0); display.println("[ COMM+SLEEP ]");
  display.drawLine(0, 9, 127, 9, SH110X_WHITE);
  display.setCursor(0, 13); display.println(espnowSendSuccess ? "ESP-NOW: OK" : "ESP-NOW: FAIL");
  display.setCursor(0, 23);
  if (lastEspNowSendMs > 0) {
    display.print("Last pkt: "); display.print((millis() - lastEspNowSendMs) / 1000); display.println("s ago");
  } else { display.println("Last pkt: --"); }
  display.setCursor(0, 33);
  if (motorStatus.dry_fault || rtc_dryFaultActive) display.println("Motor: Dry fault");
  else if (motorStatus.motor_on) display.println("Motor: ON");
  else display.println("Motor: OFF");
  // Sleep countdown
  display.setCursor(0, 43);
  if (!isNightMode()) {
    display.println("Sleep: DAY - OFF");
    display.setCursor(0, 54); display.print("Mode: DAY (no sleep)");
  } else if (firstReadingDone) {
    unsigned long elapsed = millis() - wakeStartTime;
    if (elapsed < WAKE_DURATION_NIGHT_MS) {
      unsigned long remSec = (WAKE_DURATION_NIGHT_MS - elapsed) / 1000;
      display.print("Sleep in: "); display.print(remSec); display.println("s");
      display.setCursor(0, 54); display.print(canEnterDeepSleep() ? "(ready)" : "(blocked)");
    } else {
      display.println("Entering sleep...");
      display.setCursor(0, 54); display.print(canEnterDeepSleep() ? "Requesting..." : "Blocked");
    }
  } else {
    display.println("Waiting first read");
  }
  display.display();
}

// Rotate screens
void updateDisplay() {
  if (!displayInitialized) return;
  unsigned long now = millis();
  if (now - lastScreenSwitch >= SCREEN_DISPLAY_TIME) {
    currentScreen    = (currentScreen + 1) % 4;
    lastScreenSwitch = now;
  }
  switch (currentScreen) {
    case 0: displayTankData();     break;
    case 1: displayWeatherData();  break;
    case 2: displaySystemStatus(); break;
    case 3: displayCommAndSleep(); break;
  }
}

/* ================= OTA ================= */
bool connectWiFi(uint32_t ms = OTA_WINDOW_MS) {
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t s = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - s < ms) { delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) { Serial.printf("[OTA] IP:%s\n", WiFi.localIP().toString().c_str()); return true; }
  return false;
}

String httpGetText(const char* url) {
  WiFiClientSecure c; c.setInsecure(); HTTPClient h;
  if (!h.begin(c, url)) return "";
  if (h.GET() != HTTP_CODE_OK) { h.end(); return ""; }
  String p = h.getString(); h.end(); p.trim(); return p;
}

void otaUpdate(const char* url) {
  WiFiClientSecure c; c.setInsecure(); HTTPClient h;
  if (!h.begin(c, url)) return;
  if (h.GET() != HTTP_CODE_OK) { h.end(); return; }
  int len = h.getSize();
  if (!Update.begin(len)) { h.end(); return; }
  WiFiClient* s = h.getStreamPtr();
  size_t written = 0; uint8_t buf[128]; int lastPct = -1;
  while (h.connected() && written < (size_t)len) {
    size_t av = s->available();
    if (av) { int c2 = s->readBytes(buf, min(av, sizeof(buf))); Update.write(buf, c2); written += c2;
              int pct = (written * 100) / len;
              if (pct != lastPct && pct % 10 == 0) { Serial.printf("[OTA] %d%%\n", pct); lastPct = pct; } }
    delay(1);
  }
  h.end();
  if (Update.end() && Update.isFinished()) { Serial.println("[OTA] Done"); delay(500); ESP.restart(); }
}

void checkOTA() {
  if (!OTA_ENABLE || !OTA_RUN_AT_BOOT) return;
  if (displayInitialized) { display.clearDisplay(); display.setCursor(10, 20); display.println("Checking OTA..."); display.display(); }
  if (!connectWiFi()) return;
  if (!timeInitialized) {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    struct tm ti; int tries = 0;
    while (!getLocalTime(&ti) && tries++ < 10) delay(500);
    if (tries <= 10) { timeInitialized = true; Serial.printf("[NTP] %s\n", getFullTime().c_str()); }
  }
  String sv = httpGetText(OTA_VERSION_URL);
  if (sv.length() == 0) return;
  Serial.printf("[OTA] Local:%s Server:%s\n", FIRMWARE_VERSION, sv.c_str());
  if (sv.toFloat() > String(FIRMWARE_VERSION).toFloat()) otaUpdate(OTA_FIRMWARE_URL);
}

/* ================= MQTT ================= */
#if MQTT_ENABLE
void mqttSetupClientId() { String mac = WiFi.macAddress(); mac.replace(":", ""); mqttClientId = "wt_sender_" + mac; }
void mqttConfigure() { mqttClient.setServer(MQTT_BROKER, MQTT_PORT); mqttClient.setKeepAlive(30); }
void mqttEnsureConnected() {
  if (!wifiStarted || WiFi.status() != WL_CONNECTED || mqttClient.connected()) return;
  uint32_t now = millis(); if (now - lastMqttAttemptMs < MQTT_RETRY_MS) return;
  lastMqttAttemptMs = now; if (mqttClientId.length() == 0) mqttSetupClientId();
  if (mqttClient.connect(mqttClientId.c_str())) { mqttEverConnected = true; }
}
void mqttPublishTelemetry(TankData &d) {
  if (!wifiStarted || WiFi.status() != WL_CONNECTED || !mqttClient.connected()) return;
  char p[320]; snprintf(p, sizeof(p), "{\"fw\":\"%s\",\"dist\":%.1f,\"level\":%.1f,\"gal\":%.1f,\"L\":%.1f,\"temp\":%.1f,\"hum\":%.1f,\"pres\":%.0f}",
           FIRMWARE_VERSION, d.distance_cm, d.level_percent, d.water_gallons, d.water_liters, d.temperature_c, d.humidity_percent, d.pressure_hpa);
  mqttClient.publish(MQTT_TOPIC_TELEMETRY, p, false);
}
#endif

/* ================= CALIBRATION ================= */
void runCalibration() {
  Serial.println("\n CALIBRATION MODE"); delay(2000);
  while (digitalRead(CAL_BTN) == LOW) delay(100);
  Serial.println("1. EMPTY tank — press button");
  while (digitalRead(CAL_BTN) == HIGH) delay(100); delay(500);
  float t = bme.readTemperature();
  EMPTY_DISTANCE_CM = readDistanceCM(t);
  if (EMPTY_DISTANCE_CM < 0) EMPTY_DISTANCE_CM = 137.5;
  Serial.printf("   Empty: %.1f cm\n", EMPTY_DISTANCE_CM);
  while (digitalRead(CAL_BTN) == LOW) delay(100); delay(1000);
  Serial.println("2. FULL tank — press button");
  while (digitalRead(CAL_BTN) == HIGH) delay(100); delay(500);
  t = bme.readTemperature();
  FULL_DISTANCE_CM = readDistanceCM(t);
  if (FULL_DISTANCE_CM < 0) FULL_DISTANCE_CM = 27.5;
  Serial.printf("   Full: %.1f cm\n", FULL_DISTANCE_CM);
  prefs.putFloat("empty", EMPTY_DISTANCE_CM); prefs.putFloat("full", FULL_DISTANCE_CM);
  Serial.printf("Saved: Empty=%.1f Full=%.1f\n", EMPTY_DISTANCE_CM, FULL_DISTANCE_CM);
  while (digitalRead(CAL_BTN) == LOW) delay(100); delay(2000);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200); delay(3000);
  rtc_bootCount++;
  Serial.printf("\n=== SENDER v8.1 Boot#%lu Uptime:%lu sec ===\n", (unsigned long)rtc_bootCount, (unsigned long)rtc_totalUptimeSec);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT); pinMode(CAL_BTN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
  if (bme.begin(0x76)) Serial.println("[BME280] OK"); else Serial.println("[BME280] NOT FOUND");
  initDisplay();
  if (rtc_bootCount > 1 && displayInitialized) {
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
    display.setCursor(16, 0); display.println("*** WAKING UP ***");
    display.drawLine(0, 10, 127, 10, SH110X_WHITE);
    display.setCursor(0, 14); display.print("Boot#: ");  display.println((int)rtc_bootCount);
    display.setCursor(0, 25); display.print("Uptime:"); display.print(rtc_totalUptimeSec / 60); display.println("min");
    display.setCursor(0, 36); display.print("Fault: ");  display.println(rtc_dryFaultActive ? "ACTIVE!" : "clear");
    display.setCursor(0, 47); display.print("Mode : ");  display.println(isNightMode() ? "NIGHT" : "DAY");
    display.display(); delay(2000);
  }
  prefs.begin("tank", false);
  EMPTY_DISTANCE_CM = prefs.getFloat("empty", EMPTY_DISTANCE_CM);
  FULL_DISTANCE_CM  = prefs.getFloat("full",  FULL_DISTANCE_CM);
  if (digitalRead(CAL_BTN) == LOW) runCalibration();
  checkOTA();
  WiFi.disconnect(true); delay(500); WiFi.mode(WIFI_STA); delay(500);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (esp_now_init() == ESP_OK) Serial.println("[ESP-NOW] OK");
  else { Serial.println("[ESP-NOW] FAILED"); while(1) delay(1000); }
  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceiveFromReceiver);
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, receiverMAC, 6); peer.channel = 1; peer.encrypt = false; peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);
#if MQTT_ENABLE
  mqttConfigure(); wifiStartDeadlineMs = millis() + WIFI_START_DELAY_MS;
#endif
  wakeStartTime    = millis();
  lastScreenSwitch = millis();
  lastReadMs       = 0;
  Serial.printf("[SLEEP] Night window: %02d:00-%02d:00  Wake:%lumin Sleep:%lumin\n",
                NIGHT_SLEEP_START_HOUR, NIGHT_SLEEP_END_HOUR,
                WAKE_DURATION_NIGHT_MS/60000, SLEEP_DURATION_NIGHT_MS/60000);
}

/* ================= LOOP ================= */
void loop() {
  unsigned long now = millis();

#if MQTT_ENABLE
  if (!wifiStarted && (firstReadingDone || (int32_t)(millis() - wifiStartDeadlineMs) >= 0)) {
    wifiStarted = true; WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  if (wifiStarted) { mqttEnsureConnected(); mqttClient.loop(); }
#endif

  // ── SENSOR READ every 5 sec ───────────────────────────────────────────────
  if (now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;
    float tempC = bme.readTemperature();
    humidity    = bme.readHumidity();
    pressure    = bme.readPressure() / 100.0F;
    float distance = readDistanceCM(tempC);
    totalReadCount++;
    bool ok = (distance >= SENSOR_MIN_CM && distance <= SENSOR_MAX_CM);
    if (!ok) {
      badReadCount++;
      Serial.printf("[SENSOR] Bad: %.1f cm\n", distance);
      sendData.temperature_c = tempC; sendData.humidity_percent = humidity; sendData.pressure_hpa = pressure;
    } else {
      if (distance < minDistanceSeen) minDistanceSeen = distance;
      if (distance > maxDistanceSeen) maxDistanceSeen = distance;
      float wh      = constrain(EMPTY_DISTANCE_CM - distance, 0.0, WATER_HEIGHT_FULL_CM);
      float percent = constrain((wh / WATER_HEIGHT_FULL_CM) * 100.0, 0.0, 100.0);
      float gal     = (percent / 100.0) * TANK_CAPACITY_GAL;
      float lit     = gal * GAL_TO_LITER;
      sendData.msgType = MSG_TANK_DATA; sendData.distance_cm = distance;
      sendData.level_percent = percent; sendData.water_gallons = gal; sendData.water_liters = lit;
      sendData.temperature_c = tempC; sendData.humidity_percent = humidity; sendData.pressure_hpa = pressure;
      sendData.empty_dist_cm = EMPTY_DISTANCE_CM; sendData.full_dist_cm = FULL_DISTANCE_CM;
      esp_now_send(receiverMAC, (uint8_t*)&sendData, sizeof(sendData)); delay(20);
      esp_now_send(receiverMAC, (uint8_t*)&sendData, sizeof(sendData)); delay(50);
      lastUpdateMs = millis(); firstReadingDone = true;
      Serial.printf("[DATA] Dist:%.1fcm Level:%.1f%% Vol:%.1fgal T:%.1fC TX:%s\n",
                    distance, percent, gal, tempC, espnowSendSuccess ? "OK" : "FAIL");
    }
    if (totalReadCount >= 10) sendSensorHealth();
  }

  // ── DISPLAY ───────────────────────────────────────────────────────────────
  updateDisplay();

  // ── HEARTBEAT every 10 sec ────────────────────────────────────────────────
  if (firstReadingDone && (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS)) {
    lastHeartbeatTime = millis(); sendHeartbeat();
  }

  // ── DRY FAULT ─────────────────────────────────────────────────────────────
  if (firstReadingDone) checkDryFaultAutoClear();

  // ── NO-COMM WATCHDOG: restart after 5 min no receiver response ────────────
  if (firstReadingDone && lastUpdateMs > 0) {
    bool noComm = (!motorStatusReceived && (millis() - lastUpdateMs > NO_COMM_RESTART_MS)) ||
                  (motorStatusReceived  && (millis() - lastMotorStatusTime > NO_COMM_RESTART_MS));
    if (noComm) {
      Serial.println("[WATCHDOG] No receiver response 5min → Restarting");
      delay(500); ESP.restart();
    }
  }

  // ── DEEP SLEEP (NIGHT ONLY) ───────────────────────────────────────────────
  if (DEEP_SLEEP_ENABLE && firstReadingDone) {
    if (isNightMode()) {
      if (millis() - wakeStartTime >= WAKE_DURATION_NIGHT_MS) {
        if (!canEnterDeepSleep()) { wakeStartTime = millis(); return; }
        if (!sleepPermissionReceived) {
          requestSleepPermission();
          unsigned long ws = millis();
          while (!sleepPermissionReceived && millis() - ws < SLEEP_PERMISSION_TIMEOUT_MS) delay(100);
        }
        if (sleepPermissionReceived && sleepAllowed)   { enterDeepSleep(); }
        else if (sleepPermissionReceived && !sleepAllowed) { wakeStartTime = millis(); sleepPermissionReceived = false; }
        else                                           { enterDeepSleep(); }
      }
    } else {
      // Day mode — keep resetting wake timer so sleep triggers immediately at night
      wakeStartTime = millis(); sleepPermissionReceived = false;
    }
  }
}
