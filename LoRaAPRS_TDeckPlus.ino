/*
 * LoRa APRS for LILYGO T-Deck Plus (ESP32-S3 + SX1262 + MIA-M10Q GPS)
 * Arduino IDE sketch. TNC2 APRS text is sent in the RadioLib APRS-over-LoRa
 * envelope. This is not AX.25/AFSK APRS compatibility.
 */
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Preferences.h>
#include "tft_setup.h"
#include <TFT_eSPI.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP_I2S.h>
#include "web_page.h"

// T-Deck Plus pins, from the current LILYGO board documentation.
#define PIN_POWER 10
#define PIN_LORA_CS 9
#define PIN_LORA_DIO1 45
#define PIN_LORA_RST 17
#define PIN_LORA_BUSY 13
#define PIN_SPI_SCK 40
#define PIN_SPI_MISO 38
#define PIN_SPI_MOSI 41
#define PIN_TFT_CS 12
#define PIN_SD_CS 39
#define PIN_BATTERY_ADC 4
#define PIN_TFT_DC 11
#define PIN_TFT_BL 42
#define PIN_I2C_SDA 18
#define PIN_I2C_SCL 8
#define PIN_KB_INT 46
#define PIN_TRACK_UP 3
#define PIN_TRACK_RIGHT 2
#define PIN_TRACK_DOWN 15
#define PIN_TRACK_LEFT 1
#define PIN_TRACK_CLICK 0
#define PIN_I2S_BCK 7
#define PIN_I2S_WS 5
#define PIN_I2S_DOUT 6
#define KEYBOARD_ADDR 0x55

// Set these for your local band plan and matching LoRa APRS peers.
// Frequency, bandwidth, spreading factor, coding rate, sync word, TX power.
static const float LORA_MHZ = 433.775;
static const float LORA_BW_KHZ = 125.0;
static const uint8_t LORA_SF = 12;
static const uint8_t LORA_CR = 5;  // RadioLib convention: 5 means 4/5
static const uint8_t LORA_SYNC_WORD = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
static const int8_t LORA_TX_DBM = 10;

static const char DEFAULT_CALLSIGN[] = "N0CALL-7"; // Editable in the settings page
static const char APRS_PATH[] = "WIDE1-1";
static const uint32_t BEACON_INTERVAL_MS = 180000;
static const uint32_t DISPLAY_IDLE_TIMEOUT_MS = 60000;
static const uint32_t GPS_STALE_MS = 120000;
static const uint32_t SMART_BEACON_MIN_GAP_MS = 30000;
static const uint8_t FIRST_SYMBOL_PRESET = 12;
static const uint8_t WIFI_MODE_SETTING = 19;
static const uint8_t MAIN_SETTING_ITEMS[] = {0, 1, 2, 3, 4, 22, 23, 19, 20, 21};
static const uint8_t MAIN_SETTING_COUNT = sizeof(MAIN_SETTING_ITEMS) / sizeof(MAIN_SETTING_ITEMS[0]);
static const uint8_t DETAIL_SETTING_COUNT = 17; // SmartBeacon, symbols, position options

SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
TFT_eSPI *display = new TFT_eSPI();
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Preferences prefs;
WebServer webServer(80);
bool webRoutesConfigured = false;
bool webServerStarted = false;
enum WifiProfileMode : uint8_t { WLAN_OFF = 0, WLAN_AP = 1, WLAN_CLIENT = 2 };
uint8_t wifiMode = WLAN_OFF;
char wifiSsid[33] = "TDeck-AP";
char wifiPassword[64] = "";
int lastWifiStatus = -1;
char myCallsign[10];
char stationComment[48];
char stationStatus[48];
char stationSymbol = '>';
volatile bool packetReady = false;
uint32_t lastBeacon = 0;
uint32_t positionBeaconCount = 0;
uint32_t rxCount = 0;
char inputLine[180];
size_t inputLen = 0;

struct Station { char call[16]; double lat, lon; bool hasPos; uint32_t heardAt; } stations[16];
uint8_t stationCount = 0;
struct __attribute__((packed)) StoredStation {
  uint32_t marker;
  char call[16];
  double lat, lon;
  uint8_t hasPos;
};
static const uint32_t STATION_MARKER = 0x41505253; // "APRS"
static const char STATION_LOG_PATH[] = "/aprs_stations.bin";
bool smartBeaconEnabled = true;
uint16_t smartSlowRateMin = 10;
uint16_t smartFastRateSec = 60;
uint16_t smartLowSpeedKmh = 5;
uint16_t smartHighSpeedKmh = 60;
uint16_t smartTurnAngleDeg = 25;
uint16_t smartTurnSlope = 255;
uint16_t smartTurnTimeSec = 30;
bool beaconSpeedEnabled = false;
bool beaconAltitudeEnabled = false;
uint8_t positionPrecision = 0; // 0 exact, 1..4 APRS ambiguity levels
float lastBeaconCourseDeg = 0.0f;
bool lastBeaconCourseValid = false;

struct AprsMessage { char from[16]; char text[100]; bool outgoing; bool acked; } messages[12];
uint8_t messageCount = 0;
bool sdReady = false;
static const char MESSAGE_LOG_PATH[] = "/aprs_rx.bin";
struct __attribute__((packed)) StoredRxMessage {
  uint32_t marker;
  char from[16];
  char text[100];
};
static const uint32_t RX_MESSAGE_MARKER = 0x41505258; // "APRX"
bool messageBeepActive = false;
bool audioReady = false;
I2SClass audioI2S(I2S_NUM_0);
bool messageToneEnabled = true;
bool batteryStatusEnabled = false;
bool batteryReadingValid = false;
float batteryVoltage = 0.0f;
uint8_t batteryPercent = 0;
uint32_t lastBatteryRead = 0;
uint32_t activeCpuFrequencyMhz = 160;
uint32_t messageBeepStartedAt = 0;
uint32_t messageBeepSample = 0;
uint8_t currentPage = 0;
uint8_t stationSelection = 0, messageSelection = 0, settingsSelection = 0;
uint8_t settingsMenuSelection = 0, detailsSelection = 0;
bool uiDirty = true;
bool displayAwake = true;
bool displayInitialized = false;
uint32_t lastDisplayActivity = 0;
bool editMode = false;
char editBuffer[100];
uint8_t editLimit = 0;
uint8_t composeStage = 0; // 0 idle, 1 destination, 2 message text
char composeTo[10];
char composeText[120];
uint16_t nextMessageId = 1;
volatile int16_t trackX = 0, trackY = 0;
volatile uint32_t trackClickFlag = 0;
int16_t handledTrackX = 0, handledTrackY = 0;
uint32_t handledClick = 0;
uint32_t lastTrackEvent = 0;
uint32_t lastFooterUpdate = 0;
uint32_t lastPageChange = 0;
uint32_t lastKeyboardActivity = 0;
bool keyboardBacklightOn = false;
char uiNotice[32] = "";
uint32_t uiNoticeUntil = 0;

enum { C_BG = 0x0861, C_PANEL = 0x18E3, C_WHITE = 0xFFFF, C_MUTED = 0x9CF3,
       C_CYAN = 0x07FF, C_GREEN = 0x07E0, C_YELLOW = 0xFFE0, C_RED = 0xF800 };

void onDio1() { packetReady = true; }
void maybeAckMessage(const char *packet, const char *info);
void drawUi();
void commitEdit();
void showUiNotice(const char *message);
void startMessageBeep();
void serviceMessageBeep();
void wakeDisplay();
void updateBatteryReading();
void drawBatteryIndicator();
void applyBatteryCpuFrequency();

void IRAM_ATTR onTrackUp() { ++trackY; }
void IRAM_ATTR onTrackRight() { ++trackX; }
void IRAM_ATTR onTrackDown() { --trackY; }
void IRAM_ATTR onTrackLeft() { --trackX; }
void IRAM_ATTR onTrackClick() { ++trackClickFlag; }

double parseCoord(const char *s, bool latitude) {
  // APRS uncompressed DDMM.mmN / DDDMM.mmE. Ambiguous trailing minute
  // digits are interpreted at the center of their reported area.
  int degDigits = latitude ? 2 : 3;
  int deg = 0;
  for (int i = 0; i < degDigits; ++i) deg = deg * 10 + (s[i] - '0');
  int minuteDigits[4];
  const int positions[] = {degDigits, degDigits + 1, degDigits + 3, degDigits + 4};
  bool ambiguityStarted = false;
  for (int i = 0; i < 4; ++i) {
    char digit = s[positions[i]];
    if (digit == ' ') ambiguityStarted = true;
    minuteDigits[i] = digit == ' ' ? 0 : digit - '0';
  }
  // Replace the first ambiguous digit with 5 (the cell midpoint) and all
  // following digits with 0, preserving any known leading digits.
  ambiguityStarted = false;
  for (int i = 0; i < 4; ++i) {
    if (s[positions[i]] == ' ') ambiguityStarted = true;
    if (ambiguityStarted) minuteDigits[i] = (s[positions[i]] == ' ' &&
                                              (i == 0 || s[positions[i - 1]] != ' ')) ? 5 : 0;
  }
  double minutes = minuteDigits[0] * 10 + minuteDigits[1] +
                   minuteDigits[2] / 10.0 + minuteDigits[3] / 100.0;
  double result = deg + minutes / 60.0;
  char hemi = s[degDigits + 5];
  if (hemi == 'S' || hemi == 'W') result = -result;
  return result;
}

bool extractPosition(const char *info, double &lat, double &lon) {
  // Supports APRS uncompressed position reports beginning with ! or =.
  if (info[0] != '!' && info[0] != '=') return false;
  const char *p = info + 1;
  if (strlen(p) < 18) return false;
  for (int i = 0; i < 4; ++i) if (p[i] < '0' || p[i] > '9') return false;
  if (p[4] != '.' || (p[5] != ' ' && (p[5] < '0' || p[5] > '9')) ||
      (p[6] != ' ' && (p[6] < '0' || p[6] > '9')) ||
      (p[7] != 'N' && p[7] != 'S')) return false;
  for (int i = 9; i <= 13; ++i)
    if (p[i] != ' ' && (p[i] < '0' || p[i] > '9')) return false;
  if (p[14] != '.' || (p[15] != ' ' && (p[15] < '0' || p[15] > '9')) ||
      (p[16] != ' ' && (p[16] < '0' || p[16] > '9'))) return false;
  // APRS ambiguity is contiguous from least-significant to more-significant
  // minute digits. Reject malformed fields with a digit after a blank.
  const int latDigits[] = {6, 5, 3, 2};
  bool blankSeen = false;
  for (int i = 0; i < 4; ++i) {
    char c = p[latDigits[i]];
    if (c == ' ') blankSeen = true;
    else if (blankSeen) return false;
  }
  const int lonDigits[] = {16, 15, 13, 12};
  blankSeen = false;
  for (int i = 0; i < 4; ++i) {
    char c = p[lonDigits[i]];
    if (c == ' ') blankSeen = true;
    else if (blankSeen) return false;
  }
  if (p[17] != 'E' && p[17] != 'W') return false;
  char a[9], b[10];
  memcpy(a, p, 8); a[8] = 0;
  memcpy(b, p + 9, 9); b[9] = 0;
  lat = parseCoord(a, true); lon = parseCoord(b, false);
  return fabs(lat) <= 90 && fabs(lon) <= 180;
}

int findStation(const char *call) {
  for (int i = 0; i < stationCount; ++i) if (!strcmp(stations[i].call, call)) return i;
  if (stationCount >= 16) { memmove(stations, stations + 1, sizeof(stations[0]) * 15); --stationCount; }
  int n = stationCount++;
  strncpy(stations[n].call, call, sizeof(stations[n].call) - 1);
  stations[n].call[sizeof(stations[n].call) - 1] = 0;
  stations[n].hasPos = false;
  stations[n].heardAt = millis();
  return n;
}

void cleanSource(const char *packet, char *out, size_t cap) {
  out[0] = 0;
  const char *gt = strchr(packet, '>');
  if (!gt) return;
  const char *end = gt, *start = gt;
  while (start > packet && (isalnum((unsigned char)start[-1]) || start[-1] == '-')) --start;
  if (end - start >= 3 && (size_t)(end - start) < cap) {
    memcpy(out, start, end - start); out[end - start] = 0;
  }
}

void storeMessage(const char *from, const char *text, bool outgoing) {
  if (messageCount >= 12) { memmove(messages, messages + 1, sizeof(messages[0]) * 11); --messageCount; }
  AprsMessage &m = messages[messageCount++];
  strncpy(m.from, from, sizeof(m.from) - 1); m.from[sizeof(m.from) - 1] = 0;
  strncpy(m.text, text, sizeof(m.text) - 1); m.text[sizeof(m.text) - 1] = 0;
  m.outgoing = outgoing; m.acked = false;
  messageSelection = messageCount - 1;
  uiDirty = true;
}

void persistReceivedMessage(const char *from, const char *text) {
  if (!sdReady) return;
  StoredRxMessage record = {};
  record.marker = RX_MESSAGE_MARKER;
  strncpy(record.from, from, sizeof(record.from) - 1);
  strncpy(record.text, text, sizeof(record.text) - 1);
  File file = SD.open(MESSAGE_LOG_PATH, FILE_APPEND);
  if (!file) {
    Serial.println("SD: Nachrichtenprotokoll kann nicht geoeffnet werden");
    return;
  }
  size_t written = file.write((const uint8_t *)&record, sizeof(record));
  file.flush();
  file.close();
  if (written != sizeof(record)) Serial.println("SD: Nachricht konnte nicht vollstaendig gespeichert werden");
}

void loadReceivedMessages() {
  if (!sdReady) return;
  File file = SD.open(MESSAGE_LOG_PATH, FILE_READ);
  if (!file) return; // First boot or no messages yet.
  const size_t recordSize = sizeof(StoredRxMessage);
  size_t total = file.size();
  size_t start = total > recordSize * 12 ? total - recordSize * 12 : 0;
  // Align to a record boundary so a partially written final record is ignored.
  start = (start / recordSize) * recordSize;
  file.seek(start);
  while (file.available() >= (int)recordSize) {
    StoredRxMessage record;
    if (file.read((uint8_t *)&record, recordSize) != recordSize) break;
    if (record.marker != RX_MESSAGE_MARKER || !memchr(record.from, 0, sizeof(record.from)) ||
        !memchr(record.text, 0, sizeof(record.text))) continue;
    storeMessage(record.from, record.text, false);
  }
  file.close();
  if (messageCount) {
    messageSelection = messageCount - 1;
    Serial.printf("SD: %u empfangene Nachrichten wiederhergestellt\n", messageCount);
  }
}

bool rewriteReceivedMessageLog() {
  if (!sdReady) return false;
  static const char tempPath[] = "/aprs_rx.tmp";
  static const char backupPath[] = "/aprs_rx.bak";
  SD.remove(tempPath);
  File file = SD.open(tempPath, FILE_WRITE);
  if (!file) return false;
  bool ok = true;
  for (int i = 0; i < messageCount; ++i) {
    if (messages[i].outgoing) continue;
    StoredRxMessage record = {};
    record.marker = RX_MESSAGE_MARKER;
    strncpy(record.from, messages[i].from, sizeof(record.from) - 1);
    strncpy(record.text, messages[i].text, sizeof(record.text) - 1);
    if (file.write((const uint8_t *)&record, sizeof(record)) != sizeof(record)) ok = false;
  }
  file.flush();
  file.close();
  if (!ok) { SD.remove(tempPath); return false; }
  SD.remove(backupPath);
  bool hadOriginal = SD.exists(MESSAGE_LOG_PATH);
  if (hadOriginal && !SD.rename(MESSAGE_LOG_PATH, backupPath)) { SD.remove(tempPath); return false; }
  if (!SD.rename(tempPath, MESSAGE_LOG_PATH)) {
    if (hadOriginal) SD.rename(backupPath, MESSAGE_LOG_PATH);
    return false;
  }
  if (hadOriginal) SD.remove(backupPath);
  return true;
}

bool deleteMessageAt(uint8_t index) {
  if (index >= messageCount) return false;
  memmove(&messages[index], &messages[index + 1], sizeof(messages[0]) * (messageCount - index - 1));
  --messageCount;
  if (!messageCount) messageSelection = 0;
  else if (messageSelection >= messageCount) messageSelection = messageCount - 1;
  bool archived = rewriteReceivedMessageLog();
  if (sdReady && !archived) Serial.println("SD: Nachrichtenarchiv konnte nach dem Loeschen nicht aktualisiert werden");
  uiDirty = true;
  return true;
}

bool rewriteStationLog() {
  if (!sdReady) return false;
  static const char tempPath[] = "/aprs_sta.tmp";
  static const char backupPath[] = "/aprs_sta.bak";
  SD.remove(tempPath);
  File file = SD.open(tempPath, FILE_WRITE);
  if (!file) return false;
  int order[16];
  for (int i = 0; i < stationCount; ++i) {
    int j = i;
    while (j > 0 && stations[order[j - 1]].heardAt > stations[i].heardAt) {
      order[j] = order[j - 1];
      --j;
    }
    order[j] = i;
  }
  int first = stationCount > 10 ? stationCount - 10 : 0;
  bool ok = true;
  for (int n = first; n < stationCount; ++n) {
    Station &station = stations[order[n]];
    StoredStation record = {};
    record.marker = STATION_MARKER;
    strncpy(record.call, station.call, sizeof(record.call) - 1);
    record.lat = station.lat;
    record.lon = station.lon;
    record.hasPos = station.hasPos ? 1 : 0;
    if (file.write((const uint8_t *)&record, sizeof(record)) != sizeof(record)) ok = false;
  }
  file.flush();
  file.close();
  if (!ok) { SD.remove(tempPath); return false; }
  SD.remove(backupPath);
  bool hadOriginal = SD.exists(STATION_LOG_PATH);
  if (hadOriginal && !SD.rename(STATION_LOG_PATH, backupPath)) { SD.remove(tempPath); return false; }
  if (!SD.rename(tempPath, STATION_LOG_PATH)) {
    if (hadOriginal) SD.rename(backupPath, STATION_LOG_PATH);
    return false;
  }
  if (hadOriginal) SD.remove(backupPath);
  return true;
}

void loadStationLog() {
  if (!sdReady) return;
  File file = SD.open(STATION_LOG_PATH, FILE_READ);
  if (!file) return;
  while (stationCount < 10 && file.available() >= (int)sizeof(StoredStation)) {
    StoredStation record;
    if (file.read((uint8_t *)&record, sizeof(record)) != sizeof(record)) break;
    if (record.marker != STATION_MARKER || !memchr(record.call, 0, sizeof(record.call))) continue;
    Station &station = stations[stationCount++];
    memset(&station, 0, sizeof(station));
    strncpy(station.call, record.call, sizeof(station.call) - 1);
    station.lat = record.lat;
    station.lon = record.lon;
    station.hasPos = record.hasPos != 0;
    station.heardAt = 0; // Restored stations appear older than stations heard this session.
  }
  file.close();
  if (stationCount) Serial.printf("SD: %u Stationen wiederhergestellt\n", stationCount);
}

bool deleteStationAt(uint8_t index) {
  if (index >= stationCount) return false;
  memmove(&stations[index], &stations[index + 1], sizeof(stations[0]) * (stationCount - index - 1));
  --stationCount;
  if (!stationCount) stationSelection = 0;
  else if (stationSelection >= stationCount) stationSelection = stationCount - 1;
  bool archived = rewriteStationLog();
  if (sdReady && !archived) Serial.println("SD: Stationsliste konnte nach dem Loeschen nicht aktualisiert werden");
  uiDirty = true;
  return true;
}

void initializeMessageStorage() {
  sdReady = SD.begin(PIN_SD_CS, SPI, 20000000);
  if (!sdReady) {
    Serial.println("SD: nicht verfuegbar; empfangene Nachrichten bleiben nur bis zum Neustart gespeichert");
    return;
  }
  Serial.printf("SD: Karte bereit, %llu MB\n", (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)));
  loadReceivedMessages();
  loadStationLog();
}

void startReplyToSelectedMessage() {
  if (!messageCount) {
    composeStage = 1;
    composeTo[0] = 0;
  } else {
    if (messageSelection >= messageCount) messageSelection = messageCount - 1;
    strncpy(composeTo, messages[messageSelection].from, sizeof(composeTo) - 1);
    composeTo[sizeof(composeTo) - 1] = 0;
    composeStage = 2;  // Skip recipient entry; start directly in the reply text.
  }
  composeText[0] = 0;
  uiDirty = true;
}

void startMessageBeep() {
  if (!audioReady || !messageToneEnabled) return;
  messageBeepActive = true;
  messageBeepStartedAt = millis();
  messageBeepSample = 0;
  const int16_t silence[64] = {};
  audioI2S.write(silence, sizeof(silence));
  Serial.println("Audio: 1750-Hz-Piep gestartet (500 ms)");
}

void serviceMessageBeep() {
  if (!messageBeepActive) return;
  if ((uint32_t)(millis() - messageBeepStartedAt) >= 500) {
    messageBeepActive = false;
    const int16_t silence[64] = {};
    audioI2S.write(silence, sizeof(silence));
    return;
  }

  // Send the tone in both I2S slots. MAX98357A board revisions can select
  // either left or right; duplicating the signal makes the alert audible on both.
  // At 28 kHz there are exactly 16 frames per 1750 Hz square-wave cycle.
  int16_t samples[512];
  for (size_t i = 0; i < 256; ++i) {
    int16_t value = ((messageBeepSample++ & 0x08) != 0) ? 2600 : -2600;
    samples[i * 2] = value;
    samples[i * 2 + 1] = value;
  }
  size_t written = audioI2S.write(samples, sizeof(samples));
  if (written != sizeof(samples)) {
    Serial.printf("Audio I2S Schreibfehler: %d, %u/%u Bytes\n", audioI2S.lastError(),
                  (unsigned)written, (unsigned)sizeof(samples));
  }
}

void copyMessageTextForDisplay(const char *source, char *dest, size_t capacity) {
  if (!capacity) return;
  // APRS uses {ID at the end of a message for ACK matching. Keep it in the
  // transmitted/stored protocol string, but omit it from user-facing text.
  const char *id = strchr(source, '{');
  size_t length = id ? (size_t)(id - source) : strlen(source);
  if (length >= capacity) length = capacity - 1;
  memcpy(dest, source, length);
  dest[length] = '\0';
}

void showPacket(const char *packet, int rssi, float snr) {
  wakeDisplay();
  bool beepAfterPacketHandling = false;
  char src[16] = "?", info[160] = "";
  const char *gt = strchr(packet, '>'), *colon = strchr(packet, ':');
  if (!gt || !colon || gt > colon) { Serial.printf("RX raw [%d bytes], RSSI %d SNR %.1f\n", (int)strlen(packet), rssi, snr); return; }
  cleanSource(packet, src, sizeof(src));
  if (!src[0]) strcpy(src, "?");
  if (strcasecmp(src, myCallsign) == 0) {
    Serial.printf("Eigenes Rufzeichen in RX ignoriert: %s\n", src);
    return;
  }
  strncpy(info, colon + 1, sizeof(info) - 1); info[sizeof(info) - 1] = 0;
  double lat, lon;
  int ix = findStation(src);
  stations[ix].heardAt = millis();
  if (extractPosition(info, lat, lon)) { stations[ix].lat = lat; stations[ix].lon = lon; stations[ix].hasPos = true; }
  if (sdReady && !rewriteStationLog()) Serial.println("SD: Stationsliste konnte nicht gespeichert werden");
  Serial.printf("\nRX %s  RSSI %d dBm  SNR %.1f dB\n%s\n", packet, rssi, snr, packet);
  if (stations[ix].hasPos) {
    Serial.printf("  Position: %.5f, %.5f", stations[ix].lat, stations[ix].lon);
    if (gps.location.isValid() && gps.location.age() < GPS_STALE_MS) {
      double km = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), stations[ix].lat, stations[ix].lon) / 1000.0;
      Serial.printf(" | Entfernung: %.1f km", km);
    } else Serial.print(" | eigene GPS-Position fehlt");
    Serial.println();
  }
  // APRS message format: :ADDRESSEE(9 chars):message{ID
  if (info[0] == ':' && strlen(info) > 11 && info[10] == ':') {
    char visibleText[100];
    copyMessageTextForDisplay(info + 11, visibleText, sizeof(visibleText));
    Serial.printf("  APRS-Nachricht an %.9s: %s\n", info + 1, visibleText);
    char target[10]; memcpy(target, info + 1, 9); target[9] = 0;
    for (int i = 8; i >= 0 && target[i] == ' '; --i) target[i] = 0;
    if (strncasecmp(info + 11, "ack", 3) == 0) {
      const char *ackId = info + 14;
      for (int i = 0; i < messageCount; ++i) {
        char suffix[16]; snprintf(suffix, sizeof(suffix), "{%s", ackId);
        if (messages[i].outgoing && strstr(messages[i].text, suffix)) messages[i].acked = true;
      }
    } else if (strcasecmp(target, myCallsign) == 0) {
      storeMessage(src, visibleText, false);
      persistReceivedMessage(src, visibleText);
      beepAfterPacketHandling = true;
    }
  }
  maybeAckMessage(packet, info);
  // ACK transmission is synchronous and can take longer than the 500 ms tone.
  // Start the timer only after that radio operation has finished.
  if (beepAfterPacketHandling) startMessageBeep();
  uiDirty = true;
}

bool sendPacket(const char *payload) {
  // RadioLib APRS-over-LoRa peers require the 3-byte marker 3C FF 01.
  // Incoming packets from APRSClient begin with this marker before TNC2 text.
  size_t payloadLen = strlen(payload);
  if (payloadLen > 200) { Serial.println("Fehler: Paket zu lang (max. 200 Zeichen)"); return false; }
  uint8_t frame[203] = { '<', 0xFF, 0x01 };
  memcpy(frame + 3, payload, payloadLen);
  int state = radio.transmit(frame, payloadLen + 3);
  if (state == RADIOLIB_ERR_NONE) Serial.printf("TX: %s\n", payload);
  else Serial.printf("TX fehlgeschlagen: %d\n", state);
  radio.startReceive();
  return state == RADIOLIB_ERR_NONE;
}

void maybeAckMessage(const char *packet, const char *info) {
  // APRS message info: :ADDRESSEE(9 chars):text{message-id
  if (info[0] != ':' || strlen(info) < 13 || info[10] != ':') return;
  char addressee[10];
  memcpy(addressee, info + 1, 9); addressee[9] = 0;
  for (int i = 8; i >= 0 && addressee[i] == ' '; --i) addressee[i] = 0;
  if (strcasecmp(addressee, myCallsign) != 0) return;

  const char *body = info + 11;
  if (!strncmp(body, "ack", 3) || !strncmp(body, "rej", 3)) return;
  const char *marker = strchr(body, '{');
  if (!marker || marker[1] == 0) return;
  char msgId[12]; size_t idLen = 0;
  while (marker[1 + idLen] && marker[1 + idLen] != ' ' && idLen < sizeof(msgId) - 1) {
    msgId[idLen] = marker[1 + idLen]; ++idLen;
  }
  if (!idLen) return;
  msgId[idLen] = 0;

  const char *gt = strchr(packet, '>');
  if (!gt) return;
  char source[16] = "";
  const char *run = gt;
  while (run > packet) {
    const char *end = run;
    while (run > packet && (isalnum((unsigned char)run[-1]) || run[-1] == '-')) --run;
    if (end - run >= 3 && end - run < (int)sizeof(source)) {
      memcpy(source, run, end - run); source[end - run] = 0; break;
    }
    if (run == packet) break;
    --run;
  }
  if (!source[0]) return;

  char ack[180];
  snprintf(ack, sizeof(ack), "%s>APLTDP,%s::%-9.9s:ack%s", myCallsign, APRS_PATH, source, msgId);
  Serial.printf("  Sende ACK an %s fuer ID %s\n", source, msgId);
  sendPacket(ack);
}

void formatAprsCoordinate(double decimalDegrees, bool latitude, uint8_t ambiguity, char *out, size_t capacity) {
  char hemisphere = latitude ? (decimalDegrees < 0 ? 'S' : 'N') : (decimalDegrees < 0 ? 'W' : 'E');
  double absolute = fabs(decimalDegrees);
  int degrees = (int)absolute;
  double minutes = (absolute - degrees) * 60.0;
  char value[10];
  if (latitude) snprintf(value, sizeof(value), "%02d%05.2f%c", degrees, minutes, hemisphere);
  else snprintf(value, sizeof(value), "%03d%05.2f%c", degrees, minutes, hemisphere);
  int degreeDigits = latitude ? 2 : 3;
  // APRS ambiguity is encoded by blanking progressively less-significant
  // minute digits in both coordinates (0.1', 1', 10', then 1 degree).
  if (ambiguity >= 1) value[degreeDigits + 4] = ' ';
  if (ambiguity >= 2) value[degreeDigits + 3] = ' ';
  if (ambiguity >= 3) value[degreeDigits + 1] = ' ';
  if (ambiguity >= 4) value[degreeDigits] = ' ';
  strncpy(out, value, capacity - 1);
  out[capacity - 1] = 0;
}

void sendBeacon(bool manual = false) {
  if (!gps.location.isValid() || gps.location.age() > GPS_STALE_MS) { Serial.println("Beacon ausgesetzt: GPS-Position nicht aktuell."); return; }
  char lat[10], lon[11], speedExtension[10] = "", altitudeExtension[16] = "", packet[220];
  formatAprsCoordinate(gps.location.lat(), true, positionPrecision, lat, sizeof(lat));
  formatAprsCoordinate(gps.location.lng(), false, positionPrecision, lon, sizeof(lon));
  if (beaconSpeedEnabled && gps.speed.isValid() && gps.speed.age() < GPS_STALE_MS) {
    int course = 0;
    if (gps.course.isValid() && gps.course.age() < GPS_STALE_MS) {
      course = (int)lround(gps.course.deg());
      if (course >= 360) course = 0;
    }
    int speedKnots = constrain((int)lround(gps.speed.kmph() / 1.852), 0, 999);
    snprintf(speedExtension, sizeof(speedExtension), "%03d/%03d", course, speedKnots);
  }
  if (beaconAltitudeEnabled && gps.altitude.isValid() && gps.altitude.age() < GPS_STALE_MS && gps.altitude.meters() >= 0) {
    unsigned long altitudeFeet = min((unsigned long)999999, (unsigned long)lround(gps.altitude.meters() * 3.28084));
    snprintf(altitudeExtension, sizeof(altitudeExtension), "/A=%06lu", altitudeFeet);
  }
  snprintf(packet, sizeof(packet), "%s>APLTDP,%s:=%s/%s%c%s%s%s", myCallsign, APRS_PATH,
           lat, lon, stationSymbol, speedExtension, altitudeExtension, stationComment);
  bool positionSent = sendPacket(packet);
  if (positionSent) ++positionBeaconCount;
  lastBeacon = millis();
  if (gps.course.isValid() && gps.course.age() < 10000) {
    lastBeaconCourseDeg = gps.course.deg();
    lastBeaconCourseValid = true;
  }
  if (positionSent && positionBeaconCount % 10 == 0) {
    updateBatteryReading();
    char statusText[80];
    if (batteryStatusEnabled && batteryReadingValid)
      snprintf(statusText, sizeof(statusText), "%s%sAkku %u%% %.2fV", stationStatus,
               stationStatus[0] ? " | " : "", batteryPercent, batteryVoltage);
    else
      snprintf(statusText, sizeof(statusText), "%s", stationStatus);
    if (statusText[0]) {
      char statusPacket[140];
      snprintf(statusPacket, sizeof(statusPacket), "%s>APLTDP,%s:>%s", myCallsign, APRS_PATH, statusText);
      sendPacket(statusPacket);
      Serial.printf("APRS-Status gesendet (Positionsbake %lu)\n", (unsigned long)positionBeaconCount);
    }
  }
  if (manual && positionSent) showUiNotice("Bake gesendet");
}

float normalizeAngleDifference(float a, float b) {
  float d = fmodf(fabsf(a - b), 360.0f);
  return d > 180.0f ? 360.0f - d : d;
}

void serviceBeaconing() {
  if (!gps.location.isValid() || gps.location.age() > GPS_STALE_MS) return;
  uint32_t now = millis();
  float speedKmh = (gps.speed.isValid() && gps.speed.age() < 10000) ? gps.speed.kmph() : 0.0f;
  uint32_t intervalMs = BEACON_INTERVAL_MS;
  bool turnBeacon = false;

  if (smartBeaconEnabled) {
    if (speedKmh <= smartLowSpeedKmh) {
      intervalMs = (uint32_t)smartSlowRateMin * 60000UL;
    } else if (speedKmh >= smartHighSpeedKmh) {
      intervalMs = (uint32_t)smartFastRateSec * 1000UL;
    } else {
      // Classic SmartBeaconing speed curve, bounded by slow/fast rates.
      float seconds = (float)smartFastRateSec * smartHighSpeedKmh / speedKmh;
      seconds = constrain(seconds, (float)smartFastRateSec, (float)smartSlowRateMin * 60.0f);
      intervalMs = (uint32_t)(seconds * 1000.0f);
    }
    if (intervalMs < SMART_BEACON_MIN_GAP_MS) intervalMs = SMART_BEACON_MIN_GAP_MS;

    if (speedKmh > smartLowSpeedKmh && gps.course.isValid() && gps.course.age() < 10000 && lastBeaconCourseValid) {
      float threshold = smartTurnAngleDeg + (float)smartTurnSlope / speedKmh;
      if (threshold > 120.0f) threshold = 120.0f;
      float change = normalizeAngleDifference((float)gps.course.deg(), lastBeaconCourseDeg);
      uint32_t turnMinimumMs = max(SMART_BEACON_MIN_GAP_MS, (uint32_t)smartTurnTimeSec * 1000UL);
      turnBeacon = change >= threshold && (now - lastBeacon >= turnMinimumMs);
    }
  }

  bool intervalBeacon = (lastBeacon == 0) || (now - lastBeacon >= intervalMs);
  if (intervalBeacon || turnBeacon) {
    Serial.printf("SmartBeacon: %s | %.1f km/h | Intervall %lu s%s\n",
      smartBeaconEnabled ? "aktiv" : "aus (Festintervall)", speedKmh,
      (unsigned long)(intervalMs / 1000UL), turnBeacon && !intervalBeacon ? " | Kursaenderung" : "");
    sendBeacon();
  }
}

void sendMessage(char *line) {
  // Command: m CALLSIGN-SSID message text
  char *target = strtok(line, " \t");
  char *body = strtok(nullptr, "");
  if (!target || !body) { Serial.println("Nutzung: m ZIEL-SSID Nachrichtentext"); return; }
  while (*body == ' ') ++body;
  char padded[10]; memset(padded, ' ', 9); padded[9] = 0;
  size_t n = min(strlen(target), (size_t)9); memcpy(padded, target, n);
  char packet[220];
  char id[8]; snprintf(id, sizeof(id), "%u", nextMessageId++);
  char text[150]; snprintf(text, sizeof(text), "%s{%s", body, id);
  snprintf(packet, sizeof(packet), "%s>APLTDP,%s::%s:%s", myCallsign, APRS_PATH, padded, text);
  storeMessage(target, text, true);
  sendPacket(packet);
}

void printGpsStatus() {
  if (gps.satellites.isValid())
    Serial.printf("GPS: %lu Satelliten", (unsigned long)gps.satellites.value());
  else
    Serial.print("GPS: Satellitenzahl noch nicht verfuegbar");
  if (gps.location.isValid() && gps.location.age() < GPS_STALE_MS) {
    Serial.printf(" | Fix: ja | %.6f, %.6f | Datenalter %lu s\n",
      gps.location.lat(), gps.location.lng(), (unsigned long)(gps.location.age() / 1000));
  } else {
    Serial.printf(" | Fix: nein | letzte Positionsdaten %s\n",
      gps.location.isValid() ? "veraltet" : "keine");
  }
}

void handleCommand(char *line) {
  if (!strcmp(line, "?")) {
    Serial.println("Befehle: ? | b (Position senden) | m ZIEL-SSID Text | p (GPS)");
  } else if (!strcmp(line, "b")) sendBeacon(true);
  else if (!strcmp(line, "p")) printGpsStatus();
  else if (line[0] == 'm' && line[1] == ' ') sendMessage(line + 2);
  else if (*line) Serial.println("Unbekannter Befehl. ? zeigt Hilfe.");
}

void loadSettings() {
  prefs.begin("lora-aprs", false);
  String s = prefs.getString("call", DEFAULT_CALLSIGN); s.toCharArray(myCallsign, sizeof(myCallsign));
  s = prefs.getString("comment", "T-Deck Plus"); s.toCharArray(stationComment, sizeof(stationComment));
  s = prefs.getString("status", "LoRa APRS"); s.toCharArray(stationStatus, sizeof(stationStatus));
  stationSymbol = prefs.getChar("symbol", '>');
  wifiMode = prefs.getUChar("wifi_mode", WLAN_OFF);
  if (wifiMode > WLAN_CLIENT) wifiMode = WLAN_OFF;
  s = prefs.getString("wifi_ssid", "TDeck-AP"); s.toCharArray(wifiSsid, sizeof(wifiSsid));
  s = prefs.getString("wifi_pass", ""); s.toCharArray(wifiPassword, sizeof(wifiPassword));
  smartBeaconEnabled = prefs.getBool("sb_on", true);
  smartSlowRateMin = prefs.getUShort("sb_slow", 10);
  smartFastRateSec = prefs.getUShort("sb_fast", 60);
  smartLowSpeedKmh = prefs.getUShort("sb_low", 5);
  smartHighSpeedKmh = prefs.getUShort("sb_high", 60);
  smartTurnAngleDeg = prefs.getUShort("sb_angle", 25);
  smartTurnSlope = prefs.getUShort("sb_slope", 255);
  smartTurnTimeSec = max((uint16_t)30, prefs.getUShort("sb_time", 30));
  beaconSpeedEnabled = prefs.getBool("tx_speed", false);
  beaconAltitudeEnabled = prefs.getBool("tx_alt", false);
  positionPrecision = min((uint8_t)4, prefs.getUChar("pos_precision", 0));
  messageToneEnabled = prefs.getBool("tone_on", true);
  batteryStatusEnabled = prefs.getBool("bat_status", false);
  if (!myCallsign[0]) strcpy(myCallsign, DEFAULT_CALLSIGN);
  prefs.end();
}

void saveSettings() {
  prefs.begin("lora-aprs", false);
  prefs.putString("call", myCallsign);
  prefs.putString("comment", stationComment);
  prefs.putString("status", stationStatus);
  prefs.putChar("symbol", stationSymbol);
  prefs.putUChar("wifi_mode", wifiMode);
  prefs.putString("wifi_ssid", wifiSsid);
  prefs.putString("wifi_pass", wifiPassword);
  prefs.putBool("sb_on", smartBeaconEnabled);
  prefs.putUShort("sb_slow", smartSlowRateMin);
  prefs.putUShort("sb_fast", smartFastRateSec);
  prefs.putUShort("sb_low", smartLowSpeedKmh);
  prefs.putUShort("sb_high", smartHighSpeedKmh);
  prefs.putUShort("sb_angle", smartTurnAngleDeg);
  prefs.putUShort("sb_slope", smartTurnSlope);
  prefs.putUShort("sb_time", smartTurnTimeSec);
  prefs.putBool("tx_speed", beaconSpeedEnabled);
  prefs.putBool("tx_alt", beaconAltitudeEnabled);
  prefs.putUChar("pos_precision", positionPrecision);
  prefs.putBool("tone_on", messageToneEnabled);
  prefs.putBool("bat_status", batteryStatusEnabled);
  prefs.end();
  Serial.printf("Einstellungen gespeichert: %s Symbol %c\n", myCallsign, stationSymbol);
}

const char *wifiModeName() {
  if (wifiMode == WLAN_AP) return "AP";
  if (wifiMode == WLAN_CLIENT) return "Client";
  return "Aus";
}

void applyWifiMode() {
  lastWifiStatus = -1;
  if (webServerStarted) {
    webServer.stop();
    webServerStarted = false;
  }
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_OFF);
  if (wifiMode == WLAN_OFF) {
    Serial.println("WLAN ausgeschaltet");
  } else if (!wifiSsid[0]) {
    Serial.println("WLAN nicht gestartet: SSID fehlt");
  } else if (wifiMode == WLAN_AP) {
    if (strlen(wifiPassword) < 8) {
      Serial.println("WLAN-AP nicht gestartet: Passwort muss mindestens 8 Zeichen haben");
    } else {
      WiFi.mode(WIFI_AP);
      if (WiFi.softAP(wifiSsid, wifiPassword)) {
        Serial.printf("WLAN-AP '%s' aktiv | IP %s\n", wifiSsid, WiFi.softAPIP().toString().c_str());
      } else {
        Serial.println("WLAN-AP konnte nicht gestartet werden");
        WiFi.mode(WIFI_OFF);
      }
    }
  } else if (wifiMode == WLAN_CLIENT) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(wifiSsid, wifiPassword);
    Serial.printf("WLAN-Client verbindet mit '%s'\n", wifiSsid);
  }
  uiDirty = true;
}

void startSettingEdit() {
  editMode = true;
  switch (settingsSelection) {
    case 0: strncpy(editBuffer, myCallsign, sizeof(editBuffer)); editLimit = 9; break;
    case 1: editBuffer[0] = stationSymbol; editBuffer[1] = 0; editLimit = 1; break;
    case 2: strncpy(editBuffer, stationComment, sizeof(editBuffer)); editLimit = sizeof(stationComment) - 1; break;
    case 3: strncpy(editBuffer, stationStatus, sizeof(editBuffer)); editLimit = sizeof(stationStatus) - 1; break;
    case 5: snprintf(editBuffer, sizeof(editBuffer), "%u", smartSlowRateMin); editLimit = 3; break;
    case 6: snprintf(editBuffer, sizeof(editBuffer), "%u", smartFastRateSec); editLimit = 3; break;
    case 7: snprintf(editBuffer, sizeof(editBuffer), "%u", smartLowSpeedKmh); editLimit = 3; break;
    case 8: snprintf(editBuffer, sizeof(editBuffer), "%u", smartHighSpeedKmh); editLimit = 3; break;
    case 9: snprintf(editBuffer, sizeof(editBuffer), "%u", smartTurnAngleDeg); editLimit = 3; break;
    case 10: snprintf(editBuffer, sizeof(editBuffer), "%u", smartTurnSlope); editLimit = 3; break;
    case 11: snprintf(editBuffer, sizeof(editBuffer), "%u", smartTurnTimeSec); editLimit = 3; break;
    case 20: strncpy(editBuffer, wifiSsid, sizeof(editBuffer)); editLimit = 32; break;
    case 21: strncpy(editBuffer, wifiPassword, sizeof(editBuffer)); editLimit = 63; break;
  }
  editBuffer[sizeof(editBuffer) - 1] = 0;
  uiDirty = true;
}

char presetSymbolCode(uint8_t setting) {
  static const char codes[] = { '$', '>', 'v', 'k', '[', 'F', 's' };
  if (setting < FIRST_SYMBOL_PRESET || setting >= WIFI_MODE_SETTING) return 0;
  return codes[setting - FIRST_SYMBOL_PRESET];
}

void activateSetting() {
  if (settingsSelection == 4) {
    smartBeaconEnabled = !smartBeaconEnabled;
    saveSettings();
    uiDirty = true;
  } else if (settingsSelection >= FIRST_SYMBOL_PRESET && settingsSelection < WIFI_MODE_SETTING) {
    stationSymbol = presetSymbolCode(settingsSelection);
    saveSettings();
    Serial.printf("APRS-Symbol gesetzt: /%c\n", stationSymbol);
    uiDirty = true;
  } else if (settingsSelection == WIFI_MODE_SETTING) {
    wifiMode = (wifiMode + 1) % 3;
    saveSettings();
    applyWifiMode();
  } else if (settingsSelection == 22) {
    messageToneEnabled = !messageToneEnabled;
    if (!messageToneEnabled && audioReady) {
      messageBeepActive = false;
      const int16_t silence[64] = {};
      audioI2S.write(silence, sizeof(silence));
    }
    saveSettings();
    uiDirty = true;
  } else if (settingsSelection == 23) {
    batteryStatusEnabled = !batteryStatusEnabled;
    saveSettings();
    uiDirty = true;
  } else {
    startSettingEdit();
  }
}

void activateSelectedSetting() {
  if (currentPage == 2) settingsSelection = MAIN_SETTING_ITEMS[settingsMenuSelection];
  else if (currentPage == 3) {
    if (detailsSelection < 14) {
      settingsSelection = 5 + detailsSelection;
      activateSetting();
      return;
    }
    if (detailsSelection == 14) beaconSpeedEnabled = !beaconSpeedEnabled;
    else if (detailsSelection == 15) beaconAltitudeEnabled = !beaconAltitudeEnabled;
    else positionPrecision = (positionPrecision + 1) % 5;
    saveSettings();
    uiDirty = true;
    return;
  }
  activateSetting();
}

void commitEdit() {
  if (currentPage == 2 || currentPage == 3) {
    switch (settingsSelection) {
      case 0: if (strlen(editBuffer) >= 3) strncpy(myCallsign, editBuffer, sizeof(myCallsign)); break;
      case 1: if (editBuffer[0]) stationSymbol = editBuffer[0]; break;
      case 2: strncpy(stationComment, editBuffer, sizeof(stationComment) - 1); stationComment[sizeof(stationComment) - 1] = 0; break;
      case 3: strncpy(stationStatus, editBuffer, sizeof(stationStatus) - 1); stationStatus[sizeof(stationStatus) - 1] = 0; break;
      case 5: smartSlowRateMin = constrain(atoi(editBuffer), 1, 100); break;
      case 6: smartFastRateSec = constrain(atoi(editBuffer), 10, 180); break;
      case 7: smartLowSpeedKmh = constrain(atoi(editBuffer), 2, 30); break;
      case 8: smartHighSpeedKmh = constrain(atoi(editBuffer), 3, 90); break;
      case 9: smartTurnAngleDeg = constrain(atoi(editBuffer), 5, 90); break;
      case 10: smartTurnSlope = constrain(atoi(editBuffer), 1, 255); break;
      case 11: smartTurnTimeSec = constrain(atoi(editBuffer), 30, 180); break;
      case 20: strncpy(wifiSsid, editBuffer, sizeof(wifiSsid) - 1); wifiSsid[sizeof(wifiSsid) - 1] = 0; break;
      case 21: strncpy(wifiPassword, editBuffer, sizeof(wifiPassword) - 1); wifiPassword[sizeof(wifiPassword) - 1] = 0; break;
    }
    if (smartHighSpeedKmh <= smartLowSpeedKmh)
      smartHighSpeedKmh = min((uint16_t)90, (uint16_t)(smartLowSpeedKmh + 1));
    if (smartFastRateSec >= smartSlowRateMin * 60)
      smartFastRateSec = max((uint16_t)10, (uint16_t)(smartSlowRateMin * 60 - 1));
    saveSettings();
    if (settingsSelection == 20 || settingsSelection == 21) applyWifiMode();
  }
  editMode = false; uiDirty = true;
}

bool transmitMessage(const char *target, const char *body) {
  if (!target[0] || !body[0]) { Serial.println("Ziel und Nachricht benoetigt."); return false; }
  char padded[10]; memset(padded, ' ', 9); padded[9] = 0;
  memcpy(padded, target, min(strlen(target), (size_t)9));
  char id[8]; snprintf(id, sizeof(id), "%u", nextMessageId++);
  char text[150]; snprintf(text, sizeof(text), "%s{%s", body, id);
  char packet[220]; snprintf(packet, sizeof(packet), "%s>APLTDP,%s::%s:%s", myCallsign, APRS_PATH, padded, text);
  storeMessage(target, text, true);
  return sendPacket(packet);
}



void appendJsonString(String &out, const char *value) {
  out += '"';
  while (*value) {
    uint8_t c = (uint8_t)*value++;
    if (c == '"') out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if (c < 0x20) out += ' ';
    else out += (char)c;
  }
  out += '"';
}

void handleWebState() {
  String json;
  json.reserve(7000);
  json += "{\"callsign\":"; appendJsonString(json, myCallsign);
  bool fix = gps.location.isValid() && gps.location.age() < GPS_STALE_MS;
  json += ",\"fix\":"; json += fix ? "true" : "false";
  json += ",\"lat\":"; json += fix ? String(gps.location.lat(), 6) : "0";
  json += ",\"lon\":"; json += fix ? String(gps.location.lng(), 6) : "0";
  json += ",\"satellites\":"; json += gps.satellites.isValid() ? String(gps.satellites.value()) : "0";
  json += ",\"toneEnabled\":"; json += messageToneEnabled ? "true" : "false";
  json += ",\"batteryValid\":"; json += batteryReadingValid ? "true" : "false";
  json += ",\"batteryVoltage\":"; json += batteryReadingValid ? String(batteryVoltage, 2) : "null";
  json += ",\"batteryPercent\":"; json += batteryReadingValid ? String(batteryPercent) : "null";
  json += ",\"batteryStatusEnabled\":"; json += batteryStatusEnabled ? "true" : "false";
  IPAddress ip = wifiMode == WLAN_AP ? WiFi.softAPIP() : WiFi.localIP();
  json += ",\"ip\":"; appendJsonString(json, ip.toString().c_str());
  json += ",\"stations\":[";
  for (int i = 0; i < stationCount; ++i) {
    if (i) json += ',';
    json += "{\"call\":"; appendJsonString(json, stations[i].call);
    json += ",\"index\":"; json += i;
    json += ",\"lat\":"; json += stations[i].hasPos ? String(stations[i].lat, 6) : "null";
    json += ",\"lon\":"; json += stations[i].hasPos ? String(stations[i].lon, 6) : "null";
    json += ",\"distanceKm\":";
    if (fix && stations[i].hasPos) {
      double km = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), stations[i].lat, stations[i].lon) / 1000.0;
      json += String(km, 2);
      double bearing = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), stations[i].lat, stations[i].lon);
      json += ",\"bearingDeg\":"; json += String(bearing, 1);
    } else {
      json += "null,\"bearingDeg\":null";
    }
    json += '}';
  }
  json += "],\"messages\":[";
  for (int i = 0; i < messageCount; ++i) {
    if (i) json += ',';
    char visibleText[100]; copyMessageTextForDisplay(messages[i].text, visibleText, sizeof(visibleText));
    json += "{\"from\":"; appendJsonString(json, messages[i].from);
    json += ",\"index\":"; json += i;
    json += ",\"text\":"; appendJsonString(json, visibleText);
    json += ",\"outgoing\":"; json += messages[i].outgoing ? "true" : "false";
    json += ",\"acked\":"; json += messages[i].acked ? "true" : "false";
    json += '}';
  }
  json += "]}";
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json; charset=utf-8", json);
}

void handleWebSend() {
  wakeDisplay();
  String target = webServer.arg("to");
  String body = webServer.arg("text");
  target.trim(); target.toUpperCase(); body.trim();
  bool validCall = target.length() >= 1 && target.length() <= 9;
  for (size_t i = 0; validCall && i < target.length(); ++i) {
    char c = target[i];
    if (!isalnum((unsigned char)c) && c != '-') validCall = false;
  }
  bool validText = body.length() > 0 && body.length() <= 100;
  for (size_t i = 0; validText && i < body.length(); ++i) {
    uint8_t c = (uint8_t)body[i];
    if (c < 0x20 || c == 0x7F) validText = false;
  }
  if (!validCall || !validText) {
    webServer.send(400, "application/json; charset=utf-8", "{\"error\":\"Rufzeichen oder Nachricht ungueltig (max. 100 Zeichen).\"}");
    return;
  }
  bool sent = transmitMessage(target.c_str(), body.c_str());
  if (!sent) {
    webServer.send(503, "application/json; charset=utf-8", "{\"error\":\"Funkuebertragung fehlgeschlagen.\"}");
    return;
  }
  webServer.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void handleWebTone() {
  wakeDisplay();
  String enabled = webServer.arg("enabled");
  if (enabled != "0" && enabled != "1") {
    webServer.send(400, "application/json; charset=utf-8", "{\"error\":\"enabled muss 0 oder 1 sein.\"}");
    return;
  }
  messageToneEnabled = enabled == "1";
  if (!messageToneEnabled && audioReady) {
    messageBeepActive = false;
    const int16_t silence[64] = {};
    audioI2S.write(silence, sizeof(silence));
  }
  saveSettings();
  uiDirty = true;
  webServer.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void handleWebBatteryStatus() {
  wakeDisplay();
  String enabled = webServer.arg("enabled");
  if (enabled != "0" && enabled != "1") {
    webServer.send(400, "application/json; charset=utf-8", "{\"error\":\"enabled muss 0 oder 1 sein.\"}");
    return;
  }
  batteryStatusEnabled = enabled == "1";
  saveSettings();
  uiDirty = true;
  webServer.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void handleWebDeleteMessage() {
  wakeDisplay();
  String value = webServer.arg("index");
  bool validIndex = value.length() > 0;
  for (size_t i = 0; i < value.length(); ++i) if (!isDigit(value[i])) validIndex = false;
  if (!validIndex) {
    webServer.send(400, "application/json; charset=utf-8", "{\"error\":\"Nachrichtenindex fehlt.\"}");
    return;
  }
  int index = value.toInt();
  if (index < 0 || index >= messageCount || !deleteMessageAt((uint8_t)index)) {
    webServer.send(404, "application/json; charset=utf-8", "{\"error\":\"Nachricht nicht gefunden.\"}");
    return;
  }
  webServer.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void handleWebDeleteStation() {
  wakeDisplay();
  String value = webServer.arg("index");
  bool validIndex = value.length() > 0;
  for (size_t i = 0; i < value.length(); ++i) if (!isDigit(value[i])) validIndex = false;
  if (!validIndex) {
    webServer.send(400, "application/json; charset=utf-8", "{\"error\":\"Stationsindex fehlt.\"}");
    return;
  }
  int index = value.toInt();
  if (index < 0 || index >= stationCount || !deleteStationAt((uint8_t)index)) {
    webServer.send(404, "application/json; charset=utf-8", "{\"error\":\"Station nicht gefunden.\"}");
    return;
  }
  webServer.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void configureWebRoutes() {
  if (webRoutesConfigured) return;
  webServer.on("/", HTTP_GET, []() { webServer.send_P(200, "text/html; charset=utf-8", WEB_PAGE); });
  webServer.on("/api/state", HTTP_GET, handleWebState);
  webServer.on("/api/send", HTTP_POST, handleWebSend);
  webServer.on("/api/tone", HTTP_POST, handleWebTone);
  webServer.on("/api/battery-status", HTTP_POST, handleWebBatteryStatus);
  webServer.on("/api/message/delete", HTTP_POST, handleWebDeleteMessage);
  webServer.on("/api/station/delete", HTTP_POST, handleWebDeleteStation);
  webServer.onNotFound([]() { webServer.send(404, "text/plain; charset=utf-8", "Nicht gefunden"); });
  webRoutesConfigured = true;
}

void serviceWebServer() {
  bool networkReady = (wifiMode == WLAN_AP && WiFi.getMode() == WIFI_AP) ||
                      (wifiMode == WLAN_CLIENT && WiFi.status() == WL_CONNECTED);
  if (networkReady && !webServerStarted) {
    webServer.begin();
    webServerStarted = true;
    IPAddress ip = wifiMode == WLAN_AP ? WiFi.softAPIP() : WiFi.localIP();
    Serial.printf("APRS-Webseite: http://%s/\n", ip.toString().c_str());
  }
  if (webServerStarted) webServer.handleClient();
}

void appendChar(char *dst, size_t cap, char c) {
  size_t n = strlen(dst);
  if (n + 1 < cap && isPrintable(c)) { dst[n] = c; dst[n + 1] = 0; }
}

void handleKey(char key) {
  if (key == 0) return;
  wakeDisplay();
  if (composeStage) {
    char *target = composeStage == 1 ? composeTo : composeText;
    size_t cap = composeStage == 1 ? sizeof(composeTo) : sizeof(composeText);
    if (key == '\r' || key == '\n') {
      if (composeStage == 1 && composeTo[0]) composeStage = 2;
      else if (composeStage == 2 && composeText[0]) {
        transmitMessage(composeTo, composeText); composeStage = 0;
        composeTo[0] = composeText[0] = 0;
      }
    } else if (key == '\b' || key == 127) {
      size_t n = strlen(target); if (n) target[n - 1] = 0;
    } else appendChar(target, cap, key);
    uiDirty = true; return;
  }
  if (editMode) {
    if (key == '\r' || key == '\n') commitEdit();
    else if (key == '\b' || key == 127) { size_t n = strlen(editBuffer); if (n) editBuffer[n - 1] = 0; }
    else appendChar(editBuffer, min((size_t)editLimit + 1, sizeof(editBuffer)), key);
    uiDirty = true; return;
  }
  if (key == '\r' || key == '\n') {
    if (currentPage == 1) { composeStage = 1; composeTo[0] = composeText[0] = 0; uiDirty = true; }
    else if (currentPage == 2 || currentPage == 3) activateSelectedSetting();
  } else if (currentPage == 1 && (key == 'm' || key == 'M')) {
    composeStage = 1; composeTo[0] = composeText[0] = 0; uiDirty = true;
  } else if (currentPage == 1 && (key == 'r' || key == 'R')) {
    startReplyToSelectedMessage();
  } else if (currentPage == 1 && (key == 'd' || key == 'D') && messageCount) {
    if (deleteMessageAt(messageSelection)) showUiNotice("Nachricht geloescht");
  } else if (currentPage == 0 && (key == 'd' || key == 'D') && stationCount) {
    if (deleteStationAt(stationSelection)) showUiNotice("Station geloescht");
  } else if (key >= '1' && key <= '4') { currentPage = key - '1'; uiDirty = true; }
}

void handleTrackball(int dx, int dy, bool click) {
  if (editMode) { if (click) commitEdit(); return; }
  if (composeStage) { if (click) { composeStage = 0; composeTo[0] = composeText[0] = 0; uiDirty = true; } return; }
  // Trackball sensors may emit a short burst of edges for one movement.
  // Limit page navigation to one step per deliberate movement interval.
  bool changed = false;
  if (dx && millis() - lastPageChange >= 350) {
    if (dx < 0) currentPage = (currentPage + 3) % 4;
    else currentPage = (currentPage + 1) % 4;
    lastPageChange = millis();
    changed = true;
  }
  if (dy != 0) {
    if (currentPage == 0 && stationCount) { int old = stationSelection; stationSelection = constrain((int)stationSelection - dy, 0, stationCount - 1); changed |= old != stationSelection; }
    if (currentPage == 1 && messageCount) { int old = messageSelection; messageSelection = constrain((int)messageSelection - dy, 0, messageCount - 1); changed |= old != messageSelection; }
    if (currentPage == 2) { int old = settingsMenuSelection; settingsMenuSelection = constrain((int)settingsMenuSelection - dy, 0, MAIN_SETTING_COUNT - 1); changed |= old != settingsMenuSelection; }
    if (currentPage == 3) { int old = detailsSelection; detailsSelection = constrain((int)detailsSelection - dy, 0, DETAIL_SETTING_COUNT - 1); changed |= old != detailsSelection; }
  }
  if (click) {
    if (currentPage == 1 && !composeStage) { startReplyToSelectedMessage(); changed = true; }
    else if ((currentPage == 2 || currentPage == 3) && !editMode) { activateSelectedSetting(); changed = true; }
    else if (editMode) { commitEdit(); changed = true; }
  }
  if (changed) uiDirty = true;
}

void setKeyboardBacklight(uint8_t brightness) {
  Wire.beginTransmission(KEYBOARD_ADDR);
  Wire.write((uint8_t)0x01); // LILYGO keyboard brightness command
  Wire.write(brightness);    // 0 = off, 127 = comfortable default brightness
  Wire.endTransmission();
  keyboardBacklightOn = brightness != 0;
}

void pollKeyboard() {
  // LILYGO's master example polls the keyboard MCU directly; relying on INT
  // can miss input on keyboard firmware revisions that do not hold INT low.
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll < 5) return;
  lastPoll = millis();
  Wire.requestFrom((uint8_t)KEYBOARD_ADDR, (uint8_t)1);
  if (Wire.available()) {
    int rawKey = Wire.read();
    if (rawKey > 0 && rawKey <= 0x7F) {
      char key = (char)rawKey;
      lastKeyboardActivity = millis();
      if (!keyboardBacklightOn) setKeyboardBacklight(127);
      handleKey(key);
    }
  }
}

void pollTrackball() {
  int16_t x, y; uint32_t clicks;
  noInterrupts(); x = trackX; y = trackY; clicks = trackClickFlag; interrupts();
  int dx = x - handledTrackX, dy = y - handledTrackY;
  if ((dx || dy) && millis() - lastTrackEvent > 70) {
    wakeDisplay();
    handledTrackX = x; handledTrackY = y; lastTrackEvent = millis();
    handleTrackball(dx, dy, false);
  }

  static bool clickTracking = false;
  static bool longPressHandled = false;
  static uint32_t clickStartedAt = 0;
  static uint32_t releaseCandidateAt = 0;
  static uint8_t clickStartedPage = 0;

  if (clicks != handledClick) {
    wakeDisplay();
    handledClick = clicks;
    if (!clickTracking) {
      clickTracking = true;
      longPressHandled = false;
      clickStartedAt = millis();
      releaseCandidateAt = 0;
      clickStartedPage = currentPage;
    }
  }

  if (clickTracking && digitalRead(PIN_TRACK_CLICK) == LOW) {
    releaseCandidateAt = 0;
    if (!longPressHandled && clickStartedPage == 0 && millis() - clickStartedAt >= 2000) {
      longPressHandled = true;
      Serial.println("Trackball-Langdruck: manuelle Positionsbake");
      sendBeacon(true);
    }
  } else if (clickTracking) {
    if (!releaseCandidateAt) releaseCandidateAt = millis();
    if (millis() - releaseCandidateAt >= 30) {
      if (!longPressHandled) handleTrackball(0, 0, true);
      clickTracking = false;
      releaseCandidateAt = 0;
    }
  }
}

void drawHeader(const char *title) {
  display->fillScreen(C_BG);
  display->fillRect(0, 0, 320, 34, C_PANEL);
  display->setTextSize(2); display->setTextColor(C_CYAN, C_PANEL);
  display->setCursor(8, 8); display->print(title);
  display->setTextSize(1); display->setTextColor(C_MUTED, C_PANEL);
  drawBatteryIndicator();
  display->setTextSize(1); display->setTextColor(C_MUTED, C_PANEL);
  display->setCursor(294, 12); display->printf("%u/4", currentPage + 1);
  display->drawFastHLine(0, 36, 320, C_CYAN);
}

void drawBatteryIndicator() {
  if (!displayInitialized || !displayAwake) return;
  display->fillRect(164, 0, 120, 34, C_PANEL);
  display->setTextSize(2); display->setTextColor(C_CYAN, C_PANEL);
  display->setCursor(166, 8);
  if (batteryReadingValid) display->printf("Akku %u%%", batteryPercent);
  else display->print("Akku --%");
}

void drawFooter() {
  display->drawFastHLine(0, 214, 320, C_PANEL);
  display->setTextSize(1); display->setTextColor(C_MUTED, C_BG);
  display->setCursor(7, 222);
  if (gps.location.isValid() && gps.location.age() < GPS_STALE_MS)
    display->printf("GPS %lu sat | %s", (unsigned long)gps.satellites.value(), myCallsign);
  else display->printf("GPS kein Fix | %s", myCallsign);
  display->setCursor(225, 222); display->print("< > Seite");
}

void updateFooter() {
  // Refresh live GPS data without repainting the whole screen.
  if (!displayAwake) return;
  display->fillRect(0, 214, 320, 26, C_BG);
  drawFooter();
}

void showUiNotice(const char *message) {
  strncpy(uiNotice, message, sizeof(uiNotice) - 1);
  uiNotice[sizeof(uiNotice) - 1] = 0;
  uiNoticeUntil = millis() + 2000;
  uiDirty = true;
}

void wakeDisplay() {
  lastDisplayActivity = millis();
  if (!displayAwake) {
    displayAwake = true;
    digitalWrite(PIN_TFT_BL, HIGH);
    uiDirty = true;
  }
}

uint8_t batteryPercentFromVoltage(float voltage) {
  static const uint16_t millivolts[] = {3300, 3500, 3600, 3700, 3750, 3800, 3850, 3900, 4000, 4100, 4200};
  static const uint8_t percentages[] = {0, 5, 10, 20, 30, 40, 50, 60, 75, 90, 100};
  uint16_t mv = (uint16_t)(voltage * 1000.0f + 0.5f);
  if (mv <= millivolts[0]) return 0;
  for (size_t i = 1; i < sizeof(millivolts) / sizeof(millivolts[0]); ++i) {
    if (mv <= millivolts[i]) {
      uint16_t span = millivolts[i] - millivolts[i - 1];
      uint16_t offset = mv - millivolts[i - 1];
      return percentages[i - 1] + (uint8_t)(((uint16_t)(percentages[i] - percentages[i - 1]) * offset + span / 2) / span);
    }
  }
  return 100;
}

void updateBatteryReading() {
  uint32_t sumMillivolts = 0;
  const uint8_t samples = 12;
  for (uint8_t i = 0; i < samples; ++i) {
    sumMillivolts += analogReadMilliVolts(PIN_BATTERY_ADC);
    delayMicroseconds(200);
  }
  batteryVoltage = (sumMillivolts / (float)samples) * 2.0f / 1000.0f;
  batteryReadingValid = batteryVoltage >= 2.5f && batteryVoltage <= 5.0f;
  if (batteryReadingValid) batteryPercent = batteryPercentFromVoltage(batteryVoltage);
  lastBatteryRead = millis();
  applyBatteryCpuFrequency();
  drawBatteryIndicator();
}

void applyBatteryCpuFrequency() {
  if (!batteryReadingValid) return;
  uint32_t targetFrequency = activeCpuFrequencyMhz;
  if (activeCpuFrequencyMhz == 160 && batteryPercent < 25) targetFrequency = 80;
  else if (activeCpuFrequencyMhz == 80 && batteryPercent > 30) targetFrequency = 160;
  if (targetFrequency != activeCpuFrequencyMhz && setCpuFrequencyMhz(targetFrequency)) {
    activeCpuFrequencyMhz = getCpuFrequencyMhz();
    Serial.printf("CPU-Takt angepasst: %lu MHz bei Akku %u%%\n",
                  (unsigned long)activeCpuFrequencyMhz, batteryPercent);
  }
}

void sleepDisplay() {
  if (!displayAwake) return;
  displayAwake = false;
  digitalWrite(PIN_TFT_BL, LOW);
}

void drawStations() {
  drawHeader("Stationen");
  display->setTextSize(1); display->setTextColor(C_MUTED, C_BG);
  display->setCursor(8, 43); display->print("d: loeschen | Rufzeichen Distanz Richtung");
  if (!stationCount) {
    display->setTextColor(C_WHITE, C_BG); display->setCursor(20, 95); display->setTextSize(2);
    display->print("Warte auf Stationen...");
  }
  int first = stationSelection > 6 ? stationSelection - 6 : 0;
  for (int row = 0; row < 7 && first + row < stationCount; ++row) {
    int i = first + row; int y = 62 + row * 21;
    bool selected = i == stationSelection;
    if (selected) display->fillRoundRect(4, y - 2, 312, 19, 3, C_PANEL);
    display->setTextSize(1); display->setTextColor(selected ? C_YELLOW : C_WHITE, selected ? C_PANEL : C_BG);
    display->setCursor(9, y); display->print(stations[i].call);
    if (stations[i].hasPos) {
      if (gps.location.isValid() && gps.location.age() < GPS_STALE_MS) {
        double km = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), stations[i].lat, stations[i].lon) / 1000.0;
        double deg = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), stations[i].lat, stations[i].lon);
        display->setCursor(130, y); if (km < 10) display->printf("%.2f km", km); else display->printf("%.1f km", km);
        display->setCursor(235, y); display->printf("%03.0f deg", deg);
      } else {
        // The remote station's coordinates are still useful immediately after
        // reboot, before this device has acquired its own GPS fix.
        display->setTextColor(C_WHITE, selected ? C_PANEL : C_BG);
        display->setCursor(130, y); display->printf("%.4f,%.4f", stations[i].lat, stations[i].lon);
      }
    } else {
      display->setTextColor(C_MUTED, selected ? C_PANEL : C_BG); display->setCursor(130, y); display->print("keine Position");
    }
  }
  drawFooter();
}

void drawMessages() {
  drawHeader("Nachrichten");
  display->setTextSize(1); display->setTextColor(C_MUTED, C_BG); display->setCursor(8, 43);
  display->print("Klick/R: Antwort | d: loeschen | Enter/m: neu");
  int first = messageSelection > 4 ? messageSelection - 4 : 0;
  for (int row = 0; row < 5 && first + row < messageCount; ++row) {
    int i = first + row; int y = 61 + row * 22; AprsMessage &m = messages[i];
    if (i == messageSelection && !composeStage) display->fillRoundRect(4, y - 2, 312, 20, 3, C_PANEL);
    display->setTextColor(m.outgoing ? C_CYAN : C_WHITE, i == messageSelection && !composeStage ? C_PANEL : C_BG);
    display->setCursor(8, y); display->printf("%c %s", m.outgoing ? '>' : '<', m.from);
    char visibleText[100]; copyMessageTextForDisplay(m.text, visibleText, sizeof(visibleText));
    display->setCursor(112, y); display->printf("%.24s", visibleText);
    if (m.outgoing) { display->setCursor(284, y); display->setTextColor(m.acked ? C_GREEN : C_YELLOW, i == messageSelection && !composeStage ? C_PANEL : C_BG); display->print(m.acked ? "ACK" : "..."); }
  }
  if (composeStage) {
    display->fillRect(0, 174, 320, 40, C_PANEL);
    display->setTextColor(C_YELLOW, C_PANEL); display->setCursor(7, 179);
    if (composeStage == 1) display->printf("An: %s_", composeTo);
    else { display->printf("An %s", composeTo); display->setCursor(7, 195); display->printf("Text: %s_", composeText); }
  } else {
    display->setTextColor(C_MUTED, C_BG); display->setCursor(8, 190);
    display->print("Klick/R antwortet; Enter/m erstellt neu");
  }
  drawFooter();
}

void drawSettings() {
  drawHeader("Optionen");
  static const char *labels[] = {
    "Rufzeichen", "Symbol", "Kommentar", "Status", "SmartBeacon",
    "Slow rate min", "Fast rate s", "Low speed km/h", "High speed km/h",
    "Turn angle deg", "Turn slope", "Turn time s", "Telefon /$", "Auto />",
    "Van /v", "LKW /k", "Mensch /[", "Trecker /F", "Schiff /s",
    "Wireless", "WLAN SSID", "WLAN Passwort", "Nachrichtenton", "Akku im Status"
  };
  uint8_t first = settingsMenuSelection > 4 ? settingsMenuSelection - 4 : 0;
  for (int row = 0; row < 5 && first + row < MAIN_SETTING_COUNT; ++row) {
    uint8_t item = first + row;
    uint8_t i = MAIN_SETTING_ITEMS[item];
    int y = 47 + row * 31;
    bool selected = item == settingsMenuSelection;
    if (selected) display->fillRoundRect(4, y - 2, 312, 29, 4, C_PANEL);
    display->setTextSize(1); display->setTextColor(selected ? C_YELLOW : C_MUTED, selected ? C_PANEL : C_BG);
    display->setCursor(9, y + 3); display->print(labels[i]);
    display->setTextColor(C_WHITE, selected ? C_PANEL : C_BG); display->setCursor(157, y + 3);
    switch (i) {
      case 0: display->print(myCallsign); break;
      case 1: display->printf("/%c", stationSymbol); break;
      case 2: display->printf("%.18s", stationComment); break;
      case 3: display->printf("%.18s", stationStatus); break;
      case 4: display->print(smartBeaconEnabled ? "AN" : "AUS"); break;
      case 19: display->print(wifiModeName()); break;
      case 20: display->printf("%.18s", wifiSsid); break;
      case 21: display->print(wifiPassword[0] ? "********" : "(leer)"); break;
      case 22: display->print(messageToneEnabled ? "AN" : "AUS"); break;
      case 23: display->print(batteryStatusEnabled ? "AN" : "AUS"); break;
      default: display->print(stationSymbol == presetSymbolCode(i) ? "AKTIV" : "setzen"); break;
    }
    if (i == 19) {
      String ipLine;
      if (wifiMode == WLAN_AP && WiFi.getMode() == WIFI_AP)
        ipLine = String("IP: ") + WiFi.softAPIP().toString();
      else if (wifiMode == WLAN_CLIENT && WiFi.status() == WL_CONNECTED)
        ipLine = String("IP: ") + WiFi.localIP().toString();
      else if (wifiMode == WLAN_CLIENT)
        ipLine = "IP: wartet auf Verbindung";
      else if (wifiMode == WLAN_AP)
        ipLine = "IP: AP nicht gestartet";
      else
        ipLine = "IP: WLAN aus";
      display->setTextColor(selected ? C_CYAN : C_MUTED, selected ? C_PANEL : C_BG);
      display->setCursor(9, y + 15); display->print(ipLine);
    }
    if (editMode && selected) {
      display->setTextColor(C_CYAN, C_PANEL); display->setCursor(157, y + 15);
      display->printf(">%s_", editBuffer);
    }
  }
  display->setTextSize(1); display->setTextColor(C_MUTED, C_BG); display->setCursor(8, 204);
  display->print("Enter/Klick: aendern");
  drawFooter();
}

void drawDetails() {
  drawHeader("Details");
  static const char *labels[DETAIL_SETTING_COUNT] = {
    "Slow rate min", "Fast rate s", "Low speed km/h", "High speed km/h",
    "Turn angle deg", "Turn slope", "Turn time s", "Telefon /$", "Auto />",
    "Van /v", "LKW /k", "Mensch /[", "Trecker /F", "Schiff /s",
    "Tempo", "Hoehe", "Genauigkeit"
  };
  uint8_t section = detailsSelection < 7 ? 0 : (detailsSelection < 14 ? 1 : 2);
  uint8_t sectionStart = section == 0 ? 0 : (section == 1 ? 7 : 14);
  uint8_t sectionCount = section == 2 ? 3 : 7;
  uint8_t groupSelection = detailsSelection - sectionStart;
  uint8_t first = groupSelection > 4 ? groupSelection - 4 : 0;
  display->setTextSize(1); display->setTextColor(C_CYAN, C_BG);
  display->setCursor(8, 43);
  if (section == 0) display->print("SmartBeaconing:");
  else if (section == 1) display->print("APRS Symbol:");
  else display->print("Positionsgenauigkeit:");
  for (int row = 0; row < 5 && first + row < sectionCount; ++row) {
    uint8_t item = sectionStart + first + row;
    int y = 61 + row * 26;
    bool selected = item == detailsSelection;
    if (selected) display->fillRoundRect(4, y - 2, 312, 29, 4, C_PANEL);
    display->setTextSize(1); display->setTextColor(selected ? C_YELLOW : C_MUTED, selected ? C_PANEL : C_BG);
    display->setCursor(9, y + 3); display->print(labels[item]);
    display->setTextColor(C_WHITE, selected ? C_PANEL : C_BG); display->setCursor(157, y + 3);
    if (section == 0) {
      switch (item) {
        case 0: display->printf("%u", smartSlowRateMin); break;
        case 1: display->printf("%u", smartFastRateSec); break;
        case 2: display->printf("%u", smartLowSpeedKmh); break;
        case 3: display->printf("%u", smartHighSpeedKmh); break;
        case 4: display->printf("%u", smartTurnAngleDeg); break;
        case 5: display->printf("%u", smartTurnSlope); break;
        case 6: display->printf("%u", smartTurnTimeSec); break;
      }
    } else if (section == 1) {
      uint8_t setting = 5 + item;
      display->print(stationSymbol == presetSymbolCode(setting) ? "AKTIV" : "setzen");
    } else if (item == 14) {
      display->print(beaconSpeedEnabled ? "Ja" : "Nein");
    } else if (item == 15) {
      display->print(beaconAltitudeEnabled ? "Ja" : "Nein");
    } else {
      static const char *precisionLabels[] = {
        "DD\xB0 MM.mm'", "DD\xB0 MM.m'", "DD\xB0 MM'", "DD\xB0 Mx'", "DD\xB0"
      };
      display->print(precisionLabels[positionPrecision]);
    }
    if (editMode && selected) {
      display->setTextColor(C_CYAN, C_PANEL); display->setCursor(157, y + 15);
      display->printf(">%s_", editBuffer);
    }
  }
  display->setTextSize(1); display->setTextColor(C_MUTED, C_BG); display->setCursor(8, 204);
  display->print("Enter/Klick: aendern");
  drawFooter();
}

void drawUi() {
  if (!displayAwake) return;
  if (currentPage == 0) drawStations();
  else if (currentPage == 1) drawMessages();
  else if (currentPage == 2) drawSettings();
  else drawDetails();
  if (uiNotice[0] && (int32_t)(millis() - uiNoticeUntil) < 0) {
    display->fillRoundRect(56, 178, 208, 30, 5, C_GREEN);
    display->setTextSize(2); display->setTextColor(C_WHITE, C_GREEN);
    display->setCursor(77, 185); display->print(uiNotice);
  }
  uiDirty = false;
}

void setup() {
  setCpuFrequencyMhz(160);
  Serial.begin(115200);
  delay(250);
  Serial.printf("BOOT: setup gestartet | CPU %u MHz\n", (unsigned)getCpuFrequencyMhz());
  Serial.println("BOOT: GPIO10 vorbereiten");
  pinMode(PIN_POWER, OUTPUT); digitalWrite(PIN_POWER, HIGH);
  pinMode(PIN_BATTERY_ADC, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);
  updateBatteryReading();
  Serial.println("BOOT: GPIO10 HIGH gesetzt");
  delay(200);
  Serial.println("BOOT: Peripherie-Strom an");
  audioI2S.setPins(PIN_I2S_BCK, PIN_I2S_WS, PIN_I2S_DOUT);
  audioReady = audioI2S.begin(I2S_MODE_STD, 28000, I2S_DATA_BIT_WIDTH_16BIT,
                              I2S_SLOT_MODE_STEREO);
  Serial.printf("I2S Audio: %s\n", audioReady ? "bereit" : "nicht verfuegbar");
  pinMode(PIN_TFT_CS, OUTPUT); digitalWrite(PIN_TFT_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_LORA_CS);
  Serial.println("BOOT: SPI bereit");
  initializeMessageStorage();
  loadSettings();
  Serial.println("BOOT: Einstellungen geladen");
  pinMode(PIN_TFT_BL, OUTPUT); digitalWrite(PIN_TFT_BL, HIGH);
  displayAwake = true;
  lastDisplayActivity = millis();
  Serial.println("TFT_eSPI init startet");
  display->init();
  displayInitialized = true;
  display->setRotation(1); // Setup210: portrait panel rotated to 320x240 landscape
  display->setTextFont(1); // Explicitly select TFT_eSPI's built-in GLCD font
  Serial.println("TFT_eSPI init abgeschlossen");
  display->fillScreen(0xF800); delay(120);
  display->fillScreen(0x07E0); delay(120);
  display->fillScreen(0x001F); delay(120);
  display->fillScreen(C_BG);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  pinMode(PIN_KB_INT, INPUT_PULLUP);
  Wire.requestFrom((uint8_t)KEYBOARD_ADDR, (uint8_t)1);
  bool keyboardOnline = Wire.available();
  while (Wire.available()) Wire.read();
  Serial.println(keyboardOnline ? "Tastatur-I2C: online" : "Tastatur-I2C: keine Antwort");
  setKeyboardBacklight(0);
  pinMode(PIN_TRACK_UP, INPUT_PULLUP); pinMode(PIN_TRACK_RIGHT, INPUT_PULLUP);
  pinMode(PIN_TRACK_DOWN, INPUT_PULLUP); pinMode(PIN_TRACK_LEFT, INPUT_PULLUP);
  pinMode(PIN_TRACK_CLICK, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TRACK_UP), onTrackUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_TRACK_RIGHT), onTrackRight, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_TRACK_DOWN), onTrackDown, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_TRACK_LEFT), onTrackLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_TRACK_CLICK), onTrackClick, FALLING);
  // T-Deck Plus MIA-M10Q UART: GPS TX -> ESP32 GPIO44 (RX), GPS RX <- GPIO43 (TX).
  gpsSerial.begin(38400, SERIAL_8N1, 44, 43);
  Serial.printf("LoRa APRS | %s | %.3f MHz | SF%u/BW%.0f\n", myCallsign, LORA_MHZ, LORA_SF, LORA_BW_KHZ);
  // RadioLib 8.x takes a ConfigLoRa_t object. Frequencies/bandwidth are Hz.
  ConfigLoRa_t loraConfig;
  loraConfig.frequency = (uint32_t)(LORA_MHZ * 1000000.0f + 0.5f);
  loraConfig.bandwidth = (uint32_t)(LORA_BW_KHZ * 1000.0f + 0.5f);
  loraConfig.spreadingFactor = LORA_SF;
  loraConfig.codingRate = LORA_CR;
  loraConfig.syncWord = LORA_SYNC_WORD;
  loraConfig.power = LORA_TX_DBM;
  int state = radio.begin(loraConfig);
  if (state != RADIOLIB_ERR_NONE) { Serial.printf("Radio init failed: %d\n", state); while (true) delay(1000); }
  radio.setDio1Action(onDio1);
  radio.startReceive();
  configureWebRoutes();
  applyWifiMode();
  Serial.println("LoRa APRS bereit. Trackball links/rechts wechselt Seiten; Tastatur bedient Eingaben.");
  uiDirty = true; drawUi();
  printGpsStatus();
}

void loop() {
  if (millis() - lastBatteryRead >= 5000) updateBatteryReading();
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  pollKeyboard();
  pollTrackball();
  if (wifiMode == WLAN_CLIENT) {
    int status = (int)WiFi.status();
    if (status != lastWifiStatus) {
      lastWifiStatus = status;
      if (status == WL_CONNECTED)
        Serial.printf("WLAN verbunden | IP %s\n", WiFi.localIP().toString().c_str());
      else
        Serial.println("WLAN-Client noch nicht verbunden");
      if (currentPage == 2) uiDirty = true;
    }
  } else {
    lastWifiStatus = -1;
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\r') wakeDisplay();
    if (c == '\r') continue;
    if (c == '\n') { inputLine[inputLen] = 0; handleCommand(inputLine); inputLen = 0; }
    else if (inputLen < sizeof(inputLine) - 1) inputLine[inputLen++] = c;
  }
  if (packetReady) {
    packetReady = false;
    uint8_t buffer[256]; size_t len = radio.getPacketLength();
    if (len > 0 && len < sizeof(buffer)) {
      int state = radio.readData(buffer, len);
      if (state == RADIOLIB_ERR_NONE) {
        buffer[len] = 0; ++rxCount;
        showPacket((char *)buffer, radio.getRSSI(), radio.getSNR());
      }
    }
    radio.startReceive();
  }
  serviceMessageBeep();
  serviceBeaconing();
  serviceWebServer();
  if (millis() - lastFooterUpdate >= 1000) {
    lastFooterUpdate = millis();
    updateFooter();
  }
  if (keyboardBacklightOn && millis() - lastKeyboardActivity >= 5000) {
    setKeyboardBacklight(0);
  }
  if (uiNotice[0] && (int32_t)(millis() - uiNoticeUntil) >= 0) {
    uiNotice[0] = 0;
    uiDirty = true;
  }
  if (displayAwake && millis() - lastDisplayActivity >= DISPLAY_IDLE_TIMEOUT_MS) sleepDisplay();
  if (uiDirty) drawUi();
  delay(2);
}
