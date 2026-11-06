#include <LittleFS.h>

#define RXD2 16
#define TXD2 17

const uint32_t BAUD = 9600;
const uint8_t  ADDR = 0x01;

const uint16_t REG_MOIST = 0x0000;   // ÷10 -> %
const uint16_t REG_TEMP  = 0x0001;   // ÷10 -> °C (signed)
const uint16_t REG_EC    = 0x0002;   // uS/cm
const uint16_t REG_PH    = 0x0003;   // ÷10

const uint8_t  NUM_READINGS = 5;
const uint32_t READ_INTERVAL_MS = 1000;
const char *LOG_FILE = "/soil_avg.jsonl";

volatile bool doRead = false;   // set true (or 1) to trigger a batch

struct SoilReading {
  uint32_t timeMs;
  float moisture;   // %
  float temp;       // °C
  float ec;         // uS/cm
  float ph;
  bool valid;
};

SoilReading readings[NUM_READINGS];   // latest batch (RAM)
SoilReading avgReading;              // average of latest batch
uint8_t  avgValidCount = 0;
uint32_t batchCount = 0;

// ---------------- Modbus ----------------
uint16_t crc16(const uint8_t *b, uint8_t len) {
  uint16_t c = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    c ^= b[i];
    for (uint8_t j = 0; j < 8; j++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
  }
  return c;
}

bool readReg(uint16_t reg, uint16_t &val) {
  uint8_t req[8] = {ADDR, 0x03, (uint8_t)(reg >> 8), (uint8_t)reg, 0x00, 0x01, 0, 0};
  uint16_t c = crc16(req, 6); req[6] = c & 0xFF; req[7] = c >> 8;

  while (Serial2.available()) Serial2.read();
  Serial2.write(req, 8);
  Serial2.flush();

  uint8_t r[7], n = 0;
  uint32_t t = millis();
  while (n < 7 && millis() - t < 300)
    if (Serial2.available()) r[n++] = Serial2.read();

  if (n < 7) {
    Serial.printf("  reg 0x%04X: got %u bytes\n", reg, n);
    return false;
  }
  uint16_t rc = crc16(r, 5);
  if (r[1] != 0x03 || r[5] != (rc & 0xFF) || r[6] != (rc >> 8)) {
    Serial.printf("  reg 0x%04X: bad CRC\n", reg);
    return false;
  }

  val = (r[3] << 8) | r[4];
  return true;
}

// ---------------- JSON storage ----------------
void logAvgJSON() {
  if (avgValidCount == 0) {
    Serial.println("No valid readings, batch not logged");
    return;
  }

  File f = LittleFS.open(LOG_FILE, FILE_APPEND);
  if (!f) { Serial.println("Log write failed"); return; }

  f.printf("{\"batch\":%lu,\"time_ms\":%lu,\"moisture\":%.1f,\"temp\":%.1f,\"ec\":%.0f,\"ph\":%.1f,\"valid\":%u}\n",
           batchCount, avgReading.timeMs,
           avgReading.moisture, avgReading.temp, avgReading.ec, avgReading.ph,
           avgValidCount);
  f.close();
  Serial.println("Saved average to flash");
}

void printLogJSON() {
  File f = LittleFS.open(LOG_FILE, FILE_READ);
  if (!f) { Serial.println("[]"); return; }

  Serial.println("[");
  bool first = true;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (!first) Serial.println(",");
    Serial.print(line);
    first = false;
  }
  Serial.println("\n]");
  f.close();
}

void clearLog() {
  LittleFS.remove(LOG_FILE);
  batchCount = 0;
  Serial.println("Log cleared");
}

void restoreBatchCount() {
  File f = LittleFS.open(LOG_FILE, FILE_READ);
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    int i = line.indexOf("\"batch\":");
    if (i >= 0) {
      uint32_t b = line.substring(i + 8).toInt();
      if (b > batchCount) batchCount = b;
    }
  }
  f.close();
}

// ---------------- Readings ----------------
void takeReadings() {
  batchCount++;
  float sumM = 0, sumT = 0, sumEC = 0, sumPH = 0;
  uint8_t ok = 0;

  for (uint8_t i = 0; i < NUM_READINGS; i++) {
    SoilReading &r = readings[i];
    r.timeMs = millis();
    uint16_t m, t, e, p;

    bool gM = readReg(REG_MOIST, m); delay(50);
    bool gT = readReg(REG_TEMP,  t); delay(50);
    bool gE = readReg(REG_EC,    e); delay(50);
    bool gP = readReg(REG_PH,    p);
    r.valid = gM && gT && gE && gP;

    if (r.valid) {
      r.moisture = m / 10.0;
      r.temp     = (int16_t)t / 10.0;
      r.ec       = e;
      r.ph       = p / 10.0;

      sumM += r.moisture; sumT += r.temp; sumEC += r.ec; sumPH += r.ph; ok++;
      Serial.printf("#%u  Moist: %.1f %%  Temp: %.1f C  EC: %.0f uS/cm  pH: %.1f\n",
                    i + 1, r.moisture, r.temp, r.ec, r.ph);
    } else {
      r.moisture = r.temp = r.ec = r.ph = 0;
      Serial.printf("#%u  read failed\n", i + 1);
    }

    if (i < NUM_READINGS - 1) delay(READ_INTERVAL_MS);
  }

  avgReading.timeMs  = millis();
  avgValidCount      = ok;
  avgReading.valid   = ok > 0;
  avgReading.moisture = ok ? sumM  / ok : 0;
  avgReading.temp     = ok ? sumT  / ok : 0;
  avgReading.ec       = ok ? sumEC / ok : 0;
  avgReading.ph       = ok ? sumPH / ok : 0;

  Serial.printf("AVG  Moist: %.1f %%  Temp: %.1f C  EC: %.0f uS/cm  pH: %.1f (%u/%u valid)\n",
                avgReading.moisture, avgReading.temp, avgReading.ec, avgReading.ph,
                ok, NUM_READINGS);

  logAvgJSON();
  Serial.println("----");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD, SERIAL_8N1, RXD2, TXD2);
  delay(1000);

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
  restoreBatchCount();

  Serial.printf("Soil sensor ready (%lu batches logged)\n", batchCount);
  Serial.println("'1' = read, 'p' = print JSON log, 'c' = clear log");
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == '1') doRead = true;
    else if (ch == 'p') printLogJSON();
    else if (ch == 'c') clearLog();
  }

  if (doRead) {
    takeReadings();
    doRead = false;
  }
}