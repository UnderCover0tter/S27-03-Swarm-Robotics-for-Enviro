#include <LittleFS.h>

#define RXD2 16
#define TXD2 17

const uint32_t BAUD = 9600;
const uint8_t  ADDR = 0x01;

const uint16_t REG_N = 0x001E;
const uint16_t REG_P = 0x001F;
const uint16_t REG_K = 0x0020;

const uint8_t  NUM_READINGS = 5;
const uint32_t READ_INTERVAL_MS = 1000;   // time between readings in a batch
const char *LOG_FILE = "/npk_avg.jsonl";

volatile bool doRead = false;   // set true (or 1) to trigger a batch

struct NPKReading {
  uint32_t timeMs;
  uint16_t n, p, k;
  bool valid;
};

NPKReading readings[NUM_READINGS];   // latest batch (RAM)
NPKReading avgReading;              // average of latest batch (RAM)
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

  if (n < 7) return false;
  uint16_t rc = crc16(r, 5);
  if (r[1] != 0x03 || r[5] != (rc & 0xFF) || r[6] != (rc >> 8)) return false;

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

  f.printf("{\"batch\":%lu,\"time_ms\":%lu,\"n\":%u,\"p\":%u,\"k\":%u,\"valid\":%u}\n",
           batchCount, avgReading.timeMs,
           avgReading.n, avgReading.p, avgReading.k, avgValidCount);
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
  uint32_t sumN = 0, sumP = 0, sumK = 0;
  uint8_t ok = 0;

  for (uint8_t i = 0; i < NUM_READINGS; i++) {
    NPKReading &r = readings[i];
    r.timeMs = millis();

    bool gN = readReg(REG_N, r.n); delay(50);
    bool gP = readReg(REG_P, r.p); delay(50);
    bool gK = readReg(REG_K, r.k);
    r.valid = gN && gP && gK;

    if (r.valid) {
      sumN += r.n; sumP += r.p; sumK += r.k; ok++;
      Serial.printf("#%u  N: %u  P: %u  K: %u mg/kg\n", i + 1, r.n, r.p, r.k);
    } else {
      r.n = r.p = r.k = 0;
      Serial.printf("#%u  read failed\n", i + 1);
    }

    if (i < NUM_READINGS - 1) delay(READ_INTERVAL_MS);
  }

  avgReading.timeMs = millis();
  avgValidCount = ok;
  avgReading.valid = ok > 0;
  avgReading.n = ok ? (sumN + ok / 2) / ok : 0;   // rounded
  avgReading.p = ok ? (sumP + ok / 2) / ok : 0;
  avgReading.k = ok ? (sumK + ok / 2) / ok : 0;

  Serial.printf("AVG  N: %u  P: %u  K: %u mg/kg (%u/%u valid)\n",
                avgReading.n, avgReading.p, avgReading.k, ok, NUM_READINGS);

  logAvgJSON();
  Serial.println("----");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD, SERIAL_8N1, RXD2, TXD2);
  delay(1000);

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
  restoreBatchCount();

  Serial.printf("NPK ready (%lu batches logged)\n", batchCount);
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