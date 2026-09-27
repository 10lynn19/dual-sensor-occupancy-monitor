#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <cstdio>

#include "FusionWifi.h"

/*
  ESP32 MLX90640 People Counter - Memory Optimized

  Core logic kept the same as the user's Python version:
  - threshold at HEAD_TEMP_THRESHOLD
  - connected-component blob detection (8-neighbor)
  - blob size < MIN_HEAD_SIZE => ignore
  - blob size < MULTI_PERSON_THRESHOLD => 1 person
  - blob size >= MULTI_PERSON_THRESHOLD => split into 2 people horizontally
  - nearest-neighbor tracking with MATCHING_DISTANCE
  - count ENTER when prev_x >= DOOR_LEFT_EDGE and curr_x < DOOR_LEFT_EDGE
  - count EXIT  when prev_x <= DOOR_RIGHT_EDGE and curr_x > DOOR_RIGHT_EDGE
  - remove trajectory after counting

  Non-essential PC-side features removed.
  Added:
  - [CENTER] output for detected centers
  - [FPS] output every 10 seconds
  - serial 's' to pause/resume processing

  Fusion project: optional FusionWifi sends one JSON line per ENTER/EXIT (detection logic unchanged).
*/

// ==================== SENSOR CONFIG ====================
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;
static const uint32_t I2C_CLOCK_HZ = 800000;
static const uint32_t SERIAL_BAUD = 115200;

Adafruit_MLX90640 mlx;

// ==================== CORE CONFIG (kept from Python) ====================
static const int ROWS = 24;
static const int COLS = 32;
static const int PIXELS = ROWS * COLS;

static const float HEAD_TEMP_THRESHOLD = 25.5f;
static const int MIN_HEAD_SIZE = 13;
static const int MAX_HEAD_SIZE = 300;           // kept as config; not used by original logic either
static const int SINGLE_PERSON_SIZE = 100;      // kept as config; not used by original logic either
static const int MULTI_PERSON_THRESHOLD = 90;
static const int MAX_HEADS = 10;

static const int DOOR_LEFT_EDGE = 11;
static const int DOOR_RIGHT_EDGE = 21;
static const int DOOR_CENTER = 16;
static const int TRACKING_TIMEOUT = 10;
static const int MIN_TRACKING_FRAMES = 3;
static const float MATCHING_DISTANCE = 30.0f;
static const float MAX_X_JUMP_PER_FRAME = 8.0f;
static const float MAX_Y_JUMP_PER_FRAME = 8.0f;
static const int HISTORY_LEN = 5;
static const int MAX_TRACKS = 16;

static const uint32_t FPS_REPORT_INTERVAL_MS = 10000;

static uint32_t fusionEventId = 0;

// ==================== DATA TYPES ====================
struct Point2f {
  float x;
  float y;
};

struct DetectedBody {
  bool valid;
  Point2f pos;
  float temp;
  uint16_t size;
};

struct Track {
  bool active;
  int id;
  Point2f pos;
  Point2f history[HISTORY_LEN];
  uint8_t historyLen;
  uint8_t framesSinceSeen;
  int firstFrame;
  float temp;
  /** Last matched detection blob size (pixels); for Serial debug only — does not change tracking math. */
  uint16_t blobSize;
};

struct MatchCandidate {
  int trackIdx;
  int detIdx;
  float dist;
};

// ==================== GLOBAL MEMORY-OPTIMIZED BUFFERS ====================
static float frameData[PIXELS];               // 768 * 4 = 3072 B
static uint8_t binaryMask[PIXELS];            // 768 B
static uint8_t visited[PIXELS];               // 768 B
static uint16_t bfsQueue[PIXELS];             // 1536 B, stores flat pixel index
static uint16_t blobPixels[PIXELS];           // 1536 B, reused for one blob at a time

static DetectedBody detectedHeads[MAX_HEADS];
static int detectedCount = 0;

static Track trackedHeads[MAX_TRACKS];
static int nextHeadId = 0;

static int frameCount = 0;
static int peopleInside = 0;
static int totalEntered = 0;
static int totalExited = 0;
static int sensorErrorCount = 0;

static bool processingEnabled = true;
static uint32_t fpsWindowStartMs = 0;
static uint32_t fpsWindowFrames = 0;

// ==================== HELPERS ====================
static inline int idxRC(int r, int c) {
  return r * COLS + c;
}

static inline int rowFromIdx(int idx) {
  return idx / COLS;
}

static inline int colFromIdx(int idx) {
  return idx % COLS;
}

static inline float sqrf(float x) {
  return x * x;
}

static inline float pointDistance(const Point2f &a, const Point2f &b) {
  return sqrtf(sqrf(a.x - b.x) + sqrf(a.y - b.y));
}

void pushHistory(Track &t, const Point2f &p) {
  if (t.historyLen < HISTORY_LEN) {
    t.history[t.historyLen++] = p;
  } else {
    for (int i = 1; i < HISTORY_LEN; ++i) {
      t.history[i - 1] = t.history[i];
    }
    t.history[HISTORY_LEN - 1] = p;
  }
}

void clearDetectedHeads() {
  detectedCount = 0;
  for (int i = 0; i < MAX_HEADS; ++i) {
    detectedHeads[i].valid = false;
  }
}

void addDetectedHead(float cx, float cy, float temp, uint16_t size) {
  if (detectedCount >= MAX_HEADS) return;
  detectedHeads[detectedCount].valid = true;
  detectedHeads[detectedCount].pos = {cx, cy};
  detectedHeads[detectedCount].temp = temp;
  detectedHeads[detectedCount].size = size;
  detectedCount++;
}

int estimate_people_count(int size) {
  if (size < MIN_HEAD_SIZE) return 0;
  if (size < MULTI_PERSON_THRESHOLD) return 1;
  return 2;
}

// ==================== FUSION Wi-Fi (TCP JSON line, lab-style transport) ====================
static void thermal_send_fusion_event(bool isEnter, float prevX, float currX) {
  fusionEventId++;
  const char *dir = isEnter ? "in" : "out";
  char buf[384];
  snprintf(buf, sizeof(buf),
           "{\"sensor\":\"thermal\",\"event_id\":%lu,\"direction\":\"%s\",\"delta_est\":1,"
           "\"features\":{\"prev_x\":%.2f,\"curr_x\":%.2f},\"confidence_raw\":0.72}",
           (unsigned long)fusionEventId, dir, prevX, currX);
  fusion_wifi_send_json_line(buf);
}

// ==================== SERIAL COMMANDS ====================
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == 's' || ch == 'S') {
      processingEnabled = !processingEnabled;
      fpsWindowStartMs = millis();
      fpsWindowFrames = 0;
      Serial.println(processingEnabled ? "[STATE] processing resumed" : "[STATE] processing paused");
    }
  }
}

// ==================== SENSOR ====================
bool readMLX90640Frame() {
  int status = mlx.getFrame(frameData);
  if (status != 0) {
    sensorErrorCount++;
    if ((sensorErrorCount % 20) == 0) {
      Serial.printf("[WARN] MLX frame read errors: %d\n", sensorErrorCount);
    }
    return false;
  }

  if (sensorErrorCount > 0) sensorErrorCount = 0;
  return true;
}

void initSensor() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  int retryCount = 0;
  while (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Sensor initialization failed, retrying...");
    delay(500);
    retryCount++;
    if (retryCount > 5) {
      Serial.println("ERROR: Cannot connect to MLX90640");
      Serial.println("Check SDA->21, SCL->22, VIN->3.3V, GND->GND");
      while (true) delay(1000);
    }
  }

  mlx.setMode(MLX90640_CHESS);
  mlx.setRefreshRate(MLX90640_16_HZ);

  Serial.println("SUCCESS: Sensor initialized");
  Serial.println("Mode: Chess pattern");
  Serial.println("Refresh rate setting: 16 Hz");
  Serial.println("Serial: 115200");
}

// ==================== DETECTION ====================
void buildBinaryMask() {
  for (int i = 0; i < PIXELS; ++i) {
    binaryMask[i] = (frameData[i] > HEAD_TEMP_THRESHOLD) ? 1 : 0;
    visited[i] = 0;
  }
}

uint16_t collectBlobBFS(int startIdx) {
  uint16_t qHead = 0;
  uint16_t qTail = 0;
  uint16_t blobCount = 0;

  bfsQueue[qTail++] = (uint16_t)startIdx;
  visited[startIdx] = 1;

  while (qHead < qTail) {
    uint16_t idx = bfsQueue[qHead++];
    blobPixels[blobCount++] = idx;

    int r = rowFromIdx(idx);
    int c = colFromIdx(idx);

    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) continue;
        int nr = r + dr;
        int nc = c + dc;
        if (nr < 0 || nr >= ROWS || nc < 0 || nc >= COLS) continue;

        int nidx = idxRC(nr, nc);
        if (visited[nidx]) continue;
        if (!binaryMask[nidx]) continue;

        visited[nidx] = 1;
        bfsQueue[qTail++] = (uint16_t)nidx;
      }
    }
  }

  return blobCount;
}

void split_blob_into_people(uint16_t blobCount, int numPeople) {
  if (blobCount == 0 || numPeople <= 0) return;

  int xMin = 9999;
  int xMax = -9999;
  float maxTemp = -1000.0f;

  for (uint16_t i = 0; i < blobCount; ++i) {
    int idx = blobPixels[i];
    int x = colFromIdx(idx);
    if (x < xMin) xMin = x;
    if (x > xMax) xMax = x;
    if (frameData[idx] > maxTemp) maxTemp = frameData[idx];
  }

  if (numPeople == 1) {
    float sumX = 0.0f;
    float sumY = 0.0f;
    for (uint16_t i = 0; i < blobCount; ++i) {
      int idx = blobPixels[i];
      sumX += colFromIdx(idx);
      sumY += rowFromIdx(idx);
    }
    addDetectedHead(sumX / blobCount, sumY / blobCount, maxTemp, blobCount);
    return;
  }

  float segmentWidth = float(xMax - xMin + 1) / float(numPeople);

  for (int seg = 0; seg < numPeople; ++seg) {
    float segMin = xMin + seg * segmentWidth;
    float segMax = xMin + (seg + 1) * segmentWidth;

    float sumX = 0.0f;
    float sumY = 0.0f;
    uint16_t count = 0;

    for (uint16_t i = 0; i < blobCount; ++i) {
      int idx = blobPixels[i];
      float x = (float)colFromIdx(idx);
      if (x >= segMin && x < segMax) {
        sumX += x;
        sumY += rowFromIdx(idx);
        count++;
      }
    }

    if (count > 0) {
      addDetectedHead(sumX / count, sumY / count, maxTemp, blobCount / numPeople);
    }
  }
}

void detect_heads() {
  clearDetectedHeads();
  buildBinaryMask();

  for (int idx = 0; idx < PIXELS; ++idx) {
    if (!binaryMask[idx] || visited[idx]) continue;

    uint16_t size = collectBlobBFS(idx);
    if (size < MIN_HEAD_SIZE) continue;

    int numPeople = estimate_people_count(size);
    if (numPeople == 0) continue;

    split_blob_into_people(size, numPeople);
    if (detectedCount >= MAX_HEADS) return;
  }
}

void printDetectedCenters() {
  Serial.printf("[CENTER] detected=%d", detectedCount);
  for (int i = 0; i < detectedCount; ++i) {
    if (!detectedHeads[i].valid) continue;
    Serial.printf(" | P%d x=%.2f y=%.2f size=%u",
                  i,
                  detectedHeads[i].pos.x,
                  detectedHeads[i].pos.y,
                  detectedHeads[i].size);
  }
  Serial.println();
}

// ==================== TRACKING ====================
void resetTracks() {
  for (int i = 0; i < MAX_TRACKS; ++i) {
    trackedHeads[i].active = false;
    trackedHeads[i].id = -1;
    trackedHeads[i].historyLen = 0;
    trackedHeads[i].framesSinceSeen = 0;
  }
}

int createTrack(const DetectedBody &body) {
  for (int i = 0; i < MAX_TRACKS; ++i) {
    if (!trackedHeads[i].active) {
      trackedHeads[i].active = true;
      trackedHeads[i].id = nextHeadId++;
      trackedHeads[i].pos = body.pos;
      trackedHeads[i].historyLen = 0;
      pushHistory(trackedHeads[i], body.pos);
      trackedHeads[i].framesSinceSeen = 0;
      trackedHeads[i].firstFrame = frameCount;
      trackedHeads[i].temp = body.temp;
      trackedHeads[i].blobSize = body.size;
      return i;
    }
  }
  return -1;
}

void removeTrack(int idx) {
  trackedHeads[idx].active = false;
  trackedHeads[idx].id = -1;
  trackedHeads[idx].historyLen = 0;
  trackedHeads[idx].framesSinceSeen = 0;
}

void track_and_count() {
  bool trackMatched[MAX_TRACKS];
  bool detMatched[MAX_HEADS];
  for (int i = 0; i < MAX_TRACKS; ++i) trackMatched[i] = false;
  for (int i = 0; i < MAX_HEADS; ++i) detMatched[i] = false;

  MatchCandidate candidates[MAX_TRACKS * MAX_HEADS];
  int candidateCount = 0;

  for (int t = 0; t < MAX_TRACKS; ++t) {
    if (!trackedHeads[t].active) continue;

    for (int h = 0; h < detectedCount; ++h) {
      if (!detectedHeads[h].valid) continue;

      float dx = detectedHeads[h].pos.x - trackedHeads[t].pos.x;
      float dy = detectedHeads[h].pos.y - trackedHeads[t].pos.y;
      float adx = fabsf(dx);
      float ady = fabsf(dy);
      if (adx > MAX_X_JUMP_PER_FRAME) continue;
      if (ady > MAX_Y_JUMP_PER_FRAME) continue;

      float distance = sqrtf(dx * dx + dy * dy);
      if (distance < MATCHING_DISTANCE) {
        candidates[candidateCount].trackIdx = t;
        candidates[candidateCount].detIdx = h;
        candidates[candidateCount].dist = distance;
        candidateCount++;
      }
    }
  }

  for (int i = 0; i < candidateCount - 1; ++i) {
    for (int j = i + 1; j < candidateCount; ++j) {
      if (candidates[j].dist < candidates[i].dist) {
        MatchCandidate tmp = candidates[i];
        candidates[i] = candidates[j];
        candidates[j] = tmp;
      }
    }
  }

  for (int i = 0; i < candidateCount; ++i) {
    int t = candidates[i].trackIdx;
    int h = candidates[i].detIdx;
    if (trackMatched[t] || detMatched[h]) continue;

    trackedHeads[t].pos = detectedHeads[h].pos;
    pushHistory(trackedHeads[t], detectedHeads[h].pos);
    trackedHeads[t].framesSinceSeen = 0;
    trackedHeads[t].temp = detectedHeads[h].temp;
    trackedHeads[t].blobSize = detectedHeads[h].size;
    trackMatched[t] = true;
    detMatched[h] = true;
  }

  for (int h = 0; h < detectedCount; ++h) {
    if (!detectedHeads[h].valid) continue;
    if (detMatched[h]) continue;
    createTrack(detectedHeads[h]);
  }

  int toRemove[MAX_TRACKS];
  int removeCount = 0;

  for (int t = 0; t < MAX_TRACKS; ++t) {
    if (!trackedHeads[t].active) continue;

    bool counted = false;
    int framesTracked = frameCount - trackedHeads[t].firstFrame;

    if (trackedHeads[t].historyLen >= 2 && framesTracked >= MIN_TRACKING_FRAMES) {
      float prevX = trackedHeads[t].history[trackedHeads[t].historyLen - 2].x;
      float currX = trackedHeads[t].history[trackedHeads[t].historyLen - 1].x;

      if (prevX >= DOOR_LEFT_EDGE && currX < DOOR_LEFT_EDGE) {
        peopleInside++;
        totalEntered++;
        Serial.printf("[ENTER] Crossed X=%d (%.1f->%.1f) Inside: %d\n",
                      DOOR_LEFT_EDGE, prevX, currX, peopleInside);
        Serial.printf(
            "[THERMAL_DBG] track_id=%d blob_pixels=%u peak_temp_C=%.2f est_people=%d | "
            "door_x %.2f->%.2f (left_edge=%d) | track_frames=%d | "
            "total_in=%d total_out=%d inside_now=%d | t=%lu ms\n",
            trackedHeads[t].id,
            trackedHeads[t].blobSize,
            trackedHeads[t].temp,
            estimate_people_count((int)trackedHeads[t].blobSize),
            prevX,
            currX,
            DOOR_LEFT_EDGE,
            framesTracked,
            totalEntered,
            totalExited,
            peopleInside,
            (unsigned long)millis());
        thermal_send_fusion_event(true, prevX, currX);
        counted = true;
      } else if (prevX <= DOOR_RIGHT_EDGE && currX > DOOR_RIGHT_EDGE) {
        if (peopleInside > 0) peopleInside--;
        totalExited++;
        Serial.printf("[EXIT] Crossed X=%d (%.1f->%.1f) Inside: %d\n",
                      DOOR_RIGHT_EDGE, prevX, currX, peopleInside);
        Serial.printf(
            "[THERMAL_DBG] track_id=%d blob_pixels=%u peak_temp_C=%.2f est_people=%d | "
            "door_x %.2f->%.2f (right_edge=%d) | track_frames=%d | "
            "total_in=%d total_out=%d inside_now=%d | t=%lu ms\n",
            trackedHeads[t].id,
            trackedHeads[t].blobSize,
            trackedHeads[t].temp,
            estimate_people_count((int)trackedHeads[t].blobSize),
            prevX,
            currX,
            DOOR_RIGHT_EDGE,
            framesTracked,
            totalEntered,
            totalExited,
            peopleInside,
            (unsigned long)millis());
        thermal_send_fusion_event(false, prevX, currX);
        counted = true;
      }
    }

    if (counted) {
      toRemove[removeCount++] = t;
      continue;
    }

    if (!trackMatched[t]) {
      trackedHeads[t].framesSinceSeen++;
      if (trackedHeads[t].framesSinceSeen >= TRACKING_TIMEOUT) {
        toRemove[removeCount++] = t;
      }
    }
  }

  for (int i = 0; i < removeCount; ++i) {
    removeTrack(toRemove[i]);
  }
}

// ==================== FPS ====================
void reportFPSIfNeeded() {
  uint32_t now = millis();
  if (now - fpsWindowStartMs >= FPS_REPORT_INTERVAL_MS) {
    float seconds = float(now - fpsWindowStartMs) / 1000.0f;
    float fps = (seconds > 0.0f) ? (float(fpsWindowFrames) / seconds) : 0.0f;
    Serial.printf("[FPS] %.2f frames/s over last %.1f s\n", fps, seconds);
    fpsWindowStartMs = now;
    fpsWindowFrames = 0;
  }
}

void printStartupInfo() {
  Serial.println();
  Serial.println("================================================");
  Serial.println("ESP32 MLX90640 People Counter - Memory Optimized");
  Serial.println("================================================");
  Serial.printf("Threshold: %.2f C\n", HEAD_TEMP_THRESHOLD);
  Serial.printf("MIN_HEAD_SIZE: %d\n", MIN_HEAD_SIZE);
  Serial.printf("MULTI_PERSON_THRESHOLD: %d\n", MULTI_PERSON_THRESHOLD);
  Serial.printf("Door zone: left=%d center=%d right=%d\n", DOOR_LEFT_EDGE, DOOR_CENTER, DOOR_RIGHT_EDGE);
  Serial.printf("Match gate: dist<%.1f, |dx|<=%.1f, |dy|<=%.1f\n", MATCHING_DISTANCE, MAX_X_JUMP_PER_FRAME, MAX_Y_JUMP_PER_FRAME);
  Serial.println("Commands:");
  Serial.println("  s : pause/resume processing");
  Serial.println("Output:");
  Serial.println("  [CENTER] detected centers");
  Serial.println("  [ENTER]/[EXIT] crossing + [THERMAL_DBG] detail (blob_px, temp, totals)");
  Serial.println("  [FPS] every 10 seconds");
  Serial.println("================================================");
}

// ==================== SETUP / LOOP ====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(1);
  delay(300);

  resetTracks();
  fpsWindowStartMs = millis();
  fpsWindowFrames = 0;

  initSensor();
  if (fusion_wifi_init()) {
    Serial.println("Fusion Wi-Fi task started (TCP JSON to PC; ts_ms set on server).");
  } else {
    Serial.println("[WARN] Fusion Wi-Fi init failed — serial only.");
  }
  printStartupInfo();
}

void loop() {
  handleSerialCommands();

  if (!processingEnabled) {
    delay(10);
    return;
  }

  if (readMLX90640Frame()) {
    frameCount++;
    fpsWindowFrames++;

    detect_heads();
    printDetectedCenters();
    track_and_count();
  }

  reportFPSIfNeeded();
}