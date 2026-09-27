#include <cstdio>
#include "FusionWifi.h"

const int sensorA = 36;
const int sensorB = 39;

/** Net occupancy from beam sequences (in − out). */
int count = 0;
static uint32_t fusionEventId = 0;
static uint32_t totalPassIn = 0;
static uint32_t totalPassOut = 0;

static void proximity_send_fusion_event(bool entering) {
  fusionEventId++;
  const char *dir = entering ? "in" : "out";
  char buf[320];
  snprintf(buf, sizeof(buf),
           "{\"sensor\":\"proximity\",\"event_id\":%lu,\"direction\":\"%s\",\"delta_est\":1,"
           "\"features\":{\"seq_ok\":1},\"confidence_raw\":0.82}",
           (unsigned long)fusionEventId, dir);
  fusion_wifi_send_json_line(buf);
}

// debounce
byte lastRawState = 0;
byte stableState = 0;
unsigned long lastDebounceTime = 0;
// Slower walking can cause beam jitter; use a slightly longer debounce.
const unsigned long debounceDelay = 20;  // ms

// full-sequence tracking
enum SequenceState {
  IDLE,
  ENTER_1,   // 00 -> 10
  ENTER_2,   // 00 -> 10 -> 11
  ENTER_3,   // 00 -> 10 -> 11 -> 01

  EXIT_1,    // 00 -> 01
  EXIT_2,    // 00 -> 01 -> 11
  EXIT_3     // 00 -> 01 -> 11 -> 10
};

SequenceState seqState = IDLE;

void setup() {
  pinMode(sensorA, INPUT_PULLUP);
  pinMode(sensorB, INPUT_PULLUP);
  // Use 115200 — same as most ESP32 boot logs & thermal.ino. Set Serial Monitor to 115200 baud.
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== proximity sketch started ===");
  Serial.println("[PROX] Serial 115200 | net=beam net occupancy; total_IN/OUT=crossing counts");
  Serial.flush();

  if (fusion_wifi_init()) {
    Serial.println("Fusion Wi-Fi task created (watch for [FusionWifi] lines).");
  } else {
    Serial.println("Fusion Wi-Fi init FAILED (queue alloc).");
  }

  lastRawState = readState();
  stableState = lastRawState;
}

void loop() {
  byte rawState = readState();

  // -----------------------------
  // Debounce filter
  // -----------------------------
  if (rawState != lastRawState) {
    lastDebounceTime = millis();
    lastRawState = rawState;
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (stableState != rawState) {
      stableState = rawState;
      processState(stableState);
    }
  }
}

// Convert sensor readings into 2-bit state
byte readState() {
  byte A = (digitalRead(sensorA) == LOW);  // active LOW
  byte B = (digitalRead(sensorB) == LOW);
  return (A << 1) | B;
}

// Process only stable debounced states
void processState(byte s) {

  switch (seqState) {

    case IDLE:
      if (s == 0b10) {
        seqState = ENTER_1;
      }
      else if (s == 0b01) {
        seqState = EXIT_1;
      }
      break;

    // -------------------------
    // ENTER sequence: 00->10->11->01->00
    // -------------------------
    case ENTER_1:
      if (s == 0b11) {
        seqState = ENTER_2;
      }
      else if (s == 0b00) {
        seqState = IDLE;   // backed out
      }
      else {
        seqState = IDLE;   // invalid
      }
      break;

    case ENTER_2:
      if (s == 0b11) {
        // stay here if person stands in doorway
      }
      else if (s == 0b01) {
        seqState = ENTER_3;
      }
      else if (s == 0b10) {
        // allow small backward jitter at slow speed
        seqState = ENTER_1;
      }
      else {
        seqState = IDLE;   // invalid
      }
      break;

    case ENTER_3:
      if (s == 0b00) {
        count++;
        totalPassIn++;
        Serial.printf(
            "[PROX] ENTER (into room) | net_occupancy=%d | total_IN=%lu total_OUT=%lu | t=%lu ms\n",
            count,
            (unsigned long)totalPassIn,
            (unsigned long)totalPassOut,
            (unsigned long)millis());
        proximity_send_fusion_event(true);
        seqState = IDLE;
      }
      else if (s == 0b01) {
        // stay here if person pauses here
      }
      else if (s == 0b11) {
        // allow doorway oscillation before fully clearing
        seqState = ENTER_2;
      }
      else {
        seqState = IDLE;   // invalid
      }
      break;

    // -------------------------
    // EXIT sequence: 00->01->11->10->00
    // -------------------------
    case EXIT_1:
      if (s == 0b11) {
        seqState = EXIT_2;
      }
      else if (s == 0b00) {
        seqState = IDLE;   // backed out
      }
      else {
        seqState = IDLE;   // invalid
      }
      break;

    case EXIT_2:
      if (s == 0b11) {
        // stay here if person stands in doorway
      }
      else if (s == 0b10) {
        seqState = EXIT_3;
      }
      else if (s == 0b01) {
        // allow small backward jitter at slow speed
        seqState = EXIT_1;
      }
      else {
        seqState = IDLE;   // invalid
      }
      break;

    case EXIT_3:
      if (s == 0b00) {
        count--;
        totalPassOut++;
        Serial.printf(
            "[PROX] EXIT (leave room) | net_occupancy=%d | total_IN=%lu total_OUT=%lu | t=%lu ms\n",
            count,
            (unsigned long)totalPassIn,
            (unsigned long)totalPassOut,
            (unsigned long)millis());
        proximity_send_fusion_event(false);
        seqState = IDLE;
      }
      else if (s == 0b10) {
        // stay here if person pauses here
      }
      else if (s == 0b11) {
        // allow doorway oscillation before fully clearing
        seqState = EXIT_2;
      }
      else {
        seqState = IDLE;   // invalid
      }
      break;
  }
}