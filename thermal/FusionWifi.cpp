#include "FusionWifi.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <esp_wifi.h>
#include <cstring>

#include "FusionWifi_config.h"

#ifndef FUSION_WIFI_SERIAL_DEBUG
#define FUSION_WIFI_SERIAL_DEBUG 1
#endif

static WiFiMulti s_multi;
static bool s_wifi_ready_printed = false;
static bool s_first_tcp_ok_printed = false;

#ifndef FUSION_WIFI_QUEUE_DEPTH
#define FUSION_WIFI_QUEUE_DEPTH 12
#endif
#ifndef FUSION_WIFI_LINE_MAX
#define FUSION_WIFI_LINE_MAX 512
#endif

static QueueHandle_t s_queue;

static void scan_visible_aps_once() {
#if FUSION_WIFI_SERIAL_DEBUG
  Serial.println("[FusionWifi] Scanning nearby APs...");
  int n = WiFi.scanNetworks(/*async=*/false, /*show_hidden=*/false);
  if (n <= 0) {
    Serial.println("[FusionWifi] Scan result: no visible AP found.");
    return;
  }
  for (int i = 0; i < n; ++i) {
    Serial.printf("[FusionWifi] AP[%d] SSID=\"%s\" RSSI=%d ch=%d enc=%d\n",
                  i,
                  WiFi.SSID(i).c_str(),
                  WiFi.RSSI(i),
                  WiFi.channel(i),
                  (int)WiFi.encryptionType(i));
  }
  Serial.println("[FusionWifi] Scan done.");
#endif
}

static void ensure_wifi_connected() {
  uint32_t lastDiagMs = 0;
  uint32_t lastReconnectMs = 0;

  while (WiFi.status() != WL_CONNECTED) {
    // Visible SSID path only: trigger reconnect every ~6s.
    uint32_t now = millis();
    if (now - lastReconnectMs >= 6000) {
      lastReconnectMs = now;
      WiFi.disconnect(false, false);
      delay(30);
      WiFi.begin(STASSID, STAPSK);
    }
    delay(300);

#if FUSION_WIFI_SERIAL_DEBUG
    if (now - lastDiagMs >= 5000) {
      lastDiagMs = now;
      Serial.printf(
          "[FusionWifi] Still trying... WiFi.status=%d SSID=\"%s\" | "
          "visible-only reconnect active | 1=no AP, 4=wrong password, 6=disconnected\n",
          (int)WiFi.status(), STASSID);
    }
#endif
  }
}


static void connect_tcp_send_line(const char *line) {
  WiFiClient client;
  int retries = 80;
  while (!client.connect(FUSION_HOST, FUSION_TCP_PORT) && retries-- > 0) {
    if (WiFi.status() != WL_CONNECTED) {
      ensure_wifi_connected();
    }
    delay(15);
  }
  if (!client.connected()) {
#if FUSION_WIFI_SERIAL_DEBUG
    Serial.printf("[FusionWifi] TCP connect FAILED host=%s port=%d (check Mac IP & firewall)\n",
                  FUSION_HOST, (int)FUSION_TCP_PORT);
#endif
    return;
  }
  client.print(line);
  client.print("\n");
  client.setTimeout(30);
  (void)client.readStringUntil('\n');
  client.stop();
#if FUSION_WIFI_SERIAL_DEBUG
  if (!s_first_tcp_ok_printed) {
    s_first_tcp_ok_printed = true;
    Serial.println("[FusionWifi] First TCP send to fusion PC OK (JSON line delivered).");
  }
#endif
}

static void wifi_worker(void *param) {
  (void)param;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  scan_visible_aps_once();
  WiFi.begin(STASSID, STAPSK);

#if FUSION_WIFI_SERIAL_DEBUG
  Serial.printf(
      "[FusionWifi] Connecting to SSID=\"%s\" (2.4GHz; visible-only mode)...\n",
      STASSID);
#endif
  ensure_wifi_connected();

#if FUSION_WIFI_SERIAL_DEBUG
  if (!s_wifi_ready_printed) {
    s_wifi_ready_printed = true;
    Serial.println("[FusionWifi] ========== WiFi STA connected ==========");
    Serial.printf("[FusionWifi] SSID: %s\n", STASSID);
    Serial.printf("[FusionWifi] ESP32 IP: %s  (should be same subnet as %s)\n",
                  WiFi.localIP().toString().c_str(), FUSION_HOST);
    Serial.printf("[FusionWifi] Gateway: %s  RSSI: %d dBm\n",
                  WiFi.gatewayIP().toString().c_str(), WiFi.RSSI());
    Serial.println("[FusionWifi] Waiting for sensor events to send to fusion PC...");
    Serial.println("[FusionWifi] ==========================================");
  }
#endif

  for (;;) {
    char buf[FUSION_WIFI_LINE_MAX];
    if (xQueueReceive(s_queue, buf, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    connect_tcp_send_line(buf);
  }
}

bool fusion_wifi_init() {
  s_queue = xQueueCreate(FUSION_WIFI_QUEUE_DEPTH, FUSION_WIFI_LINE_MAX);
  if (s_queue == nullptr) {
    return false;
  }
  xTaskCreatePinnedToCore(wifi_worker, "fusion_wifi", 10240, nullptr, 1, nullptr, 0);
  return true;
}

bool fusion_wifi_send_json_line(const char *jsonLine) {
  if (s_queue == nullptr || jsonLine == nullptr) {
    return false;
  }
  char buf[FUSION_WIFI_LINE_MAX];
  strncpy(buf, jsonLine, FUSION_WIFI_LINE_MAX - 1);
  buf[FUSION_WIFI_LINE_MAX - 1] = '\0';
  return xQueueSend(s_queue, buf, pdMS_TO_TICKS(40)) == pdTRUE;
}
