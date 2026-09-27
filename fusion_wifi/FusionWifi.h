#pragma once

#include <Arduino.h>

/**
 * Lab07-style WiFi: STA + TCP line protocol to fusion PC (one JSON line per event, \\n terminated).
 * Runs a background task (separate from loop) so sensor timing stays stable — same idea as WirelessCommunication.cpp.
 */

bool fusion_wifi_init();

/** Enqueue one complete JSON object as a single line (no embedded newlines). Returns false if queue full. */
bool fusion_wifi_send_json_line(const char *jsonLine);
