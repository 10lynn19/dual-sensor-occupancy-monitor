# Dual-Sensor Room Occupancy Monitor

A real-time occupancy monitoring system that combines thermal imaging and infrared beam sensing to count people entering and leaving a room. Two ESP32 devices send events over Wi-Fi to a FastAPI service, which fuses the readings and updates a browser dashboard through WebSockets.

## Highlights

- Combines an MLX90640 thermal array with an E18-D80NK proximity sensor
- Tracks movement direction and supports closely spaced multi-person events
- Pairs sensor events within an approximately 850 ms fusion window
- Uses a lightweight TCP JSON protocol between ESP32 devices and the server
- Streams live occupancy updates to a responsive web dashboard
- Keeps sensor acquisition independent from networking through an ESP32 background task

## Architecture

```text
ESP32 + MLX90640 ── TCP/JSON ──┐
                               ├── FastAPI fusion server ── WebSocket ── Browser dashboard
ESP32 + E18-D80NK ─ TCP/JSON ──┘
```

- Port `8765`: HTTP API, dashboard, and WebSocket endpoint
- Port `8766`: newline-delimited TCP events from the ESP32 devices

Server receive time is used for event matching, so the two ESP32 devices do not need synchronized clocks.

## Repository Structure

```text
.
├── fusion_wifi/   # Shared ESP32 Wi-Fi/TCP library and configuration template
├── proximity/     # Infrared beam direction-detection firmware
├── thermal/       # MLX90640 detection and tracking firmware
└── web/           # FastAPI fusion service and browser dashboard
```

Shared networking code is stored only once in `fusion_wifi/`; the sensor folders contain only their device-specific firmware.

## Hardware

- 2 × ESP32 development boards
- 1 × MLX90640 thermal camera
- 1 × E18-D80NK infrared proximity sensor
- A computer running Python 3.10 or newer
- A shared 2.4 GHz Wi-Fi network

Use appropriate voltage protection when connecting a 5 V sensor output to a 3.3 V ESP32 GPIO.

## Run the Web Application

```bash
cd web
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python server.py
```

Open `http://127.0.0.1:8765/` on the host computer. Other devices on the same network can use `http://<host-ip>:8765/`.

## Configure the ESP32 Devices

1. Install `fusion_wifi/` as an Arduino library, or place it in your Arduino libraries directory.
2. Copy `fusion_wifi/FusionWifi_config.example.h` to `fusion_wifi/FusionWifi_config.h`.
3. Set `STASSID`, `STAPSK`, and `FUSION_HOST` in the new configuration file.
4. Install the `Adafruit MLX90640` Arduino library for the thermal device.
5. Flash `thermal/thermal.ino` and `proximity/proximity.ino` to their respective ESP32 boards.

`FusionWifi_config.h` is intentionally excluded from version control because it contains local network credentials.

## API

| Method | Endpoint | Purpose |
| --- | --- | --- |
| `GET` | `/` | Serve the live dashboard |
| `GET` | `/api/status` | Return occupancy and recent fusion state |
| `POST` | `/api/event` | Inject an event for testing or demonstrations |
| `POST` | `/api/reset` | Reset occupancy and pending events |
| WebSocket | `/ws` | Stream live updates and respond to ping/pong |

ESP32 events are sent to TCP port `8766` as one JSON object per line. Each event includes the sensor name, direction, estimated count change, confidence, event ID, and sensor-specific features.

## Fusion Strategy

The server pairs same-direction events from both sensors when they arrive within the fusion window. Unmatched events are committed according to single-sensor fallback rules. Thermal events can also be clustered over a short interval to handle people walking closely together.

The dashboard includes simulated event controls, making it possible to test the server and interface without connected hardware.

## Technologies

- ESP32 / Arduino C++
- MLX90640 thermal imaging
- FastAPI, Uvicorn, and Pydantic
- WebSockets and vanilla HTML/CSS/JavaScript

## Academic Context

Developed as a University of Michigan EECS 300 team project focused on embedded sensing, network communication, and multi-sensor data fusion.
