# 双传感器房间人数融合系统（Thermal + Proximity）

基于两块 **ESP32**（一路 **MLX90640** 热成像、一路 **E18-D80NK** 对射式接近开关）与一台运行 **Python / FastAPI** 的笔记本：门边设备只负责检测与上报，**融合与网页仪表盘**在笔记本上完成。

---

## 仓库结构

| 路径 | 说明 |
|------|------|
| **`thermal/`** | 热成像固件：`thermal.ino` + `FusionWifi.*` + 配置示例 |
| **`proximity/`** | 对射接近开关固件：`proximity.ino` + `FusionWifi.*` + 配置示例 |
| **`web/`** | 融合服务与前端：`server.py`、`static/`（HTML/CSS/JS）、`requirements.txt` |
| **`fusion_wifi/`** | 可复用的 **FusionWifi** 库源码与 **`README.txt`**（联调、热点、防火墙、端口说明更全） |

---

## 系统框图（逻辑）

```
[ESP32 + MLX90640] ──Wi-Fi TCP JSON──┐
                                    ├──► [笔记本: server.py :8765/:8766] ──► 浏览器仪表盘
[ESP32 + 接近开关] ──Wi-Fi TCP JSON──┘
```

- **8765**：HTTP（页面、`/api/*`）+ **WebSocket** `/ws`  
- **8766**：ESP32 **TCP** 上报，**每连接一行 JSON**（换行结束），与 `server.py` 中 `FUSION_TCP_PORT` 一致  

融合时间戳使用 **笔记本收到 JSON 的时刻**（`ts_ms` 由服务器写入），两块板无需 NTP 对齐。

---

## 硬件与固件前提

- 两块 **ESP32** 开发板；分别烧录 `thermal/` 与 `proximity/` 工程。  
- **MLX90640** 经 **I2C** 接热成像那块板；**E18-D80NK** 按你们原理图接另一块板（注意 **5 V → 3.3 V GPIO** 分压等保护措施）。  
- 笔记本与两块板处于 **同一局域网**（课程演示推荐 **手机 2.4 GHz 热点**：ESP32 不支持 5 GHz-only）。

---

## 笔记本端：环境与启动

**Python**：建议 3.10+（开发时使用 3.12 亦可）。

```bash
cd web
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python server.py
```

默认通过 **uvicorn** 监听 **`0.0.0.0:8765`**（必须绑定 `0.0.0.0`，手机等同网段设备才能访问）。启动后控制台会提示 **TCP 8766** 用于 ESP32。

本机浏览器打开：`http://127.0.0.1:8765/`  
同热点下其它设备：`http://<笔记本在该 Wi-Fi 下的 IPv4>:8765/`

---

## ESP32：配置与烧录

1. 每个 Arduino sketch 目录已包含所需的 **`FusionWifi.h` / `FusionWifi.cpp`**。  
2. 在 `thermal/` 与 `proximity/` 中分别复制 **`FusionWifi_config.example.h`** 为 **`FusionWifi_config.h`**，并修改：  
   - **`STASSID` / `STAPSK`**：Wi-Fi 名称与密码  
   - **`FUSION_HOST`**：笔记本在 **同一网段** 的 IPv4（不是手机 IP）  
3. 分别打开 **`thermal.ino`**、**`proximity.ino`** 烧录到对应板子。  
4. 串口监视器：确认出现 Wi-Fi 已连接日志；触发一次门事件后应出现 **TCP 发往融合机成功** 一类日志（详见 `fusion_wifi/README.txt`）。

---

## 主要 HTTP / WebSocket 接口（便于二次开发）

| 方法 | 路径 | 作用 |
|------|------|------|
| `GET` | `/` | 仪表盘页面 |
| `GET` | `/api/status` | 当前人数、最近融合记录、pending 状态等 |
| `POST` | `/api/event` | 注入 JSON 事件（演示/脚本；**非** ESP32 默认路径） |
| `POST` | `/api/reset` | 清零人数并清空 pending 队列 |
| `WebSocket` | `/ws` | 推送 `update` 消息；客户端可发 `ping` 收 `pong` |

ESP32 默认路径：**TCP `FUSION_HOST:8766`**，正文为 **单行 JSON**（字段与 `server.py` 中 `SensorEventIn` 一致：`sensor`, `direction`, `delta_est`, `confidence_raw`, `event_id`, `features` 等）。

---

## 融合逻辑（概要）

实现位于 **`web/server.py`**：

- 在约 **850 ms** 的窗口内尝试 **配对** 两路同向事件；超时则按规则做 **单路提交**。  
- 支持 **热释侧短时同向多次事件聚簇**（有上限），用于多人紧挨通过等场景。  
- 详细行为以源码与注释为准；论文中对应 **Sensor Fusion** 与 **System Communication** 小节。

---

## 联调与故障排查

优先阅读 **`fusion_wifi/README.txt`**，其中包含：

- 手机热点与 **2.4 GHz** 注意点  
- 如何查笔记本 IP、填写 **`FUSION_HOST`**  
- **macOS / Windows 防火墙** 放行 **8765、8766**  
- 串口上如何区分「仅 Wi-Fi 连上」与「已成功发到 `server.py`」

常见问题：

- **网页人数一直为 0**：ESP 未连上 TCP 8765/8766 或防火墙拦截；或仅用浏览器未触发真实事件——可用页面上的 **Simulate events**（走 HTTP）验证服务器与前端。  
- **`FUSION_HOST` 填错**：换热点子网后笔记本 IP 会变，需同步更新两块板的配置并重新烧录（或改为你们若实现的其它配置方式）。

---

## 依赖（笔记本）

见 **`web/requirements.txt`**，主要包括：

- `fastapi`  
- `uvicorn[standard]`  
- `pydantic`

---

## 许可证与课程说明

本项目为 **EECS 300（或同等）课程团队作业**。源码随课程要求提交至 Google Drive；使用或二次开发请注明课程与团队信息。

**Team 24** — *Room occupancy monitoring (thermal + proximity hybrid)*
