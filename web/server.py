"""
Fusion node for dual-sensor room occupancy (WiFi HTTP + WebSocket dashboard).

- ESP32 (lab-style): one JSON line per TCP connection to FUSION_TCP_PORT, newline-terminated
  (same spirit as lab07 client → server strings).
- Optional HTTP: POST /api/event
- Fusion timestamps: always server receive time (int ms since Unix epoch), not ESP millis.
"""

from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from collections import deque
from contextlib import asynccontextmanager
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Literal

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field

STATIC_DIR = Path(__file__).resolve().parent / "static"

SensorName = Literal["proximity", "thermal"]
Direction = Literal["in", "out", "unknown"]


class SensorEventIn(BaseModel):
    model_config = ConfigDict(extra="ignore")

    sensor: SensorName
    event_id: int = Field(..., description="Monotonic per device")
    ts_ms: int | None = Field(
        default=None,
        description="Ignored for fusion; server sets receive time on ingest.",
    )
    direction: Direction
    delta_est: int = Field(1, ge=0, le=8)
    features: dict[str, Any] = Field(default_factory=dict)
    confidence_raw: float = Field(0.5, ge=0.0, le=1.0)


@dataclass
class FusedRecord:
    ts_ms: int
    fused_delta: int
    direction: Direction
    confidence_fused: float
    reason: str
    prox_snapshot: dict[str, Any] | None
    therm_snapshot: dict[str, Any] | None
    needs_review: bool = False
    paired: bool = False


@dataclass
class ServerState:
    room_count: int = 0
    last_proximity: dict[str, Any] | None = None
    last_thermal: dict[str, Any] | None = None
    recent_fused: deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=80))
    connected_clients: int = 0
    # Deferred fusion: wait for the other sensor before counting (see DEFER_WINDOW_MS).
    pending_proximity: dict[str, Any] | None = None
    pending_prox_deadline_ms: int = 0
    # When proximity arrives first, accumulate one or more matching thermal events before commit.
    pending_prox_thermal: dict[str, Any] | None = None
    pending_prox_thermal_cluster_n: int = 0
    pending_thermal: dict[str, Any] | None = None
    pending_thermal_deadline_ms: int = 0
    # Multiple thermal lines (same direction) before proximity: coalesce (cap THERMAL_CLUSTER_CAP).
    pending_thermal_cluster_n: int = 1
    pending_thermal_cluster_start_ms: int = 0
    suppress_thermal_only_until_ms: int = 0


state = ServerState()
# Recent raw events for pairing (ts_ms, payload dict)
_prox_q: deque[tuple[int, dict[str, Any]]] = deque(maxlen=32)
_therm_q: deque[tuple[int, dict[str, Any]]] = deque(maxlen=32)

FUSION_WINDOW_MS = 800
# After proximity (or thermal) alone, wait this long for the other sensor, then commit single-sensor.
DEFER_WINDOW_MS = 850
# After paired fusion: suppress extra thermal lines. Use 0 when using thermal clusters (multi-person),
# otherwise 650 ms can drop the 2nd person's thermal.
THERMAL_DEDUP_AFTER_PAIR_MS = 0
THERMAL_CLUSTER_CAP = 4
# Lab-style TCP: one JSON object per line (ESP32 WiFiClient), same newline framing as lab07.
FUSION_TCP_PORT = 8766

_pending_state_lock = threading.Lock()


def _log_sensor_json(payload: dict[str, Any], source: str) -> None:
    """One line per board: which sensor, how received, full JSON (ts_ms = server time)."""
    tag = "[THERMAL]" if payload.get("sensor") == "thermal" else "[PROXIMITY]"
    line = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    print(f"{tag} via-{source} {line}", flush=True)


def _log_fusion_result(rec: FusedRecord, applied: bool, room_after: int) -> None:
    out = {
        "fused_delta": rec.fused_delta,
        "direction": rec.direction,
        "confidence_fused": round(rec.confidence_fused, 4),
        "reason": rec.reason,
        "paired": rec.paired,
        "needs_review": rec.needs_review,
        "applied_to_count": applied,
        "room_count_after": room_after,
    }
    print("[FUSION] " + json.dumps(out, ensure_ascii=False, separators=(",", ":")), flush=True)


def _feat(ev: dict[str, Any], key: str, default: Any = 0) -> Any:
    f = ev.get("features") or {}
    return f.get(key, default)


def _compute_fused_confidence(
    dt_ms: float,
    dir_match: bool,
    prox_conf: float,
    therm_conf: float,
    scenario_bonus: float,
) -> float:
    time_score = math.exp(-abs(dt_ms) / 300.0)
    dir_score = 1.0 if dir_match else 0.4
    sensor_score = 0.55 * prox_conf + 0.45 * therm_conf
    raw = 0.5 * sensor_score + 0.3 * time_score + 0.2 * dir_score + scenario_bonus
    return max(0.0, min(1.0, raw))


def _remove_event(q: deque[tuple[int, dict[str, Any]]], ev: dict[str, Any]) -> None:
    sid = ev.get("event_id")
    sns = ev.get("sensor")
    kept = [(t, x) for t, x in q if not (x.get("event_id") == sid and x.get("sensor") == sns)]
    q.clear()
    q.extend(kept)


def _prune_stale(now_ms: int) -> None:
    for q in (_prox_q, _therm_q):
        fresh = deque([(t, x) for t, x in q if now_ms - t < 2500], maxlen=q.maxlen)
        q.clear()
        q.extend(fresh)


def _pair_and_fuse(incoming: dict[str, Any]) -> FusedRecord | None:
    """Match proximity ↔ thermal within FUSION_WINDOW_MS and apply plan rules."""
    ts = int(incoming["ts_ms"])
    inc_dir = incoming["direction"]
    _prune_stale(ts)

    if incoming["sensor"] == "proximity":
        _prox_q.append((ts, dict(incoming)))
        other_ev = _find_best_partner(ts, inc_dir, _therm_q)
    else:
        _therm_q.append((ts, dict(incoming)))
        other_ev = _find_best_partner(ts, inc_dir, _prox_q)

    if other_ev is None:
        return _single_sensor_fuse(incoming, paired=False)

    dt = abs(ts - int(other_ev["ts_ms"]))
    if dt > FUSION_WINDOW_MS:
        return _single_sensor_fuse(incoming, paired=False)

    prox = incoming if incoming["sensor"] == "proximity" else other_ev
    therm = incoming if incoming["sensor"] == "thermal" else other_ev
    _remove_event(_prox_q, prox)
    _remove_event(_therm_q, therm)

    p_conf = float(prox.get("confidence_raw", 0.5))
    t_conf = float(therm.get("confidence_raw", 0.5))
    double_c = int(_feat(therm, "double_candidate", 0)) == 1
    wheel_c = int(_feat(therm, "wheelchair_candidate", 0)) == 1
    seq_ok = int(_feat(prox, "seq_ok", 1)) == 1

    d_p = prox["direction"]
    d_t = therm["direction"]
    conflict = d_p != d_t and d_p != "unknown" and d_t != "unknown"
    dir_match = not conflict
    effective_dir: Direction = d_t if conflict and (t_conf - p_conf) >= 0.2 and not seq_ok else (d_p if not conflict else d_t)

    if conflict and not (t_conf - p_conf >= 0.2 and not seq_ok):
        effective_dir = d_p

    scenario_bonus = 0.0
    delta = 1

    if wheel_c:
        delta = 1
        effective_dir = d_t if d_t != "unknown" else d_p
        scenario_bonus += 0.08
    elif double_c and t_conf >= 0.75:
        delta = max(int(therm.get("delta_est", 2)), 2)
        scenario_bonus += 0.1
    else:
        delta = int(prox.get("delta_est", 1))

    conf = _compute_fused_confidence(float(dt), dir_match, p_conf, t_conf, scenario_bonus)
    reason = "paired:thermal_double" if double_c and delta >= 2 else "paired:wheelchair" if wheel_c else "paired:default"

    needs = conf < 0.75 and conf >= 0.55
    if conf < 0.55:
        reason += ";low_conf_hold"

    return FusedRecord(
        ts_ms=ts,
        fused_delta=delta,
        direction=effective_dir,
        confidence_fused=conf,
        reason=reason,
        prox_snapshot=prox,
        therm_snapshot=therm,
        needs_review=needs,
        paired=True,
    )


def _find_best_partner(ts: int, direction: str, q: deque[tuple[int, dict[str, Any]]]) -> dict[str, Any] | None:
    best: dict[str, Any] | None = None
    best_dt = FUSION_WINDOW_MS + 1
    for t, ev in q:
        if abs(ts - t) > FUSION_WINDOW_MS:
            continue
        d = ev.get("direction")
        dt = abs(ts - t)
        if d == direction or direction == "unknown" or d == "unknown":
            if dt < best_dt:
                best_dt = dt
                best = ev
    return best


def _single_sensor_fuse(ev: dict[str, Any], paired: bool = False) -> FusedRecord:
    conf = float(ev.get("confidence_raw", 0.5))
    sensor = ev["sensor"]
    reason = f"single:{sensor}"
    delta = int(ev.get("delta_est", 1))
    direction = ev["direction"]

    if sensor == "thermal":
        cn = int(ev.get("_thermal_cluster_n", 1))
        if int(_feat(ev, "wheelchair_candidate", 0)) == 1:
            delta = 1
            reason = "thermal_only:wheelchair"
        elif int(_feat(ev, "double_candidate", 0)) == 1 and conf >= 0.75:
            delta = max(max(delta, 2), min(cn, THERMAL_CLUSTER_CAP))
            reason = "thermal_only:double"
        elif cn > 1:
            delta = min(max(delta, cn), THERMAL_CLUSTER_CAP)
            reason = "thermal_only:cluster"

    needs = conf < 0.75 and conf >= 0.55
    return FusedRecord(
        ts_ms=int(ev["ts_ms"]),
        fused_delta=delta,
        direction=direction,
        confidence_fused=conf,
        reason=reason,
        prox_snapshot=ev if sensor == "proximity" else None,
        therm_snapshot=ev if sensor == "thermal" else None,
        needs_review=needs,
        paired=paired,
    )


def _should_apply_count(rec: FusedRecord) -> bool:
    c = rec.confidence_fused
    if c < 0.55:
        return False
    if rec.paired:
        return True
    if rec.prox_snapshot and not rec.therm_snapshot:
        return True
    if rec.therm_snapshot and not rec.prox_snapshot:
        return rec.reason.startswith("thermal_only:")
    return False


def _apply_count(rec: FusedRecord) -> None:
    d = rec.direction
    delta = rec.fused_delta
    if d == "in":
        state.room_count += delta
    elif d == "out":
        state.room_count = max(0, state.room_count - delta)
    # unknown: do not change count (could extend: ask operator)


def _dirs_match(a: dict[str, Any], b: dict[str, Any]) -> bool:
    da, db = a.get("direction"), b.get("direction")
    if da == "unknown" or db == "unknown":
        return True
    return da == db


def _fuse_paired_from_dicts(prox: dict[str, Any], therm: dict[str, Any], ts_fusion: int) -> FusedRecord:
    """Paired decision when both sensors have reported (defer-window path)."""
    dt = abs(int(prox["ts_ms"]) - int(therm["ts_ms"]))
    p_conf = float(prox.get("confidence_raw", 0.5))
    t_conf = float(therm.get("confidence_raw", 0.5))
    double_c = int(_feat(therm, "double_candidate", 0)) == 1
    wheel_c = int(_feat(therm, "wheelchair_candidate", 0)) == 1
    seq_ok = int(_feat(prox, "seq_ok", 1)) == 1
    cluster_n = min(int(therm.get("_thermal_cluster_n", 1)), THERMAL_CLUSTER_CAP)

    d_p = prox["direction"]
    d_t = therm["direction"]
    conflict = d_p != d_t and d_p != "unknown" and d_t != "unknown"
    dir_match = not conflict
    effective_dir: Direction = d_t if conflict and (t_conf - p_conf) >= 0.2 and not seq_ok else (d_p if not conflict else d_t)

    if conflict and not (t_conf - p_conf >= 0.2 and not seq_ok):
        effective_dir = d_p

    scenario_bonus = 0.0
    delta = 1

    if wheel_c:
        delta = 1
        effective_dir = d_t if d_t != "unknown" else d_p
        scenario_bonus += 0.08
    elif double_c and t_conf >= 0.75:
        delta = max(max(int(therm.get("delta_est", 2)), 2), cluster_n)
        scenario_bonus += 0.1
    else:
        delta = int(prox.get("delta_est", 1))
        if cluster_n > 1:
            delta = max(delta, cluster_n)

    conf = _compute_fused_confidence(float(dt), dir_match, p_conf, t_conf, scenario_bonus)
    if wheel_c:
        reason: str = "paired:wheelchair"
    elif double_c and t_conf >= 0.75 and delta >= 2:
        reason = "paired:thermal_double"
    elif cluster_n > 1 and delta >= 2:
        reason = "paired:thermal_cluster"
    else:
        reason = "paired:defer_window"

    needs = conf < 0.75 and conf >= 0.55
    if conf < 0.55:
        reason += ";low_conf_hold"

    return FusedRecord(
        ts_ms=ts_fusion,
        fused_delta=delta,
        direction=effective_dir,
        confidence_fused=conf,
        reason=reason,
        prox_snapshot=prox,
        therm_snapshot=therm,
        needs_review=needs,
        paired=True,
    )


def _commit_fusion_record(rec: FusedRecord) -> dict[str, Any]:
    apply = _should_apply_count(rec)
    if apply:
        _apply_count(rec)
    _log_fusion_result(rec, apply, state.room_count)
    entry = {
        "ts_ms": rec.ts_ms,
        "fused_delta": rec.fused_delta,
        "direction": rec.direction,
        "confidence_fused": round(rec.confidence_fused, 3),
        "reason": rec.reason,
        "needs_review": rec.needs_review,
    }
    state.recent_fused.appendleft({**entry, "prox": rec.prox_snapshot, "therm": rec.therm_snapshot})
    if rec.paired and rec.therm_snapshot is not None:
        state.suppress_thermal_only_until_ms = rec.ts_ms + THERMAL_DEDUP_AFTER_PAIR_MS
    return {"ok": True, "applied": apply, "fused": asdict(rec)}


def _flush_expired_pending(now_ms: int) -> bool:
    """Commit proximity-only / thermal-only when defer window elapsed. Returns True if anything committed."""
    did = False
    if state.pending_proximity is not None and now_ms >= state.pending_prox_deadline_ms:
        if state.pending_prox_thermal is not None:
            therm = dict(state.pending_prox_thermal)
            therm["_thermal_cluster_n"] = max(1, state.pending_prox_thermal_cluster_n)
            rec = _fuse_paired_from_dicts(state.pending_proximity, therm, now_ms)
        else:
            rec = _single_sensor_fuse(state.pending_proximity, paired=False)
        state.pending_proximity = None
        state.pending_prox_thermal = None
        state.pending_prox_thermal_cluster_n = 0
        _commit_fusion_record(rec)
        did = True
    if state.pending_thermal is not None and now_ms >= state.pending_thermal_deadline_ms:
        ev = dict(state.pending_thermal)
        ev["_thermal_cluster_n"] = state.pending_thermal_cluster_n
        state.pending_thermal = None
        state.pending_thermal_cluster_n = 1
        state.pending_thermal_cluster_start_ms = 0
        rec = _single_sensor_fuse(ev, paired=False)
        _commit_fusion_record(rec)
        did = True
    return did


def _status_dict() -> dict[str, Any]:
    """Snapshot for HTTP API and WebSocket (avoids forward-ref to route handlers)."""
    return {
        "room_count": state.room_count,
        "last_proximity": state.last_proximity,
        "last_thermal": state.last_thermal,
        "recent_fused": list(state.recent_fused),
        "connected_clients": state.connected_clients,
        "defer_window_ms": DEFER_WINDOW_MS,
        "pending_proximity": state.pending_proximity is not None,
        "pending_prox_thermal": state.pending_prox_thermal is not None,
        "pending_prox_thermal_cluster_n": state.pending_prox_thermal_cluster_n,
        "pending_thermal": state.pending_thermal is not None,
        "pending_prox_deadline_ms": state.pending_prox_deadline_ms,
        "pending_thermal_deadline_ms": state.pending_thermal_deadline_ms,
        "pending_thermal_cluster_n": state.pending_thermal_cluster_n,
        "suppress_thermal_until_ms": state.suppress_thermal_only_until_ms,
    }


async def _deferred_flush_loop() -> None:
    """Push timeout commits so the UI updates without waiting for the next HTTP packet."""
    while True:
        await asyncio.sleep(0.12)
        now_ms = int(time.time() * 1000)
        with _pending_state_lock:
            changed = _flush_expired_pending(now_ms)
        if changed:
            async with _broadcast_lock:
                await _broadcast({"type": "update", "state": _status_dict()})


async def _broadcast(message: dict[str, Any]) -> None:
    dead: list[WebSocket] = []
    for ws in _ws_clients:
        try:
            await ws.send_json(message)
        except Exception:
            dead.append(ws)
    for ws in dead:
        _ws_clients.discard(ws)


_ws_clients: set[WebSocket] = set()
_broadcast_lock = asyncio.Lock()


def ingest_event(raw: dict[str, Any], *, source: str = "http") -> dict[str, Any]:
    """Defer fusion: proximity ↔ thermal within DEFER_WINDOW_MS. Same-direction thermals coalesce (cap 4) before pairing."""
    received_ms = int(time.time() * 1000)
    merged = dict(raw)
    merged["ts_ms"] = received_ms
    body = SensorEventIn.model_validate(merged)
    payload = body.model_dump()
    payload["ts_ms"] = received_ms

    with _pending_state_lock:
        _flush_expired_pending(received_ms)

        if (
            payload["sensor"] == "thermal"
            and received_ms < state.suppress_thermal_only_until_ms
        ):
            print(
                "[FUSION] ignored duplicate thermal (dedup window after paired fusion)",
                flush=True,
            )
            return {"ok": True, "applied": False, "ignored": "thermal_dedup_after_pair"}

        if payload["sensor"] == "proximity":
            state.last_proximity = payload
        else:
            state.last_thermal = payload

        _log_sensor_json(payload, source)

        if payload["sensor"] == "proximity":
            if state.pending_thermal is not None and received_ms <= state.pending_thermal_deadline_ms and _dirs_match(
                state.pending_thermal, payload
            ):
                cn = state.pending_thermal_cluster_n
                therm = dict(state.pending_thermal)
                state.pending_thermal = None
                state.pending_thermal_cluster_n = 1
                state.pending_thermal_cluster_start_ms = 0
                therm["_thermal_cluster_n"] = cn
                rec = _fuse_paired_from_dicts(payload, therm, received_ms)
                return _commit_fusion_record(rec)

            state.pending_proximity = dict(payload)
            state.pending_prox_deadline_ms = received_ms + DEFER_WINDOW_MS
            state.pending_prox_thermal = None
            state.pending_prox_thermal_cluster_n = 0
            return {"ok": True, "applied": False, "deferred": "proximity", "deadline_ms": state.pending_prox_deadline_ms}

        if state.pending_proximity is not None and received_ms <= state.pending_prox_deadline_ms and _dirs_match(
            state.pending_proximity, payload
        ):
            # Proximity-first path: keep collecting same-direction thermal events until prox deadline.
            if state.pending_prox_thermal is None:
                state.pending_prox_thermal = dict(payload)
                state.pending_prox_thermal_cluster_n = 1
            elif _dirs_match(state.pending_prox_thermal, payload):
                state.pending_prox_thermal = dict(payload)
                state.pending_prox_thermal_cluster_n = min(
                    state.pending_prox_thermal_cluster_n + 1, THERMAL_CLUSTER_CAP
                )
            else:
                # Direction conflict after proximity-first start: keep the latest thermal snapshot.
                state.pending_prox_thermal = dict(payload)
                state.pending_prox_thermal_cluster_n = 1
            return {
                "ok": True,
                "applied": False,
                "deferred": "proximity_wait_thermal_cluster",
                "thermal_cluster": state.pending_prox_thermal_cluster_n,
                "deadline_ms": state.pending_prox_deadline_ms,
            }

        if state.pending_thermal is not None:
            same_dir = _dirs_match(state.pending_thermal, payload)
            in_defer = received_ms <= state.pending_thermal_deadline_ms
            if same_dir and in_defer:
                state.pending_thermal_cluster_n = min(
                    state.pending_thermal_cluster_n + 1, THERMAL_CLUSTER_CAP
                )
                state.pending_thermal = dict(payload)
                state.pending_thermal_deadline_ms = received_ms + DEFER_WINDOW_MS
                return {
                    "ok": True,
                    "applied": False,
                    "deferred": "thermal",
                    "thermal_cluster": state.pending_thermal_cluster_n,
                    "deadline_ms": state.pending_thermal_deadline_ms,
                }

        state.pending_thermal_cluster_n = 1
        state.pending_thermal_cluster_start_ms = received_ms
        state.pending_thermal = dict(payload)
        state.pending_thermal_deadline_ms = received_ms + DEFER_WINDOW_MS
        return {"ok": True, "applied": False, "deferred": "thermal", "deadline_ms": state.pending_thermal_deadline_ms}


async def tcp_ingest_handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
    try:
        data = await asyncio.wait_for(reader.readline(), timeout=20.0)
        line = data.decode("utf-8", errors="ignore").strip()
        if not line:
            writer.write(b"\n")
            await writer.drain()
            return
        if line[0] == "{":
            obj = json.loads(line)
            ingest_event(obj, source="tcp")
            async with _broadcast_lock:
                await _broadcast({"type": "update", "state": _status_dict()})
        writer.write(b"\n")
        await writer.drain()
    except (TimeoutError, asyncio.CancelledError, json.JSONDecodeError, ValueError, OSError):
        try:
            writer.write(b"\n")
            await writer.drain()
        except Exception:
            pass
    finally:
        writer.close()
        await writer.wait_closed()


@asynccontextmanager
async def lifespan(_: FastAPI):
    tcp_server = await asyncio.start_server(tcp_ingest_handler, "0.0.0.0", FUSION_TCP_PORT)
    bg_task = asyncio.create_task(_deferred_flush_loop())
    try:
        yield
    finally:
        bg_task.cancel()
        try:
            await bg_task
        except asyncio.CancelledError:
            pass
        tcp_server.close()
        await tcp_server.wait_closed()


app = FastAPI(title="Room Occupancy Fusion", version="1.0.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/status")
def api_status() -> dict[str, Any]:
    return _status_dict()


@app.post("/api/reset")
async def api_reset() -> dict[str, Any]:
    with _pending_state_lock:
        state.room_count = 0
        state.recent_fused.clear()
        state.pending_proximity = None
        state.pending_prox_thermal = None
        state.pending_prox_thermal_cluster_n = 0
        state.pending_thermal = None
        state.pending_prox_deadline_ms = 0
        state.pending_thermal_deadline_ms = 0
        state.pending_thermal_cluster_n = 1
        state.pending_thermal_cluster_start_ms = 0
        state.suppress_thermal_only_until_ms = 0
        _prox_q.clear()
        _therm_q.clear()
    async with _broadcast_lock:
        await _broadcast({"type": "update", "state": _status_dict()})
    return {"ok": True}


@app.post("/api/event")
async def api_event(body: SensorEventIn) -> dict[str, Any]:
    result = ingest_event(body.model_dump(exclude_none=False), source="http")
    async with _broadcast_lock:
        await _broadcast({"type": "update", "state": _status_dict()})
    return result


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    _ws_clients.add(ws)
    state.connected_clients = len(_ws_clients)
    try:
        await ws.send_json({"type": "hello", "state": _status_dict()})
        while True:
            # keep-alive; clients may send ping
            data = await ws.receive_text()
            if data == "ping":
                await ws.send_text("pong")
    except WebSocketDisconnect:
        pass
    finally:
        _ws_clients.discard(ws)
        state.connected_clients = len(_ws_clients)


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/favicon.ico", include_in_schema=False)
async def favicon() -> Response:
    return Response(status_code=204)


app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


if __name__ == "__main__":
    import uvicorn

    print(f"HTTP + WS on :8765 | Lab-style TCP JSON lines on :{FUSION_TCP_PORT}")
    print(
        "Tip: Web UI stays at 0 until ESP32 sends JSON on TCP :8766 (or use Simulate events). "
        "Sensor + fusion lines print below when events arrive.",
        flush=True,
    )
    uvicorn.run("server:app", host="0.0.0.0", port=8765, reload=True)
