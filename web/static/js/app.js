const API_BASE = "";

function $(id) {
  return document.getElementById(id);
}

function fmtTime(ts) {
  if (ts == null) return "—";
  const d = new Date(Number(ts));
  if (Number.isNaN(d.getTime())) return String(ts);
  return d.toLocaleTimeString(undefined, { hour12: false, hour: "2-digit", minute: "2-digit", second: "2-digit" });
}

function setConnStatus(online) {
  const pill = $("connPill");
  const dot = $("connDot");
  const text = $("connText");
  pill.classList.remove("online", "offline");
  if (online) {
    pill.classList.add("online");
    text.textContent = "Live (WebSocket)";
  } else {
    pill.classList.add("offline");
    text.textContent = "Offline — reconnecting…";
  }
}

function renderState(s) {
  $("roomCount").textContent = String(s.room_count ?? 0);

  const t = s.last_thermal;
  $("thermalMeta").textContent = t
    ? `dir=${t.direction} · Δ=${t.delta_est} · conf=${Number(t.confidence_raw).toFixed(2)} · id=${t.event_id}`
    : "No event yet";

  const p = s.last_proximity;
  $("proxMeta").textContent = p
    ? `dir=${p.direction} · Δ=${p.delta_est} · conf=${Number(p.confidence_raw).toFixed(2)} · id=${p.event_id}`
    : "No event yet";

  $("wsClients").textContent = `Clients: ${s.connected_clients ?? 0}`;

  const tbody = $("logBody");
  const rows = s.recent_fused || [];
  tbody.innerHTML = "";
  if (!rows.length) {
    const tr = document.createElement("tr");
    tr.className = "empty-row";
    tr.innerHTML = `<td colspan="6">Waiting for events from <code>/api/event</code>…</td>`;
    tbody.appendChild(tr);
    return;
  }
  for (const e of rows) {
    const tr = document.createElement("tr");
    const review = e.needs_review
      ? `<span class="pill-warn">Review</span>`
      : "—";
    tr.innerHTML = `
      <td>${fmtTime(e.ts_ms)}</td>
      <td>${escapeHtml(e.direction)}</td>
      <td>${escapeHtml(String(e.fused_delta))}</td>
      <td>${escapeHtml(String(e.confidence_fused))}</td>
      <td>${escapeHtml(e.reason || "")}</td>
      <td>${review}</td>
    `;
    tbody.appendChild(tr);
  }
}

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

async function postJson(path, body) {
  const r = await fetch(`${API_BASE}${path}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!r.ok) throw new Error(await r.text());
  return r.json();
}

function connectWs() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const url = `${proto}//${location.host}/ws`;
  const ws = new WebSocket(url);
  ws.addEventListener("open", () => setConnStatus(true));
  ws.addEventListener("close", () => {
    setConnStatus(false);
    setTimeout(connectWs, 1200);
  });
  ws.addEventListener("error", () => setConnStatus(false));
  ws.addEventListener("message", (ev) => {
    try {
      const msg = JSON.parse(ev.data);
      if (msg.type === "hello" || msg.type === "update") {
        renderState(msg.state);
      }
    } catch {
      /* ignore */
    }
  });
  setInterval(() => {
    if (ws.readyState === WebSocket.OPEN) ws.send("ping");
  }, 25000);
}

async function bootstrap() {
  try {
    const r = await fetch(`${API_BASE}/api/status`);
    if (r.ok) renderState(await r.json());
  } catch {
    /* offline static preview */
  }
}

/** Poll fallback so the dashboard updates even if WebSocket is flaky (ESP32 uses TCP :8766). */
function startStatusPolling() {
  setInterval(async () => {
    try {
      const r = await fetch(`${API_BASE}/api/status`);
      if (r.ok) renderState(await r.json());
    } catch {
      /* ignore */
    }
  }, 400);
}

function demoEvents() {
  const base = Date.now();
  let id = Math.floor(Math.random() * 10000);
  const post = (sensor, direction, feats, conf, delta = 1) => {
    id += 1;
    return postJson("/api/event", {
      sensor,
      event_id: id,
      ts_ms: base + id,
      direction,
      delta_est: delta,
      features: feats,
      confidence_raw: conf,
    });
  };

  post("proximity", "in", { seq_ok: 1, transition_time_ms: 420 }, 0.88).catch(console.error);
  setTimeout(() => {
    post("thermal", "in", { double_candidate: 1, track_len: 12 }, 0.8, 2).catch(console.error);
  }, 120);

  setTimeout(() => {
    post("proximity", "in", { seq_ok: 1 }, 0.82).catch(console.error);
  }, 900);

  setTimeout(() => {
    post("thermal", "in", { wheelchair_candidate: 1, track_len: 20 }, 0.78).catch(console.error);
  }, 950);
}

$("btnDemo").addEventListener("click", () => demoEvents());

$("btnReset").addEventListener("click", async () => {
  try {
    await postJson("/api/reset", {});
  } catch {
    $("roomCount").textContent = "0";
  }
});

bootstrap();
connectWs();
startStatusPolling();
