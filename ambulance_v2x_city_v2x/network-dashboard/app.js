/**
 * NR-V2X Real-Time Network Visualization Dashboard
 * Core Frontend Application
 */

(function () {
  "use strict";

  // --- STATE ---
  const state = {
    connected: false,
    simTime: 0.0,
    simStep: 0,
    simDuration: 120.0,
    pacing: 0.5,

    topology: null, // Loaded from /api/network
    ambulances: {}, // Live ambulances from simulation
    cars: [],       // Live ordinary cars
    signals: {},    // Live signals J1..J10

    metrics: {
      v2i_tx: 0,
      v2i_rx: 0,
      v2i_accepted: 0,
      i2i_tx: 0,
      i2i_rx: 0,
      total_app_tx: 0,
      total_app_rx: 0,
      active_ambulances: 0,
      active_cars: 0,
    },

    // Animations & Visuals
    packets: [], // Active in-flight packet animations
    ripples: [], // Arrival confirmation ripples

    // Event log
    events: [],
    filterType: "all",
    filterNode: "all",
    selectedEntity: null, // { type: 'junction'|'ambulance'|'packet', id: ... }

    // View transform
    view: {
      scale: 1.0,
      offsetX: 0,
      offsetY: 0,
      autoFit: true,
      isDragging: false,
      dragStartX: 0,
      dragStartY: 0,
    },

    // View options
    options: {
      showCars: true,
      showRoads: true,
      showLabels: true,
    },

    autoScrollLog: true,
  };

  // --- DOM ELEMENTS ---
  const el = {
    canvas: document.getElementById("topology-canvas"),
    canvasWrapper: document.getElementById("canvas-wrapper"),
    connPill: document.getElementById("connection-pill"),
    connStatus: document.getElementById("connection-status"),
    simTimeVal: document.getElementById("sim-time-val"),
    simStepVal: document.getElementById("sim-step-val"),
    simPacingVal: document.getElementById("sim-pacing-val"),

    statV2iTx: document.getElementById("stat-v2i-tx"),
    statV2iRx: document.getElementById("stat-v2i-rx"),
    statV2iAcc: document.getElementById("stat-v2i-acc"),
    statI2iTx: document.getElementById("stat-i2i-tx"),
    statI2iRx: document.getElementById("stat-i2i-rx"),
    statActiveAmb: document.getElementById("stat-active-amb"),
    statActiveJunc: document.getElementById("stat-active-junc"),

    hudSelectedNode: document.getElementById("hud-selected-node"),
    hudActivePackets: document.getElementById("hud-active-packets"),

    inspectorBadge: document.getElementById("inspector-badge"),
    inspectorContent: document.getElementById("inspector-content"),

    logRows: document.getElementById("event-log-rows"),
    logContainer: document.getElementById("log-table-container"),
    logCount: document.getElementById("log-count"),
    filterGroup: document.getElementById("filter-type-group"),
    filterNodeSelect: document.getElementById("filter-node-select"),

    toggleCars: document.getElementById("toggle-cars"),
    toggleRoads: document.getElementById("toggle-roads"),
    toggleLabels: document.getElementById("toggle-labels"),

    btnZoomIn: document.getElementById("btn-zoom-in"),
    btnZoomOut: document.getElementById("btn-zoom-out"),
    btnResetView: document.getElementById("btn-reset-view"),
    btnClearHistory: document.getElementById("btn-clear-history"),
  };

  const ctx = el.canvas.getContext("2d");

  // --- INITIALIZATION ---
  async function init() {
    setupCanvas();
    setupEventListeners();
    await loadTopology();
    connectEventSource();
    requestAnimationFrame(renderLoop);
  }

  // --- LOAD TOPOLOGY ---
  async function loadTopology() {
    try {
      const res = await fetch("/api/network");
      if (!res.ok) throw new Error("Failed to load network topology");
      state.topology = await res.json();
      resetViewFit();
    } catch (err) {
      console.error("[NET] Topology load error:", err);
    }
  }

  // --- CANVAS SETUP & RESIZE ---
  function setupCanvas() {
    function resize() {
      const rect = el.canvasWrapper.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      el.canvas.width = rect.width * dpr;
      el.canvas.height = rect.height * dpr;
      ctx.resetTransform();
      ctx.scale(dpr, dpr);
      if (state.view.autoFit) {
        resetViewFit();
      }
    }
    window.addEventListener("resize", resize);
    resize();
  }

  function resetViewFit() {
    if (!state.topology || !state.topology.bounds) return;
    const rect = el.canvasWrapper.getBoundingClientRect();
    const bounds = state.topology.bounds;
    const pad = 60;

    const netWidth = bounds.max_x - bounds.min_x;
    const netHeight = bounds.max_y - bounds.min_y;

    const scaleX = (rect.width - pad * 2) / netWidth;
    const scaleY = (rect.height - pad * 2) / netHeight;
    const scale = Math.min(scaleX, scaleY);

    state.view.scale = scale;
    state.view.offsetX = pad + (rect.width - pad * 2 - netWidth * scale) / 2;
    state.view.offsetY = pad + (rect.height - pad * 2 - netHeight * scale) / 2;
    state.view.autoFit = true;
  }

  // SUMO Coordinates (0..1500, 0..600 with y=0 at bottom) to Canvas Coordinates
  function toCanvasCoords(x, y) {
    if (!state.topology || !state.topology.bounds) return { cx: x, cy: y };
    const bounds = state.topology.bounds;
    const cx = state.view.offsetX + (x - bounds.min_x) * state.view.scale;
    const cy = state.view.offsetY + (bounds.max_y - y) * state.view.scale;
    return { cx, cy };
  }

  function toSumoCoords(cx, cy) {
    if (!state.topology || !state.topology.bounds) return { x: cx, y: cy };
    const bounds = state.topology.bounds;
    const x = bounds.min_x + (cx - state.view.offsetX) / state.view.scale;
    const y = bounds.max_y - (cy - state.view.offsetY) / state.view.scale;
    return { x, y };
  }

  // --- EVENT LISTENERS ---
  function setupEventListeners() {
    // Zoom & Pan
    el.canvasWrapper.addEventListener("wheel", (e) => {
      e.preventDefault();
      const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
      const rect = el.canvasWrapper.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      state.view.offsetX = mouseX - (mouseX - state.view.offsetX) * zoomFactor;
      state.view.offsetY = mouseY - (mouseY - state.view.offsetY) * zoomFactor;
      state.view.scale *= zoomFactor;
      state.view.autoFit = false;
    }, { passive: false });

    el.canvasWrapper.addEventListener("mousedown", (e) => {
      if (e.button === 0) {
        state.view.isDragging = true;
        state.view.dragStartX = e.clientX - state.view.offsetX;
        state.view.dragStartY = e.clientY - state.view.offsetY;
      }
    });

    window.addEventListener("mousemove", (e) => {
      if (state.view.isDragging) {
        state.view.offsetX = e.clientX - state.view.dragStartX;
        state.view.offsetY = e.clientY - state.view.dragStartY;
        state.view.autoFit = false;
      }
    });

    window.addEventListener("mouseup", () => {
      state.view.isDragging = false;
    });

    // Canvas click detection for Node inspection
    el.canvasWrapper.addEventListener("click", (e) => {
      const rect = el.canvasWrapper.getBoundingClientRect();
      const clickX = e.clientX - rect.left;
      const clickY = e.clientY - rect.top;
      handleCanvasClick(clickX, clickY);
    });

    // Controls
    el.btnZoomIn.addEventListener("click", () => {
      state.view.scale *= 1.25;
      state.view.autoFit = false;
    });

    el.btnZoomOut.addEventListener("click", () => {
      state.view.scale *= 0.8;
      state.view.autoFit = false;
    });

    el.btnResetView.addEventListener("click", resetViewFit);

    el.btnClearHistory.addEventListener("click", async () => {
      state.events = [];
      el.logRows.innerHTML = "";
      updateLogCount();
      try {
        await fetch("/api/clear", { method: "POST" });
      } catch (err) {}
    });

    // Options toggles
    el.toggleCars.addEventListener("change", (e) => state.options.showCars = e.target.checked);
    el.toggleRoads.addEventListener("change", (e) => state.options.showRoads = e.target.checked);
    el.toggleLabels.addEventListener("change", (e) => state.options.showLabels = e.target.checked);

    // Event Log Filters
    el.filterGroup.addEventListener("click", (e) => {
      if (e.target.classList.contains("filter-btn")) {
        el.filterGroup.querySelectorAll(".filter-btn").forEach((b) => b.classList.remove("active"));
        e.target.classList.add("active");
        state.filterType = e.target.dataset.filter;
        renderEventLog();
      }
    });

    el.filterNodeSelect.addEventListener("change", (e) => {
      state.filterNode = e.target.value;
      renderEventLog();
    });

    // Pause auto-scroll when user scrolls table
    el.logContainer.addEventListener("scroll", () => {
      const isAtBottom = el.logContainer.scrollHeight - el.logContainer.scrollTop <= el.logContainer.clientHeight + 40;
      state.autoScrollLog = isAtBottom;
    });
  }

  // --- NODE CLICK DETECTION ---
  function handleCanvasClick(clickX, clickY) {
    if (!state.topology) return;

    // 1. Check Ambulances first
    for (const [ambId, amb] of Object.entries(state.ambulances)) {
      const { cx, cy } = toCanvasCoords(amb.x, amb.y);
      const dist = Math.hypot(clickX - cx, clickY - cy);
      if (dist < 18) {
        selectEntity("ambulance", ambId);
        return;
      }
    }

    // 2. Check Junctions
    for (const [jid, junc] of Object.entries(state.topology.junctions)) {
      if (!junc.is_tls) continue;
      const { cx, cy } = toCanvasCoords(junc.x, junc.y);
      const dist = Math.hypot(clickX - cx, clickY - cy);
      if (dist < 22) {
        selectEntity("junction", jid);
        return;
      }
    }

    // Clicked empty area
    selectEntity(null, null);
  }

  function selectEntity(type, id) {
    state.selectedEntity = type ? { type, id } : null;
    el.hudSelectedNode.textContent = type ? `${type.toUpperCase()}: ${id}` : "None";
    updateInspector();
  }

  // --- SERVER-SENT EVENTS (SSE) STREAM ---
  function connectEventSource() {
    updateConnectionStatus("connecting", "CONNECTING...");
    const es = new EventSource("/api/events/stream");

    es.onopen = () => {
      updateConnectionStatus("live", "CONNECTED TO SIMULATION");
    };

    es.onerror = () => {
      updateConnectionStatus("disconnected", "STREAM DISCONNECTED (RETRYING)");
    };

    es.onmessage = (e) => {
      try {
        const msg = JSON.parse(e.data);
        handleStreamMessage(msg);
      } catch (err) {
        console.error("[SSE] Parse error:", err);
      }
    };
  }

  function updateConnectionStatus(type, label) {
    state.connected = type === "live";
    el.connPill.className = `status-pill ${type}`;
    el.connStatus.textContent = label;
  }

  function handleStreamMessage(msg) {
    if (!msg || !msg.type) return;

    if (msg.type === "connection_state") {
      if (!msg.is_connected) {
        updateConnectionStatus("disconnected", "SIMULATION STOPPED / DISCONNECTED");
      }
      return;
    }

    if (msg.type === "snapshot") {
      state.simTime = msg.sim_time || 0.0;
      state.simStep = msg.sim_step || 0;
      state.simDuration = msg.sim_duration || 120.0;
      state.pacing = msg.pacing || 0.5;
      state.ambulances = msg.ambulances || {};
      state.signals = msg.signals || {};
      if (msg.is_connected) {
        updateConnectionStatus("live", "CONNECTED TO SIMULATION");
      } else {
        updateConnectionStatus("waiting", "WAITING FOR SIMULATION");
      }
      if (msg.metrics) updateMetrics(msg.metrics);
      if (msg.history) {
        msg.history.forEach((evt) => addEventToHistory(evt, false));
        renderEventLog();
      }
      updateHeaderReadouts();
      return;
    }

    if (msg.metrics) {
      updateMetrics(msg.metrics);
    }

    switch (msg.type) {
      case "init":
        updateConnectionStatus("live", "CONNECTED TO SIMULATION");
        state.simDuration = msg.data.duration;
        state.pacing = msg.data.pacing;
        updateHeaderReadouts();
        break;

      case "step":
        state.simStep = msg.data.step;
        state.simTime = msg.data.sim_time;
        state.ambulances = msg.data.ambulances || {};
        state.cars = msg.data.cars || [];
        state.signals = msg.data.signals || {};
        updateHeaderReadouts();
        if (state.selectedEntity && state.selectedEntity.type === "junction") {
          updateInspector();
        }
        break;

      case "packet_tx":
        handlePacketTx(msg.data);
        addEventToHistory(msg, true);
        break;

      case "packet_rx":
        handlePacketRx(msg.data);
        addEventToHistory(msg, true);
        break;

      case "signal_event":
        addEventToHistory(msg, true);
        if (state.selectedEntity && state.selectedEntity.id === msg.data.junction) {
          updateInspector();
        }
        break;

      case "sim_stopped":
        updateConnectionStatus("disconnected", "SIMULATION COMPLETED / STOPPED");
        break;

      case "history_cleared":
        state.events = [];
        el.logRows.innerHTML = "";
        updateLogCount();
        break;
    }
  }

  function updateHeaderReadouts() {
    el.simTimeVal.textContent = `${state.simTime.toFixed(2)} s`;
    el.simStepVal.textContent = state.simStep;
    el.simPacingVal.textContent = `${state.pacing.toFixed(2)} s`;
  }

  function updateMetrics(m) {
    state.metrics = m;
    el.statV2iTx.textContent = m.v2i_tx;
    el.statV2iRx.textContent = m.v2i_rx;
    el.statV2iAcc.textContent = m.v2i_accepted;
    el.statI2iTx.textContent = m.i2i_tx;
    el.statI2iRx.textContent = m.i2i_rx;
    el.statActiveAmb.textContent = `${m.active_ambulances} / 8`;
  }

  // --- PACKET ANIMATION HANDLING ---
  function getNodePosition(nodeName) {
    if (!nodeName) return null;
    // Check ambulances
    if (state.ambulances[nodeName]) {
      return { x: state.ambulances[nodeName].x, y: state.ambulances[nodeName].y };
    }
    // Check junctions
    if (state.topology && state.topology.junctions[nodeName]) {
      return { x: state.topology.junctions[nodeName].x, y: state.topology.junctions[nodeName].y };
    }
    return null;
  }

  function handlePacketTx(data) {
    const fromPos = getNodePosition(data.sender);
    const toPos = getNodePosition(data.intended_dest);
    if (!fromPos || !toPos) return;

    const pkt = {
      id: `pkt_${Date.now()}_${Math.random().toString(36).substr(2, 5)}`,
      protocol: data.protocol,
      sender: data.sender,
      target: data.intended_dest,
      fromX: fromPos.x,
      fromY: fromPos.y,
      toX: toPos.x,
      toY: toPos.y,
      startTime: performance.now(),
      duration: 650, // 650ms flight duration for smooth visibility
      confirmed: false,
      accepted: false,
      raw: data,
    };
    state.packets.push(pkt);
  }

  function handlePacketRx(data) {
    // 1. Mark corresponding in-flight packet as confirmed if still in flight
    for (const pkt of state.packets) {
      if (pkt.protocol === data.protocol && pkt.sender === data.sender && pkt.target === data.receiver) {
        pkt.confirmed = true;
        pkt.accepted = !!data.accepted;
      }
    }

    // 2. Spawn arrival confirmation ripple at destination node
    const destPos = getNodePosition(data.receiver);
    if (destPos) {
      state.ripples.push({
        x: destPos.x,
        y: destPos.y,
        protocol: data.protocol,
        accepted: !!data.accepted,
        startTime: performance.now(),
        duration: 700,
      });
    }
  }

  // --- EVENT LOG MANAGEMENT ---
  function addEventToHistory(msg, live) {
    state.events.push(msg);
    if (state.events.length > 2000) state.events.shift();
    updateLogCount();

    if (live) {
      appendEventRow(msg);
    }
  }

  function updateLogCount() {
    el.logCount.textContent = `${state.events.length} events`;
  }

  function eventMatchesFilter(msg) {
    const etype = msg.type;
    const data = msg.data || {};

    // Protocol / Type filter
    if (state.filterType === "v2i") {
      if (data.protocol !== "V2I") return false;
    } else if (state.filterType === "i2i") {
      if (data.protocol !== "I2I") return false;
    } else if (state.filterType === "tx") {
      if (etype !== "packet_tx") return false;
    } else if (state.filterType === "rx") {
      if (etype !== "packet_rx") return false;
    }

    // Node filter
    if (state.filterNode !== "all") {
      const matchNode =
        data.sender === state.filterNode ||
        data.receiver === state.filterNode ||
        data.intended_dest === state.filterNode ||
        data.junction === state.filterNode ||
        data.ambulance === state.filterNode;
      if (!matchNode) return false;
    }

    return true;
  }

  function renderEventLog() {
    el.logRows.innerHTML = "";
    const filtered = state.events.filter(eventMatchesFilter);
    // Render last 300 to maintain high DOM performance
    const slice = filtered.slice(-300);
    slice.forEach((evt) => appendEventRow(evt));
  }

  function appendEventRow(msg) {
    if (!eventMatchesFilter(msg)) return;

    const data = msg.data || {};
    const tr = document.createElement("tr");

    let timeStr = `${(data.sumo_time !== undefined ? data.sumo_time : state.simTime).toFixed(1)}s`;
    let typeBadge = "";
    let senderStr = data.sender || data.junction || "--";
    let targetStr = data.receiver || data.intended_dest || data.ambulance || "--";
    let statusStr = "Normal";
    let detailsStr = "";

    if (msg.type === "packet_tx") {
      typeBadge = `<span class="badge-tag badge-${data.protocol.toLowerCase()}-tx">${data.protocol} TX</span>`;
      statusStr = `<span style="color: #94a3b8;">Emitted</span>`;
      detailsStr = data.protocol === "I2I"
        ? `msg_id: ${data.message_id}, hop: 0 (amb: ${data.ambulance})`
        : `dist: ${data.distance ? data.distance.toFixed(1) + 'm' : '--'}`;
    } else if (msg.type === "packet_rx") {
      typeBadge = `<span class="badge-tag badge-${data.protocol.toLowerCase()}-rx">${data.protocol} RX</span>`;
      if (data.accepted) {
        statusStr = `<span style="color: var(--color-confirm); font-weight:600;">Accepted</span>`;
      } else {
        statusStr = `<span style="color: #64748b;">Not Target</span>`;
      }
      detailsStr = data.protocol === "I2I"
        ? `msg_id: ${data.message_id}, hop: ${data.hop_count} (armed: ${data.ambulance})`
        : `V2I received at ${data.receiver}`;
    } else if (msg.type === "signal_event") {
      typeBadge = `<span class="badge-tag badge-sig">SIGNAL</span>`;
      statusStr = `<span style="color: #c084fc;">${data.event_type}</span>`;
      detailsStr = `${data.junction} &rarr; ${data.mode} for ${data.ambulance || '--'}`;
    } else {
      typeBadge = `<span class="badge-tag">${msg.type}</span>`;
      detailsStr = JSON.stringify(data);
    }

    tr.innerHTML = `
      <td class="mono">${timeStr}</td>
      <td>${typeBadge}</td>
      <td class="mono">${senderStr}</td>
      <td class="mono">${targetStr}</td>
      <td>${statusStr}</td>
      <td title="${detailsStr}">${detailsStr}</td>
    `;

    tr.addEventListener("click", () => {
      el.logRows.querySelectorAll("tr").forEach((r) => r.classList.remove("selected"));
      tr.classList.add("selected");
      selectEntity("packet", msg);
    });

    el.logRows.appendChild(tr);

    if (state.autoScrollLog) {
      el.logContainer.scrollTop = el.logContainer.scrollHeight;
    }
  }

  // --- INSPECTOR RENDERING ---
  function updateInspector() {
    if (!state.selectedEntity) {
      el.inspectorBadge.textContent = "Ready";
      el.inspectorContent.innerHTML = `<p class="placeholder-text">Click any junction, ambulance, or event row to inspect parameters.</p>`;
      return;
    }

    const { type, id } = state.selectedEntity;

    if (type === "junction") {
      el.inspectorBadge.textContent = "Junction RSU";
      const junc = state.topology && state.topology.junctions[id];
      const sig = state.signals[id] || {};
      const pos = junc ? `(${junc.x.toFixed(1)}, ${junc.y.toFixed(1)})` : "--";
      const ue = junc && junc.ue !== null ? `UE${junc.ue}` : "--";

      let modeColor = "#94a3b8";
      if (sig.mode === "GREEN") modeColor = "#22c55e";
      if (sig.mode === "CLEARING") modeColor = "#eab308";

      el.inspectorContent.innerHTML = `
        <div class="inspect-title-row">
          <span class="inspect-main-id">${id} Roadside Unit</span>
          <span class="mono" style="color: #38bdf8;">${ue}</span>
        </div>
        <div class="inspector-grid">
          <span class="inspect-label">Position:</span><span class="inspect-val mono">${pos}</span>
          <span class="inspect-label">Signal Mode:</span><span class="inspect-val" style="color:${modeColor}; font-weight:700;">${sig.mode || 'NORMAL'}</span>
          <span class="inspect-label">Corridor Owner:</span><span class="inspect-val mono">${sig.owner || 'None'}</span>
          <span class="inspect-label">Phase State:</span><span class="inspect-val mono">${sig.state || 'Normal Program'}</span>
          <span class="inspect-label">V2I Status:</span><span class="inspect-val">${sig.gui_v2i || 'WAITING'}</span>
          <span class="inspect-label">I2I Status:</span><span class="inspect-val">${sig.gui_i2i || 'WAITING'}</span>
          <span class="inspect-label">Armed Alert:</span><span class="inspect-val mono">${formatArmedInfo(sig.armed)}</span>
          <span class="inspect-label">Radio Bearer:</span><span class="inspect-val">I2I Sidelink Groupcast (L2:${100 + (junc.ue||0)})</span>
        </div>
      `;
    } else if (type === "ambulance") {
      el.inspectorBadge.textContent = "Emergency Vehicle";
      const amb = state.ambulances[id] || {};
      const def = state.topology && state.topology.ambulances[id];
      const ue = def && def.ue !== null ? `UE${def.ue}` : "--";
      const nextTls = amb.next_tls ? `${amb.next_tls.junction} (${amb.next_tls.distance.toFixed(1)}m ahead)` : "None upcoming";

      el.inspectorContent.innerHTML = `
        <div class="inspect-title-row">
          <span class="inspect-main-id">${id} Emergency Unit</span>
          <span class="mono" style="color: #f59e0b;">${ue}</span>
        </div>
        <div class="inspector-grid">
          <span class="inspect-label">Coordinates:</span><span class="inspect-val mono">(${amb.x ? amb.x.toFixed(1) : '--'}, ${amb.y ? amb.y.toFixed(1) : '--'})</span>
          <span class="inspect-label">Speed:</span><span class="inspect-val mono">${amb.speed ? amb.speed.toFixed(1) + ' m/s' : '--'}</span>
          <span class="inspect-label">Current Road:</span><span class="inspect-val mono">${amb.road || '--'}</span>
          <span class="inspect-label">Next Signal:</span><span class="inspect-val mono" style="color:#38bdf8;">${nextTls}</span>
          <span class="inspect-label">Planned Route:</span><span class="inspect-val mono" style="font-size:10px;">${def && def.route ? def.route.join(' &rarr; ') : '--'}</span>
          <span class="inspect-label">Sidelink TX:</span><span class="inspect-val">Groupcast 225.0.0.0:8000 (L2 ID 255)</span>
        </div>
      `;
    } else if (type === "packet") {
      el.inspectorBadge.textContent = "Packet Event";
      const msg = id; // full message object
      const data = msg.data || {};

      el.inspectorContent.innerHTML = `
        <div class="inspect-title-row">
          <span class="inspect-main-id">${data.protocol || 'APP'} Event</span>
          <span class="mono" style="color: var(--color-confirm);">${msg.type}</span>
        </div>
        <div class="inspector-grid">
          <span class="inspect-label">SUMO Time:</span><span class="inspect-val mono">${data.sumo_time !== undefined ? data.sumo_time.toFixed(2) + ' s' : '--'}</span>
          <span class="inspect-label">ns-3 Time:</span><span class="inspect-val mono">${data.ns3_time !== undefined ? data.ns3_time.toFixed(6) + ' s' : '--'}</span>
          <span class="inspect-label">Sender Node:</span><span class="inspect-val mono">${data.sender || '--'} (${data.sender_ue !== undefined ? 'UE' + data.sender_ue : '--'})</span>
          <span class="inspect-label">Dest Node:</span><span class="inspect-val mono">${data.receiver || data.intended_dest || '--'} (${data.receiver_ue !== undefined ? 'UE' + data.receiver_ue : '--'})</span>
          <span class="inspect-label">Confirmed RX:</span><span class="inspect-val mono">${data.accepted !== undefined ? (data.accepted ? 'YES (Accepted)' : 'YES (Non-Target)') : 'Pending confirmation'}</span>
          ${data.message_id !== undefined ? `<span class="inspect-label">Message ID:</span><span class="inspect-val mono">${data.message_id}</span>` : ''}
          ${data.hop_count !== undefined ? `<span class="inspect-label">Hop Count:</span><span class="inspect-val mono">${data.hop_count} (Single Hop)</span>` : ''}
          <span class="inspect-label">Channel/Band:</span><span class="inspect-val">5.89 GHz (band n47), 40 MHz BW</span>
        </div>
      `;
    }
  }

  function formatArmedInfo(armed) {
    if (!armed || Object.keys(armed).length === 0) return "None";
    return Object.entries(armed)
      .map(([amb, info]) => `${amb} (from ${info.from || '?'})`)
      .join(", ");
  }

  // --- CANVAS RENDERING LOOP (60 FPS) ---
  function renderLoop(now) {
    drawCanvas(now);
    requestAnimationFrame(renderLoop);
  }

  function drawCanvas(now) {
    const width = el.canvas.width / (window.devicePixelRatio || 1);
    const height = el.canvas.height / (window.devicePixelRatio || 1);

    // Clear background
    ctx.fillStyle = "#080c12";
    ctx.fillRect(0, 0, width, height);

    if (!state.topology) {
      drawLoading(width, height);
      return;
    }

    // 1. Draw Road Network
    if (state.options.showRoads) {
      drawRoads();
    }

    // 2. Draw Ordinary Traffic
    if (state.options.showCars) {
      drawCars();
    }

    // 3. Draw Junctions J1..J10
    drawJunctions();

    // 4. Draw Ambulances
    drawAmbulances();

    // 5. Draw In-Flight Packets
    drawPackets(now);

    // 6. Draw Arrival Ripples
    drawRipples(now);
  }

  function drawLoading(w, h) {
    ctx.fillStyle = "#64748b";
    ctx.font = "13px system-ui";
    ctx.textAlign = "center";
    ctx.fillText("Loading Network Geometry...", w / 2, h / 2);
  }

  function drawRoads() {
    ctx.save();
    ctx.strokeStyle = "#1b2533";
    ctx.lineWidth = Math.max(2, 4 * state.view.scale);
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    for (const edge of state.topology.edges) {
      for (const lane of edge.lanes) {
        if (!lane.shape || lane.shape.length < 2) continue;
        ctx.beginPath();
        const first = toCanvasCoords(lane.shape[0][0], lane.shape[0][1]);
        ctx.moveTo(first.cx, first.cy);
        for (let i = 1; i < lane.shape.length; i++) {
          const pt = toCanvasCoords(lane.shape[i][0], lane.shape[i][1]);
          ctx.lineTo(pt.cx, pt.cy);
        }
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  function drawCars() {
    ctx.save();
    ctx.fillStyle = "#475569";
    const carRadius = Math.max(1.8, 2.5 * state.view.scale);

    for (const car of state.cars) {
      const { cx, cy } = toCanvasCoords(car[1], car[2]);
      ctx.beginPath();
      ctx.arc(cx, cy, carRadius, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  function drawJunctions() {
    const junctions = state.topology.junctions;
    ctx.save();

    for (const [jid, junc] of Object.entries(junctions)) {
      if (!junc.is_tls) continue;
      const { cx, cy } = toCanvasCoords(junc.x, junc.y);
      const sig = state.signals[jid] || {};
      const isSelected = state.selectedEntity && state.selectedEntity.id === jid;

      // Base junction box
      const boxSize = 22;
      ctx.fillStyle = "#111827";
      ctx.strokeStyle = isSelected ? "#38bdf8" : "#243247";
      ctx.lineWidth = isSelected ? 2.5 : 1.5;

      ctx.beginPath();
      ctx.roundRect(cx - boxSize / 2, cy - boxSize / 2, boxSize, boxSize, 4);
      ctx.fill();
      ctx.stroke();

      // Traffic Signal Indicator Dot
      let signalColor = "#475569"; // Normal program
      if (sig.mode === "GREEN") signalColor = "#22c55e"; // Emergency priority green
      else if (sig.mode === "CLEARING") signalColor = "#eab308"; // Clearing yellow

      ctx.fillStyle = signalColor;
      ctx.beginPath();
      ctx.arc(cx, cy, 5.5, 0, Math.PI * 2);
      ctx.fill();

      // If Armed with I2I alert: draw glowing blue halo around the signal
      if (sig.armed && Object.keys(sig.armed).length > 0) {
        ctx.strokeStyle = "rgba(56, 189, 248, 0.75)";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(cx, cy, 9, 0, Math.PI * 2);
        ctx.stroke();
      }

      // Labels
      if (state.options.showLabels) {
        ctx.font = "bold 11px ui-monospace, monospace";
        ctx.textAlign = "center";
        ctx.fillStyle = isSelected ? "#38bdf8" : "#f1f5f9";
        ctx.fillText(jid, cx, cy - boxSize / 2 - 5);

        // Subtitle UE ID
        ctx.font = "9px ui-monospace, monospace";
        ctx.fillStyle = "#64748b";
        ctx.fillText(`UE${junc.ue}`, cx, cy + boxSize / 2 + 11);
      }
    }
    ctx.restore();
  }

  function drawAmbulances() {
    ctx.save();
    for (const [ambId, amb] of Object.entries(state.ambulances)) {
      const { cx, cy } = toCanvasCoords(amb.x, amb.y);
      const isSelected = state.selectedEntity && state.selectedEntity.id === ambId;

      // Vehicle body marker
      ctx.save();
      ctx.translate(cx, cy);
      if (amb.angle !== undefined) {
        ctx.rotate((amb.angle * Math.PI) / 180);
      }

      // Ambulance outer box
      ctx.fillStyle = "#dc2626";
      ctx.strokeStyle = isSelected ? "#f59e0b" : "#ffffff";
      ctx.lineWidth = isSelected ? 2.5 : 1.5;

      ctx.beginPath();
      ctx.roundRect(-8, -5, 16, 10, 2);
      ctx.fill();
      ctx.stroke();

      // Front bumper indicator
      ctx.fillStyle = "#facc15";
      ctx.fillRect(5, -3, 3, 6);
      ctx.restore();

      // Label
      if (state.options.showLabels) {
        ctx.font = "bold 11px ui-monospace, monospace";
        ctx.textAlign = "center";
        ctx.fillStyle = isSelected ? "#f59e0b" : "#f87171";
        ctx.fillText(ambId, cx, cy - 11);
      }
    }
    ctx.restore();
  }

  function drawPackets(now) {
    ctx.save();
    const alivePackets = [];

    for (const pkt of state.packets) {
      const elapsed = now - pkt.startTime;
      const progress = Math.min(1.0, elapsed / pkt.duration);

      // Interpolate in canvas coordinates
      const from = toCanvasCoords(pkt.fromX, pkt.fromY);
      const to = toCanvasCoords(pkt.toX, pkt.toY);

      const curX = from.cx + (to.cx - from.cx) * progress;
      const curY = from.cy + (to.cy - from.cy) * progress;

      const isV2I = pkt.protocol === "V2I";
      const packetColor = isV2I ? "#f59e0b" : "#38bdf8";

      // 1. Draw subtle communication ray between nodes
      ctx.beginPath();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = isV2I ? "rgba(245, 158, 11, 0.25)" : "rgba(56, 189, 248, 0.25)";
      ctx.lineWidth = 1.2;
      ctx.moveTo(from.cx, from.cy);
      ctx.lineTo(to.cx, to.cy);
      ctx.stroke();
      ctx.setLineDash([]);

      // 2. Draw flying particle
      ctx.fillStyle = packetColor;
      ctx.shadowColor = packetColor;
      ctx.shadowBlur = 8;
      ctx.beginPath();
      ctx.arc(curX, curY, 4.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.shadowBlur = 0;

      // 3. Small protocol pill above packet
      ctx.font = "bold 9px ui-monospace, monospace";
      ctx.textAlign = "center";
      ctx.fillStyle = packetColor;
      ctx.fillText(pkt.protocol, curX, curY - 7);

      if (progress < 1.0) {
        alivePackets.push(pkt);
      }
    }

    state.packets = alivePackets;
    el.hudActivePackets.textContent = state.packets.length;
    ctx.restore();
  }

  function drawRipples(now) {
    ctx.save();
    const aliveRipples = [];

    for (const rip of state.ripples) {
      const elapsed = now - rip.startTime;
      const progress = Math.min(1.0, elapsed / rip.duration);
      const { cx, cy } = toCanvasCoords(rip.x, rip.y);

      const maxRadius = 26;
      const currentRadius = 10 + (maxRadius - 10) * progress;
      const alpha = 1.0 - progress;

      let color = rip.accepted ? "rgba(16, 185, 129," : "rgba(56, 189, 248,";
      if (!rip.accepted && rip.protocol === "V2I") {
        color = "rgba(148, 163, 184,";
      }

      ctx.strokeStyle = `${color} ${alpha})`;
      ctx.lineWidth = 2 * (1 - progress);
      ctx.beginPath();
      ctx.arc(cx, cy, currentRadius, 0, Math.PI * 2);
      ctx.stroke();

      if (progress < 1.0) {
        aliveRipples.push(rip);
      }
    }

    state.ripples = aliveRipples;
    ctx.restore();
  }

  // --- BOOTSTRAP ---
  window.addEventListener("DOMContentLoaded", init);
})();
