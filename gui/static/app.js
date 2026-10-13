(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  // Pivot marker layers: march sources ("pivot") and query waypoints
  // ("path-pivot"). Each has the same controls, with ids `${layer}-${field}`.
  const MARKER_LAYERS = ["pivot", "path-pivot"];
  const MARKER_FIELDS = ["mode", "color", "shape", "fill", "size", "length",
    "width", "opacity", "head", "head-size", "anchor", "halo", "zoom"];
  const markerIds = () =>
    MARKER_LAYERS.flatMap((layer) => MARKER_FIELDS.map((f) => `${layer}-${f}`));
  const ui = Object.fromEntries(
    [
      "map-select",
      "map-upload",
      "start-x",
      "start-y",
      "start-heading",
      "radius",
      "solver-mode",
      "interaction-mode",
      "march-button",
      "cancel-button",
      "run-status",
      "query-status",
      "coord-readout",
      "palette-select",
      "field-opacity",
      "distance-overlay",
      "colorbar-toggle",
      "contour-toggle",
      "contour-count",
      "contour-color",
      "contour-width",
      "path-color",
      "path-width",
      "reversal-toggle",
      "reversal-color",
      "witness-toggle",
      "collision-check",
      "fixed-goal-heading",
      "goal-x",
      "goal-y",
      "goal-heading",
      "query-goal-button",
      "sample-step",
      "metrics-content",
      "colorbar",
      "colorbar-max",
      "colorbar-min",
      ...markerIds(),
    ].map((id) => [id, $(id)]),
  );
  const canvas = {
    map: $("map-canvas"),
    distance: $("distance-canvas"),
    laplacian: $("laplacian-canvas"),
  };
  const S = {
    boot: null,
    map: null,
    image: null,
    loadedMap: null,
    mapToken: 0,
    mapSelection: 0,
    mapLoading: true,
    mapChanging: false,
    epoch: 0,
    run: null,
    fields: null,
    rasters: {},
    contours: null,
    contourKey: "",
    query: null,
    intent: 0,
    queries: { direct: { pending: null, busy: false }, retained: { pending: null, busy: false } },
    poll: null,
    tab: "map",
    pointer: null,
    view: { fit: true, scale: 1, ox: 0, oy: 0 },
  };
  const text = (id) => ui[id].value;
  const value = (id) => text(id).trim() === "" ? NaN : Number(text(id));
  const finite = Number.isFinite;
  const fmt = (v) =>
    v == null
      ? "—"
      : typeof v === "number"
        ? finite(v)
          ? v !== 0 && Math.abs(v) < 0.001
            ? v.toExponential(3)
            : v.toFixed(3).replace(/\.000$/, "")
          : "NaN"
        : String(v);
  const status = (s, bad = false) => {
    ui["run-status"].textContent = s;
    ui["run-status"].classList.toggle("status-error", bad);
  };
  const qstatus = (s, bad = false) => {
    ui["query-status"].textContent = s;
    ui["query-status"].classList.toggle("status-error", bad);
  };
  const runId = () => S.run?.run_id;
  const directMode = () => text("solver-mode").startsWith("direct");
  const directOrder = () => text("solver-mode") === "direct_fast" ? "astar" : "dijkstra";
  const config = () => ({
    map_id: S.map?.id,
    start: {
      x: value("start-x"),
      y: value("start-y"),
      heading_deg: value("start-heading"),
    },
    radius: value("radius"),
  });
  const configKey = () => JSON.stringify(config());
  const mapReady = () => S.map && S.image && !S.mapLoading && !S.mapChanging;
  const goalCell = () => {
    const x = value("goal-x"), y = value("goal-y");
    return S.map && Number.isInteger(x) && Number.isInteger(y) &&
      x >= 0 && y >= 0 && x < S.map.width && y < S.map.height ? { x, y } : null;
  };
  const responseError = async (r) => {
    let detail;
    try { detail = (await r.json()).error; } catch (_) {}
    const error = new Error(detail?.message || detail?.code || `HTTP ${r.status}`);
    error.code = detail?.code;
    error.status = r.status;
    return error;
  };
  const errorText = async (r) => (await responseError(r)).message;
  const cellAt = (p) => {
    if (
      !mapReady() ||
      !finite(p.x) ||
      !finite(p.y) ||
      p.x < -0.5 ||
      p.y < -0.5 ||
      p.x > S.map.width - 0.5 ||
      p.y > S.map.height - 0.5
    )
      return null;
    const x = Math.round(p.x),
      y = Math.round(p.y);
    return x >= 0 && y >= 0 && x < S.map.width && y < S.map.height
      ? { x, y }
      : null;
  };
  const validConfig = (c) =>
    mapReady() &&
    c.map_id === S.map.id &&
    Number.isInteger(c.start.x) &&
    Number.isInteger(c.start.y) &&
    c.start.x >= 0 &&
    c.start.y >= 0 &&
    c.start.x < S.map.width &&
    c.start.y < S.map.height &&
    finite(c.start.heading_deg) &&
    finite(c.radius) &&
    c.radius >= 0.05 &&
    c.radius <= 10000;
  const current = (epoch, id, key) =>
    epoch === S.epoch && id === runId() && key === configKey();

  // Serialize run mutations, including cancellation before a POST has returned
  // its run ID. Field and path reads retain their independent stale guards.
  let mutations = Promise.resolve();
  function mutate(operation) {
    const pending = mutations.then(operation);
    mutations = pending.catch(() => {});
    return pending;
  }

  function resetQuery(
    message = "March a distance field, then click or drag to query a path.",
  ) {
    S.intent++;
    for (const stream of Object.values(S.queries)) stream.pending = null;
    S.query = null;
    qstatus(message);
    updateMetrics();
    drawAll();
  }
  function clearResult(message) {
    S.fields = null;
    S.rasters = {};
    S.contours = null;
    S.contourKey = "";
    resetQuery(message);
  }
  async function abandonRun() {
    const id = runId();
    clearTimeout(S.poll);
    S.poll = null;
    S.run = null;
    clearResult();
    if (id)
      try {
        await mutate(() => fetch(`/api/runs/${encodeURIComponent(id)}`, {
          method: "DELETE",
        }));
      } catch (_) {}
  }
  async function invalidate(
    message = "Configuration changed. March to compute a new field.",
  ) {
    ++S.epoch;
    status(message);
    const abandoning = abandonRun();
    ui["march-button"].disabled = false;
    ui["cancel-button"].disabled = true;
    qstatus(message);
    await abandoning;
  }

  function canvasSize(c) {
    const rect = c.getBoundingClientRect(),
      dpr = devicePixelRatio || 1,
      width = Math.max(1, Math.round(rect.width * dpr)),
      height = Math.max(1, Math.round(rect.height * dpr));
    if (c.width !== width || c.height !== height) {
      c.width = width;
      c.height = height;
    }
    return { rect, dpr };
  }
  function setup(ctx, c) {
    const { rect, dpr } = canvasSize(c);
    if (S.view.fit && S.map) {
      S.view.scale = Math.max(
        0.02,
        Math.min(
          (rect.width - 36) / S.map.width,
          (rect.height - 36) / S.map.height,
        ),
      );
      S.view.ox = (rect.width - S.map.width * S.view.scale) / 2;
      S.view.oy = (rect.height - S.map.height * S.view.scale) / 2;
      S.view.fit = false;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.translate(S.view.ox, S.view.oy);
    ctx.scale(S.view.scale, S.view.scale);
  }
  const point = (e, c) => {
    const r = c.getBoundingClientRect();
    return {
      x: (e.clientX - r.left - S.view.ox) / S.view.scale,
      y: (e.clientY - r.top - S.view.oy) / S.view.scale,
    };
  };
  function arrow(ctx, x, y, a, color, size = 5, outlined = false) {
    if (!finite(a)) return;
    const head = size * 0.45,
      half = head * 0.65;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(a);
    ctx.strokeStyle = ctx.fillStyle = color;
    ctx.lineWidth = 1 / S.view.scale;
    ctx.beginPath();
    ctx.moveTo(-size, 0);
    ctx.lineTo(size, 0);
    if (outlined) {
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 4 / S.view.scale;
      ctx.stroke();
      ctx.strokeStyle = color;
      ctx.lineWidth = 1 / S.view.scale;
    }
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(size, 0);
    ctx.lineTo(size - head, -half);
    ctx.lineTo(size - head, half);
    if (outlined) {
      ctx.closePath();
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 2 / S.view.scale;
      ctx.stroke();
    }
    ctx.fill();
    ctx.restore();
  }
  // The marker style of one layer. Sizes are screen pixels unless the layer
  // scales with zoom (map cells); an invalid number uses the control's default.
  function markerStyle(layer) {
    const control = (field) => ui[`${layer}-${field}`];
    const number = (field) => {
      const v = Number(control(field).value);
      return finite(v) && v > 0 ? v : Number(control(field).defaultValue);
    };
    const unit = control("zoom").checked ? 1 : 1 / S.view.scale;
    return {
      mode: control("mode").value,
      color: control("color").value,
      shape: control("shape").value,
      filled: control("fill").value === "filled",
      radius: number("size") * unit,
      length: number("length") * unit,
      width: number("width") * unit,
      alpha: Math.min(1, number("opacity")),
      head: control("head").value,
      headLength: (number("head-size") / 100) * number("length") * unit,
      tail: control("anchor").value === "tail",
      halo: control("halo").checked ? 2 / S.view.scale : 0,
    };
  }
  // Draws one pivot marker at (x, y) with heading a (radians, may be absent).
  function drawMarker(ctx, x, y, a, st) {
    // Each part is traced by build(); a halo is a wider white stroke beneath.
    const paint = (build, fill) => {
      ctx.beginPath();
      build();
      if (st.halo) {
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = st.width + 2 * st.halo;
        ctx.stroke();
      }
      ctx.strokeStyle = ctx.fillStyle = st.color;
      ctx.lineWidth = st.width;
      if (fill) ctx.fill();
      else ctx.stroke();
    };
    ctx.save();
    ctx.globalAlpha = st.alpha;
    ctx.translate(x, y);
    ctx.lineJoin = "round";
    if ((st.mode === "arrows" || st.mode === "both") && finite(a)) {
      ctx.save();
      ctx.rotate(a);
      const begin = st.tail ? 0 : -st.length / 2,
        tip = begin + st.length,
        h = Math.min(st.head === "none" ? 0 : st.headLength, st.length),
        w = 0.65 * h;
      // A filled head covers the shaft's end; an open head meets it at the tip.
      paint(() => {
        ctx.moveTo(begin, 0);
        ctx.lineTo(st.head === "filled" ? tip - h / 2 : tip, 0);
      }, false);
      if (st.head === "filled")
        paint(() => {
          ctx.moveTo(tip, 0);
          ctx.lineTo(tip - h, -w);
          ctx.lineTo(tip - h, w);
          ctx.closePath();
        }, true);
      else if (st.head === "open")
        paint(() => {
          ctx.moveTo(tip - h, -w);
          ctx.lineTo(tip, 0);
          ctx.lineTo(tip - h, w);
        }, false);
      ctx.restore();
    }
    if (st.mode === "points" || st.mode === "both") {
      const r = st.radius,
        lines = st.shape === "cross" || st.shape === "plus";
      if (st.shape === "triangle" && finite(a)) ctx.rotate(a);
      paint(() => {
        if (st.shape === "square") ctx.rect(-r, -r, 2 * r, 2 * r);
        else if (st.shape === "diamond") {
          ctx.moveTo(0, -r);
          ctx.lineTo(r, 0);
          ctx.lineTo(0, r);
          ctx.lineTo(-r, 0);
          ctx.closePath();
        } else if (st.shape === "triangle") {
          ctx.moveTo(1.3 * r, 0);
          ctx.lineTo(-0.8 * r, -0.9 * r);
          ctx.lineTo(-0.8 * r, 0.9 * r);
          ctx.closePath();
        } else if (st.shape === "cross") {
          ctx.moveTo(-r, -r);
          ctx.lineTo(r, r);
          ctx.moveTo(-r, r);
          ctx.lineTo(r, -r);
        } else if (st.shape === "plus") {
          ctx.moveTo(-r, 0);
          ctx.lineTo(r, 0);
          ctx.moveTo(0, -r);
          ctx.lineTo(0, r);
        } else ctx.arc(0, 0, r, 0, Math.PI * 2);
      }, st.filled && !lines);
    }
    ctx.restore();
  }
  const palette = () =>
    S.boot?.palettes?.find((p) => p.name === text("palette-select"));
  const coolwarm = () => ({
    colors: Array.from({ length: 256 }, (_, i) => {
      const t = i / 255;
      return t < 0.5
        ? [
            Math.round(59 + 392 * t),
            Math.round(76 + 348 * t),
            Math.round(192 + 126 * t),
          ]
        : [
            Math.round(255 - 304 * (t - 0.5)),
            Math.round(250 - 388 * (t - 0.5)),
            Math.round(255 - 416 * (t - 0.5)),
          ];
    }),
  });
  function raster(data, range, pal) {
    if (!data || !pal || !S.map) return null;
    const c = document.createElement("canvas");
    c.width = S.map.width;
    c.height = S.map.height;
    const image = c.getContext("2d").createImageData(c.width, c.height),
      span = Math.max(Number.EPSILON, range[1] - range[0]);
    for (let i = 0; i < data.length; i++) {
      if (!finite(data[i])) continue;
      const color =
          pal.colors[
            Math.max(
              0,
              Math.min(255, Math.round((255 * (data[i] - range[0])) / span)),
            )
          ],
        k = 4 * i;
      image.data[k] = color[0];
      image.data[k + 1] = color[1];
      image.data[k + 2] = color[2];
      image.data[k + 3] = 255;
    }
    c.getContext("2d").putImageData(image, 0, 0);
    return c;
  }
  function refreshRasters() {
    if (!S.fields || !S.run) return;
    S.rasters.distance = raster(
      S.fields.distance,
      S.run.distance_range,
      palette(),
    );
    const limit = S.run.laplacian_limit || 1;
    S.rasters.laplacian = raster(
      S.fields.laplacian,
      [-limit, limit],
      coolwarm(),
    );
    drawAll();
  }
  function drawContours(ctx) {
    if (!ui["contour-toggle"].checked || !S.contours?.paths) return;
    ctx.strokeStyle = text("contour-color");
    ctx.lineWidth = value("contour-width") / S.view.scale;
    for (const path of S.contours.paths) {
      ctx.beginPath();
      path.forEach((p, i) =>
        i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]),
      );
      ctx.stroke();
    }
  }
  function drawPivots(ctx) {
    const style = markerStyle("pivot");
    if (!S.run || style.mode === "none") return;
    for (const p of S.run.pivots || []) drawMarker(ctx, p.x, p.y, p.heading_rad, style);
  }
  function drawPath(ctx) {
    const q = S.query;
    if (!q?.polyline?.length) return;
    ctx.strokeStyle = text("path-color");
    ctx.lineWidth = value("path-width") / S.view.scale;
    ctx.beginPath();
    q.polyline.forEach((p, i) =>
      i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]),
    );
    ctx.stroke();
    if (ui["reversal-toggle"].checked)
      for (const r of q.reversals || []) {
        ctx.strokeStyle = text("reversal-color");
        ctx.setLineDash([3 / S.view.scale, 2 / S.view.scale]);
        ctx.beginPath();
        ctx.arc(r.x, r.y, 4 / S.view.scale, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    if (ui["witness-toggle"].checked)
      for (const w of q.collisions || []) {
        ctx.fillStyle = "#d32f2f";
        ctx.beginPath();
        ctx.arc(w.x, w.y, 3 / S.view.scale, 0, Math.PI * 2);
        ctx.fill();
      }
    const style = markerStyle("path-pivot");
    if (style.mode !== "none")
      for (const w of q.waypoints || [])
        if (w.role === "pivot") drawMarker(ctx, w.x, w.y, w.heading_rad, style);
  }
  function drawStart(ctx) {
    const p = config().start;
    if (finite(p.x) && finite(p.y) && finite(p.heading_deg)) {
      ctx.fillStyle = "#13748b";
      ctx.beginPath();
      ctx.arc(p.x, p.y, 5 / S.view.scale, 0, Math.PI * 2);
      ctx.fill();
      arrow(ctx, p.x, p.y, ((p.heading_deg % 360) * Math.PI) / 180,
        "#13748b", 18 / S.view.scale, true);
    }
    const goal = goalCell();
    if (goal) {
      ctx.strokeStyle = "#222";
      ctx.lineWidth = 2 / S.view.scale;
      ctx.beginPath();
      ctx.arc(goal.x, goal.y, 5 / S.view.scale, 0, Math.PI * 2);
      ctx.stroke();
      // Requested heading is visible before querying; a free-heading goal uses
      // the actual returned terminal pose, never an invented orientation.
      const angle = ui["fixed-goal-heading"].checked
        ? ((value("goal-heading") % 360) * Math.PI) / 180
        : S.query?.waypoints?.at(-1)?.heading_rad;
      if (finite(angle)) arrow(ctx, goal.x, goal.y, angle, "#222", 18 / S.view.scale, true);
    }
  }
  function overlay(ctx, kind) {
    const image =
      kind === "map"
        ? ui["distance-overlay"].checked
          ? S.rasters.distance
          : null
        : S.rasters[kind];
    if (image && !directMode()) {
      ctx.globalAlpha = value("field-opacity");
      ctx.drawImage(image, -0.5, -0.5, S.map.width, S.map.height);
      ctx.globalAlpha = 1;
    }
    if (!directMode()) {
      if (kind !== "laplacian") drawContours(ctx);
      drawPivots(ctx);
    }
    drawPath(ctx);
    drawStart(ctx);
  }
  function draw(c, kind) {
    const ctx = c.getContext("2d");
    canvasSize(c);
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = "#e9eef0";
    ctx.fillRect(0, 0, c.width, c.height);
    if (!S.image || !S.map) return;
    setup(ctx, c);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(S.image, -0.5, -0.5, S.map.width, S.map.height);
    overlay(ctx, kind);
  }
  function drawBar() {
    const distance =
        S.tab === "distance" ||
        (S.tab === "map" && ui["distance-overlay"].checked),
      lap = S.tab === "laplacian",
      show = !directMode() && (distance || lap) && ui["colorbar-toggle"].checked && S.run && S.fields;
    ui.colorbar.classList.toggle("hidden", !show);
    if (!show) return;
    const range = lap
        ? [-(S.run.laplacian_limit || 1), S.run.laplacian_limit || 1]
        : S.run.distance_range,
      pal = lap ? coolwarm() : palette();
    ui["colorbar-max"].textContent = fmt(range[1]);
    ui["colorbar-min"].textContent = fmt(range[0]);
    ui.colorbar.querySelector("i").style.background =
      `linear-gradient(to top,${pal.colors.map((c) => `rgb(${c[0]},${c[1]},${c[2]})`).join(",")})`;
  }
  function drawAll() {
    if (canvas[S.tab]) draw(canvas[S.tab], S.tab);
    drawBar();
  }

  async function loadMap(map, useSuggestedStart) {
    const token = ++S.mapToken;
    if (S.map?.id !== map.id) {
      ui["goal-x"].value = "";
      ui["goal-y"].value = "";
    }
    S.map = map;
    S.mapLoading = true;
    S.pointer = null;
    S.image = null;
    S.view.fit = true;
    drawAll();
    if (useSuggestedStart && map.suggested_start) {
      ui["start-x"].value = map.suggested_start.x;
      ui["start-y"].value = map.suggested_start.y;
    }
    const image = new Image();
    image.onload = () => {
      if (token !== S.mapToken || S.map?.id !== map.id) return;
      const c = document.createElement("canvas");
      c.width = map.width;
      c.height = map.height;
      c.getContext("2d").drawImage(image, 0, 0);
      S.image = c;
      S.loadedMap = map;
      S.mapLoading = false;
      S.view.fit = true;
      drawAll();
    };
    image.onerror = () => {
      if (token === S.mapToken) status("Could not load map preview.", true);
    };
    image.src =
      map.image_url || `/api/maps/${encodeURIComponent(map.id)}/image`;
  }
  async function contours(
    force = false,
    epoch = S.epoch,
    id = runId(),
    key = configKey(),
  ) {
    if (!S.run || !S.fields || !ui["contour-toggle"].checked) {
      drawAll();
      return;
    }
    const count = Math.max(
        2,
        Math.min(150, Math.round(value("contour-count"))),
      ),
      cacheKey = `${id}:${count}`;
    ui["contour-count"].value = count;
    if (!force && cacheKey === S.contourKey) {
      drawAll();
      return;
    }
    const r = await fetch(
      `/api/runs/${encodeURIComponent(id)}/contours?count=${count}`,
    );
    if (!current(epoch, id, key) || count !== Math.round(value("contour-count"))) return;
    if (!r.ok) throw Error(await errorText(r));
    const data = await r.json();
    if (
      current(epoch, id, key) &&
      data.run_id === id &&
      count === Math.round(value("contour-count"))
    ) {
      S.contours = data;
      S.contourKey = cacheKey;
      drawAll();
    }
  }
  async function fields(epoch, id, key) {
    const r = await fetch(
      S.run.fields_url || `/api/runs/${encodeURIComponent(id)}/fields`,
    );
    if (!r.ok) throw Error(await errorText(r));
    const buffer = await r.arrayBuffer();
    if (!current(epoch, id, key)) return;
    const n = S.run.width * S.run.height;
    if (buffer.byteLength !== n * 16) throw Error("Invalid binary field size");
    S.fields = {
      distance: new Float64Array(buffer, 0, n),
      laplacian: new Float64Array(buffer, n * 8, n),
    };
    refreshRasters();
    const draggingGoal = S.pointer?.mode === "goal" && S.pointer.epoch === S.epoch;
    if (!directMode() && goalCell() && !draggingGoal) requestTarget(goalCell());
    await contours(false, epoch, id, key);
  }
  async function poll(id, epoch, key) {
    try {
      const r = await fetch(`/api/runs/${encodeURIComponent(id)}`);
      if (!r.ok) throw Error(await errorText(r));
      const j = await r.json();
      if (!current(epoch, id, key)) return;
      if (j.status === "ready") {
        S.run = { ...j, configKey: key };
        ui["cancel-button"].disabled = true;
        ui["march-button"].disabled = false;
        if (!directMode()) {
          status("March complete. Select Query path to choose a goal.");
          qstatus("Distance field ready. Click or drag to query a path.");
        }
        try {
          await fields(epoch, id, key);
          if (current(epoch, id, key)) {
            updateMetrics();
          }
        } catch (e) {
          if (current(epoch, id, key))
            status(`Field loading failed: ${e.message}`, true);
        }
        return;
      }
      if (j.status === "cancelled" || j.status === "failed") {
        S.run = null;
        clearResult();
        ui["cancel-button"].disabled = true;
        ui["march-button"].disabled = false;
        status(
          `March ${j.status}: ${j.error?.message || ""}`,
          j.status === "failed",
        );
        return;
      }
      status(`Marching${j.phase ? `: ${j.phase}` : ""}`);
      S.poll = setTimeout(() => poll(id, epoch, key), 300);
    } catch (e) {
      if (!current(epoch, id, key)) return;
      status(`March status failed: ${e.message}`, true);
      ui["cancel-button"].disabled = true;
      ui["march-button"].disabled = false;
    }
  }
  async function march() {
    if (directMode()) {
      queryGoal();
      return;
    }
    const c = config();
    if (!validConfig(c)) {
      status(
        "Provide an in-map integer start, finite orientation, and minimum turning radius from 0.05 to 10000.",
        true,
      );
      return;
    }
    const clearing = invalidate();
    const epoch = S.epoch,
      key = JSON.stringify(c);
    ui["march-button"].disabled = true;
    ui["cancel-button"].disabled = false;
    status("Starting march…");
    try {
      await clearing;
      if (epoch !== S.epoch || key !== configKey()) return;
      ui["march-button"].disabled = true;
      ui["cancel-button"].disabled = false;
      status("Starting march…");
      await mutate(async () => {
        if (epoch !== S.epoch || key !== configKey()) return;
        const r = await fetch("/api/runs", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(c),
        });
        if (!r.ok) throw Error(await errorText(r));
        const j = await r.json();
        if (epoch !== S.epoch || key !== configKey()) {
          await fetch(`/api/runs/${encodeURIComponent(j.run_id)}`, {
            method: "DELETE",
          });
          return;
        }
        S.run = { ...j, configKey: key };
        void poll(j.run_id, epoch, key);
      });
    } catch (e) {
      if (epoch === S.epoch) {
        status(`March could not start: ${e.message}`, true);
        ui["march-button"].disabled = false;
        ui["cancel-button"].disabled = true;
      }
    }
  }
  async function cancel() {
    await invalidate("March cancelled.");
  }

  function queryLabel(q) {
    if (q.direct && q.status !== "invalid") {
      const share = q.direct.free_cells ? (100 * q.direct.cells_reached) / q.direct.free_cells : NaN;
      const length = q.status === "clear" ? ` · length ${fmt(q.reconstructed_length)}` : "";
      const mode = q.direct.order === "astar" ? "One-to-one (approximate)" : "One-to-one";
      return `${mode} ${q.status}${length} · reached ${fmt(share)}% of free cells in ${fmt(q.timings_ms?.direct_total)} ms`;
    }
    if (q.status === "unreached")
      return "No path to this goal was found; a path may still exist.";
    if (q.status === "invalid") return "Goal is outside the map or in an occupied cell.";
    if (q.status === "blocked")
      return `Blocked: ${q.collision_count ?? 0} blocked motion primitive(s); markers show a detected collision point on each blocked primitive.`;
    return `${q.status} · length ${fmt(q.reconstructed_length)}`;
  }
  function requestTarget(target) {
    if (!target || !mapReady()) return;
    ui["goal-x"].value = target.x;
    ui["goal-y"].value = target.y;
    if (!goalCell()) {
      rejectTargetQuery("Enter integer goal coordinates inside the map.");
      return;
    }
    if (directMode()) {
      if (!validConfig(config())) {
        rejectTargetQuery("Provide an in-map integer start, finite orientation, and minimum turning radius from 0.05 to 10000.");
        return;
      }
      ui["fixed-goal-heading"].checked = true;
      ui["goal-heading"].disabled = false;
    } else if (!S.run || !S.fields || S.run.configKey !== configKey()) {
      rejectTargetQuery("Goal set. March to compute a field for the start configuration.", false);
      return;
    }
    const step = value("sample-step");
    if (!finite(step) || step < 0.1 || step > 5) {
      rejectTargetQuery("Path display step must be between 0.1 and 5 cells.");
      return;
    }
    if (ui["fixed-goal-heading"].checked &&
        (!finite(value("goal-heading")) || Math.abs(value("goal-heading")) > 1e9)) {
      rejectTargetQuery("Goal orientation must be finite and between −1e9 and 1e9 degrees.");
      return;
    }
    S.intent++;
    S.query = null;
    const direct = directMode(), stream = S.queries[direct ? "direct" : "retained"];
    const body = { x: target.x, y: target.y,
      check_collisions: ui["collision-check"].checked, sample_step: step };
    if (ui["fixed-goal-heading"].checked) body.heading_deg = value("goal-heading");
    stream.pending = { target, body, config: config(), intent: S.intent,
      epoch: S.epoch, mapToken: S.mapToken, id: runId(), key: configKey(),
      mode: text("solver-mode"), order: directOrder() };
    updateMetrics();
    drawAll();
    if (!stream.busy) void sendTarget(stream, direct);
  }
  function queryGoal() {
    const target = goalCell();
    if (target) requestTarget(target);
    else {
      rejectTargetQuery("Enter integer goal coordinates inside the map.");
    }
  }
  function rejectTargetQuery(message, bad = true) {
    S.intent++;
    for (const stream of Object.values(S.queries)) stream.pending = null;
    S.query = null;
    qstatus(message, bad);
    updateMetrics();
    drawAll();
  }
  async function sendTarget(stream, direct) {
    const job = stream.pending;
    if (!job) return;
    stream.pending = null;
    stream.busy = true;
    const { epoch, id, key, mapToken, mode, body } = job;
    // Bind queued work and responses to the displayed occupancy and requested pose.
    const matchingRun = () => current(epoch, id, key) && mapToken === S.mapToken;
    const fresh = () => mapReady() && job.intent === S.intent &&
      mapToken === S.mapToken && text("solver-mode") === mode &&
      (direct ? epoch === S.epoch && key === configKey() : matchingRun());
    if (!fresh()) {
      stream.busy = false;
      return;
    }
    qstatus(direct ? `One-to-one query (${job.target.x}, ${job.target.y}): marching…`
                   : `Querying (${job.target.x}, ${job.target.y})…`);
    try {
      const r = direct
        ? await fetch("/api/direct", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              ...job.config,
              goal: { x: body.x, y: body.y, heading_deg: body.heading_deg },
              check_collisions: body.check_collisions,
              sample_step: body.sample_step,
              order: job.order,
            }),
          })
        : await fetch(`/api/runs/${encodeURIComponent(id)}/query`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body),
          });
      if (!r.ok) throw await responseError(r);
      const q = await r.json();
      if (fresh() && (direct || q.run_id === id)) {
        S.query = q;
        qstatus(queryLabel(q), !["clear", "reconstructed"].includes(q.status));
        updateMetrics();
        drawAll();
      }
    } catch (e) {
      if (!direct && matchingRun() && S.run?.status === "ready" &&
          ["solver_timeout", "solver_stopped", "solver_protocol"].includes(e.code)) {
        S.run = null;
        S.fields = null;
        S.rasters = {};
        S.contours = null;
        S.contourKey = "";
        stream.pending = null;
        ui["cancel-button"].disabled = true;
        ui["march-button"].disabled = false;
        if (!directMode()) {
          resetQuery(`Query failed: ${e.message}`);
          status(`One-to-all solver failed: ${e.message}. March again.`, true);
          qstatus(`Query failed: ${e.message}`, true);
        } else {
          updateMetrics();
          drawAll();
        }
      } else if (fresh()) qstatus(`Query failed: ${e.message}`, true);
    } finally {
      stream.busy = false;
      if (stream.pending) void sendTarget(stream, direct);
    }
  }
  function card(title, object) {
    const entries = Object.entries(object || {})
      .filter(([, v]) => typeof v !== "object")
      .map(([k, v]) => `<dt>${k.replaceAll("_", " ")}</dt><dd>${fmt(v)}</dd>`)
      .join("");
    return `<div class="metric-card"><h3>${title}</h3><dl>${entries || "<dt>—</dt><dd>—</dd>"}</dl></div>`;
  }
  function updateMetrics() {
    if (directMode() && S.query?.direct) {
      const q = S.query, d = q.direct;
      const direct = {
        order: d.order,
        cells_reached: d.cells_reached,
        free_cells: d.free_cells,
        marched_share_percent: d.free_cells ? (100 * d.cells_reached) / d.free_cells : null,
        stopped_early: d.stopped_early,
        stop_key: d.stop_key,
        settled_pops: d.pops,
        stop_evaluations: d.evaluations,
        evaluation_solves: d.evaluation_solves,
        evaluation_clearance_checks: d.evaluation_clearance_checks,
      };
      const path = {
        status: q.status,
        goal_x: q.target.x,
        goal_y: q.target.y,
        requested_goal_orientation_degrees: q.target.heading_rad * 180 / Math.PI,
        path_length: q.reconstructed_length,
        path_type: q.route_kind,
        gear_changes: q.reversal_count,
        blocked_motion_primitives: q.collision_count,
        terminal_clearance_qualified: q.terminal_clearance_qualified,
      };
      ui["metrics-content"].innerHTML =
        `<div class="metrics-grid metrics-summary">${card("Selected path", path)}${card("One-to-one query", direct)}${card("One-to-one runtime (ms)", q.timings_ms)}</div><div class="metrics-grid">${card("Query work", q.query_work)}</div><p class="hint">${d.stop_rule}. The partial field is not displayed.</p>`;
      return;
    }
    if (directMode() || !S.run?.metrics) {
      ui["metrics-content"].textContent = directMode() ? "No one-to-one query result." : "No completed march.";
      return;
    }
    const q = S.query,
      m = S.run.metrics,
      field = {
        width: S.run.width,
        height: S.run.height,
        distance_min: S.run.distance_range?.[0],
        distance_max: S.run.distance_range?.[1],
        effective_radius: S.run.effective_radius,
        sources_including_start: S.run.pivots?.length,
        ...m.field,
      },
      path = q
        ? {
            status: q.status,
            query_mode: q.query_mode,
            goal_x: q.target.x,
            goal_y: q.target.y,
            ...(q.query_mode === "fixed_heading"
              ? {
                  requested_goal_orientation_degrees: q.target.heading_rad * 180 / Math.PI,
                  path_length: q.reconstructed_length,
                  field_distance: q.stored_distance,
                }
              : {
                  field_distance: q.stored_distance,
                  path_length: q.reconstructed_length,
                }),
            distance_residual: q.distance_residual,
            word_match: q.word_match,
            gear_changes: q.reversal_count,
            blocked_motion_primitives: q.collision_count,
            collision_checked: q.collision_checked,
            terminal_clearance_qualified: q.terminal_clearance_qualified,
          }
        : {};
    const primitive = (q?.primitives || [])
        .map(
          (p) =>
            `<tr><td>${p.index}</td><td>${p.type}</td><td>${p.gear}</td><td>${fmt(p.length)}</td><td>${fmt(p.clearance)}</td></tr>`,
        )
        .join(""),
      waypoint = (q?.waypoints || [])
        .map(
          (p) =>
            `<tr><td>${p.role}</td><td>${fmt(p.x)}, ${fmt(p.y)}</td><td>${fmt((p.heading_rad * 180) / Math.PI)}°</td></tr>`,
        )
        .join("");
    ui["metrics-content"].innerHTML =
      `<div class="metrics-grid metrics-summary">${card("Selected path", path)}${card("Query runtime (ms)", q?.timings_ms)}${card("March runtime (ms)", m.timings_ms)}</div><h3>Motion primitives</h3><table><tr><th>#</th><th>Type</th><th>Gear</th><th>Length</th><th>Collision status</th></tr>${primitive}</table><h3>Waypoint orientations</h3><table><tr><th>Role</th><th>Position</th><th>Orientation</th></tr>${waypoint}</table><h3>Field and work details</h3><div class="metrics-grid">${card("Field", field)}${card("Work", m.work)}</div><p class="hint">The blocked motion primitive count reports how many path segments intersect obstacles.</p>`;
  }

  function setStart(cell, angle) {
    ui["start-x"].value = cell.x;
    ui["start-y"].value = cell.y;
    if (finite(angle)) ui["start-heading"].value = ((angle % 360) + 360) % 360;
  }
  function down(e, c) {
    if (e.button === 1 || e.button === 2 || e.shiftKey) {
      S.pointer = { pan: true, x: e.clientX, y: e.clientY };
      c.setPointerCapture(e.pointerId);
      return;
    }
    const cell = cellAt(point(e, c));
    if (!cell) return;
    const mode = text("interaction-mode");
    S.pointer = { mode, cell, epoch: S.epoch, mapId: S.map.id };
    c.setPointerCapture(e.pointerId);
    if (mode === "start") {
      setStart(cell);
      void invalidate(directMode() ? "Start configuration updated. Run a one-to-one query."
                                   : "Start configuration updated. March to compute a new field.");
    } else if (mode === "goal") {
      ui["fixed-goal-heading"].checked = true;
      ui["goal-heading"].disabled = false;
      if (!finite(value("goal-heading"))) ui["goal-heading"].value = 0;
      ui["goal-x"].value = cell.x;
      ui["goal-y"].value = cell.y;
      rejectTargetQuery("Drag to set the goal orientation; release to query.", false);
    } else requestTarget(cell);
  }
  function move(e, c) {
    const p = point(e, c),
      cell = cellAt(p);
    ui["coord-readout"].textContent = cell
      ? `x ${cell.x} · y ${cell.y}`
      : "x — · y —";
    if (!S.pointer) return;
    if (S.pointer.pan) {
      S.view.ox += e.clientX - S.pointer.x;
      S.view.oy += e.clientY - S.pointer.y;
      S.pointer.x = e.clientX;
      S.pointer.y = e.clientY;
      drawAll();
      return;
    }
    if (S.pointer.mode === "start") {
      const a =
        (Math.atan2(p.y - S.pointer.cell.y, p.x - S.pointer.cell.x) * 180) /
        Math.PI;
      if (Math.hypot(p.x - S.pointer.cell.x, p.y - S.pointer.cell.y) > 0.4)
        setStart(S.pointer.cell, a);
      drawAll();
    } else if (S.pointer.mode === "goal") {
      if (S.pointer.epoch !== S.epoch || S.pointer.mapId !== S.map.id) return;
      const dx = p.x - S.pointer.cell.x, dy = p.y - S.pointer.cell.y;
      if (Math.hypot(dx, dy) > 0.4)
        ui["goal-heading"].value = ((Math.atan2(dy, dx) * 180 / Math.PI) + 360) % 360;
      drawAll();
    } else if (cell) requestTarget(cell);
  }
  function up(e, c) {
    const pointer = S.pointer;
    if (c.hasPointerCapture(e.pointerId)) c.releasePointerCapture(e.pointerId);
    S.pointer = null;
    if (pointer?.mode === "goal" && e.type !== "pointercancel" &&
        pointer.epoch === S.epoch && pointer.mapId === S.map?.id)
      queryGoal();
  }
  function tab(name) {
    S.tab = name;
    for (const b of document.querySelectorAll("[role=tab]"))
      b.setAttribute("aria-selected", b.dataset.tab === name);
    for (const name of ["map", "distance", "laplacian", "metrics"])
      $(`panel-${name}`).classList.toggle("hidden", name !== S.tab);
    drawAll();
  }
  async function boot() {
    try {
      const r = await fetch("/api/bootstrap");
      if (!r.ok) throw Error(await errorText(r));
      S.boot = await r.json();
      for (const map of S.boot.maps || [])
        ui["map-select"].add(new Option(map.name, map.id));
      for (const p of S.boot.palettes || [])
        ui["palette-select"].add(new Option(p.name, p.name));
      const d = S.boot.defaults || {};
      ui["map-select"].value = d.map_id || S.boot.maps?.[0]?.id || "";
      ui["start-x"].value = d.start?.x ?? "";
      ui["start-y"].value = d.start?.y ?? "";
      ui["start-heading"].value = d.start?.heading_deg ?? 0;
      ui.radius.value = d.radius ?? "";
      ui["palette-select"].value = S.boot.palettes?.[0]?.name || "";
      await loadMap(
        S.boot.maps.find((map) => map.id === text("map-select")),
        false,
      );
      status("Map ready. Set a start configuration and march.");
    } catch (e) {
      status(`Could not load GUI: ${e.message}`, true);
    }
  }
  function safe(promise, label) {
    const epoch = S.epoch;
    promise.catch((e) => {
      if (epoch === S.epoch) status(`${label}: ${e.message}`, true);
    });
  }
  function wire() {
    ui["map-select"].addEventListener("change", () =>
      safe(
        (async () => {
          const selection = ++S.mapSelection;
          const id = text("map-select");
          S.mapChanging = true;
          const clearing = invalidate();
          await loadMap(S.boot.maps.find((map) => map.id === id), true);
          await clearing;
          if (selection === S.mapSelection) S.mapChanging = false;
        })(),
        "Map change failed",
      ),
    );
    ui["map-upload"].addEventListener("change", () =>
      safe(
        (async () => {
          const file = ui["map-upload"].files[0];
          if (!file) return;
          const selection = ++S.mapSelection;
          const previousMap = S.loadedMap;
          S.mapChanging = true;
          ++S.mapToken;
          S.mapLoading = true;
          S.map = null;
          S.image = null;
          S.pointer = null;
          ui["goal-x"].value = "";
          ui["goal-y"].value = "";
          const clearing = invalidate();
          try {
            const r = await fetch(
              `/api/maps?name=${encodeURIComponent(file.name)}`,
              { method: "POST", body: file },
            );
            if (!r.ok) throw Error(await errorText(r));
            const map = await r.json();
            if (!S.boot.maps.some((entry) => entry.id === map.id)) {
              S.boot.maps.push(map);
              ui["map-select"].add(new Option(map.name, map.id));
            }
            if (selection !== S.mapSelection) return;
            ui["map-select"].value = map.id;
            await loadMap(map, true);
            await clearing;
            if (selection === S.mapSelection) S.mapChanging = false;
          } catch (e) {
            if (selection === S.mapSelection) {
              if (previousMap) {
                ui["map-select"].value = previousMap.id;
                await loadMap(previousMap, false);
              }
              await clearing;
              S.mapChanging = false;
            }
            throw e;
          }
        })(),
        "Upload failed",
      ),
    );
    for (const id of ["start-x", "start-y", "start-heading", "radius"])
      ui[id].addEventListener("input", () => {
        void invalidate(directMode() ? "Configuration changed. Run a one-to-one query."
                                     : "Configuration changed. March to compute a new field.");
      });
    ui["march-button"].onclick = () => void march();
    ui["solver-mode"].addEventListener("change", () => {
      const direct = directMode();
      ui["march-button"].textContent = direct ? "One-to-one query" : "March";
      ui["fixed-goal-heading"].disabled = direct;
      if (direct) {
        ui["fixed-goal-heading"].checked = true;
        ui["goal-heading"].disabled = false;
      }
      // Mode is query state, not part of the retained march configuration.
      // Keep its run, fields and poll alive; invalidate only pending queries.
      S.pointer = null;
      const message = direct
        ? "One-to-one: set start and goal configurations, then query a path."
        : S.fields ? "Distance field ready. Query paths to goal positions."
                   : "One-to-all: march a distance field, then query paths.";
      status(message);
      resetQuery(message);
      ui["march-button"].disabled = !direct && S.run?.status === "running";
    });
    ui["cancel-button"].onclick = () => void cancel();
    ui["interaction-mode"].addEventListener("change", () => {
      S.pointer = null;
      if (text("interaction-mode") !== "query") return;
      if (!directMode()) {
        ui["fixed-goal-heading"].checked = false;
        ui["goal-heading"].disabled = true;
      }
      if (goalCell()) queryGoal();
      else drawAll();
    });
    for (const id of [
      "field-opacity",
      "distance-overlay",
      "colorbar-toggle",
      ...markerIds(),
      "path-color",
      "path-width",
      "reversal-toggle",
      "reversal-color",
      "witness-toggle",
      "contour-color",
      "contour-width",
    ])
      ui[id].addEventListener("input", drawAll);
    // Reset restores the markup defaults of a layer's style controls only.
    for (const details of document.querySelectorAll("fieldset.marker details"))
      details.querySelector("button.reset").onclick = () => {
        for (const el of details.querySelectorAll("input, select")) {
          if (el.type === "checkbox") el.checked = el.defaultChecked;
          else if (el.tagName === "SELECT")
            for (const o of el.options) o.selected = o.defaultSelected;
          else el.value = el.defaultValue;
        }
        drawAll();
      };
    ui["palette-select"].addEventListener("change", refreshRasters);
    ui["contour-toggle"].addEventListener("change", () =>
      safe(contours(), "Contours failed"),
    );
    ui["contour-count"].addEventListener("change", () =>
      safe(contours(true), "Contours failed"),
    );
    for (const id of ["collision-check", "sample-step"])
      ui[id].addEventListener("input", () =>
        resetQuery(
          "Query settings changed. Click or drag to query again.",
        ),
      );
    ui["fixed-goal-heading"].addEventListener("change", () => {
      ui["goal-heading"].disabled = !ui["fixed-goal-heading"].checked;
      if (goalCell()) queryGoal();
      else drawAll();
    });
    ui["goal-heading"].addEventListener("input", () => {
      if (ui["fixed-goal-heading"].checked && goalCell()) queryGoal();
      else drawAll();
    });
    for (const id of ["goal-x", "goal-y"])
      ui[id].addEventListener("input", queryGoal);
    ui["query-goal-button"].onclick = queryGoal;
    for (const b of document.querySelectorAll("[role=tab]"))
      b.onclick = () => tab(b.dataset.tab);
    for (const c of Object.values(canvas)) {
      c.addEventListener("pointerdown", (e) => down(e, c));
      c.addEventListener("pointermove", (e) => move(e, c));
      c.addEventListener("pointerup", (e) => up(e, c));
      c.addEventListener("pointercancel", (e) => up(e, c));
      c.addEventListener(
        "wheel",
        (e) => {
          e.preventDefault();
          const p = point(e, c),
            f = Math.exp(-e.deltaY * 0.001),
            r = c.getBoundingClientRect();
          S.view.scale = Math.max(0.02, Math.min(100, S.view.scale * f));
          S.view.ox = e.clientX - r.left - p.x * S.view.scale;
          S.view.oy = e.clientY - r.top - p.y * S.view.scale;
          drawAll();
        },
        { passive: false },
      );
      c.oncontextmenu = (e) => e.preventDefault();
    }
    new ResizeObserver(() => {
      S.view.fit = true;
      drawAll();
    }).observe(document.querySelector(".workspace"));
  }
  wire();
  void boot();
})();
