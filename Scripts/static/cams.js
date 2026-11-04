// Camera strip for the map / 3D pages: the robot's camera feeds (MJPEG from /cam/<name>.mjpg — the three
// VLA cameras and the RealSense, or the simulator's renders) as panels on the right edge of the view;
// a checkbox per feed in the side panel. Each panel folds on a click of its caption, resizes by dragging
// its bottom handle (the aspect ratio is kept), and a click on the picture toggles a big view. Unchecked
// or folded feeds close their stream. Choices and sizes are remembered per browser.
(function () {
  const STYLE = `
    #cams { position: absolute; right: 10px; display: flex; flex-direction: column; gap: 6px; align-items: flex-end; pointer-events: none; }
    #cams figure.cam { margin: 0; background: rgba(0,0,0,.6); border-radius: 6px; padding: 3px; pointer-events: auto; display: flex; flex-direction: column; }
    #cams figure.cam[hidden] { display: none; }   /* unchecked in the side panel: the flex rule above would otherwise beat the hidden attribute */
    #cams figure.cam figcaption { font-size: 11px; color: #9aa0ad; padding: 2px 3px; cursor: pointer; user-select: none; display: flex; align-items: center; gap: 5px; border-radius: 4px; }
    #cams figure.cam figcaption:hover { color: #e8e9ee; background: rgba(255,255,255,.08); }
    #cams figure.cam figcaption::before { content: "▾"; font-size: 10px; width: 8px; }
    #cams figure.cam.closed figcaption::before { content: "▸"; }
    #cams figure.cam .box { position: relative; width: 200px; height: 150px; min-height: 40px; overflow: hidden; border-radius: 4px; background: #0b0c10; cursor: zoom-in; }
    #cams figure.cam.big .box { cursor: zoom-out; }
    #cams figure.cam.closed .box, #cams figure.cam.closed .grip { display: none; }
    #cams figure.cam .box img { display: block; width: 100%; height: 100%; object-fit: contain; }
    #cams figure.cam .grip { height: 7px; margin: 2px 0 0; border-radius: 3px; cursor: ns-resize; background: repeating-linear-gradient(90deg, #4a4f5c 0 3px, transparent 3px 6px) center / 24px 2px no-repeat; }
    #cams figure.cam .grip:hover, #cams figure.cam.resizing .grip { background-color: rgba(255,255,255,.12); }
    #cams figure.cam.dead .box img { opacity: .25; }
    #cams figure.cam.dead figcaption { color: #c93c3c; }
  `;
  const H_DEFAULT = 150, H_BIG = 420, H_MIN = 40, H_MAX = 900;
  window.initCams = function initCams({ view, panel, top = 8, enabled = null }) {   // enabled(): page-side switch (e.g. map mode only)
    const st = document.createElement("style"); st.textContent = STYLE; document.head.appendChild(st);
    const strip = document.createElement("div"); strip.id = "cams"; strip.style.top = top + "px"; view.appendChild(strip);
    const figs = {}; let cams = [];
    // the manager keeps a hidden tab's iframe alive: with no size we are not on screen, so drop the
    // streams (8 open streams across two hidden-or-not pages would exhaust the connection pool)
    const onScreen = () => window.innerWidth > 0 && window.innerHeight > 0 && (!enabled || enabled());
    let shown = onScreen();
    setInterval(() => { const now = onScreen(); if (now !== shown) { shown = now; for (const n of Object.keys(figs)) apply(n); } }, 1000);
    const load = (k, dflt) => { try { return JSON.parse(localStorage.getItem(k) || "null") ?? dflt; } catch (e) { return dflt; } };
    const store = (k, v) => { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) { /* ignore */ } };
    const prefs = load("cams.on", null), closed = load("cams.closed", {}), sizes = load("cams.size", {});
    const save = () => store("cams.on", Object.fromEntries(cams.map((c) => [c.name, figs[c.name].cb.checked])));
    function apply(name) {
      const f = figs[name], on = f.cb.checked, open = !f.fig.classList.contains("closed");
      f.fig.hidden = !on;
      f.fig.classList.toggle("dead", !f.cam.alive);
      f.cap.textContent = f.cam.label + (f.cam.alive ? "" : " · no frames");
      const want = on && open && f.cam.alive && shown;
      if (want && !f.img.dataset.on) { f.img.src = f.cam.url + "?fps=10"; f.img.dataset.on = "1"; }
      if (!want && f.img.dataset.on) { f.img.removeAttribute("src"); delete f.img.dataset.on; }
    }
    // panel size: the height is what you set; the width follows the frame's aspect ratio (4:3 until a frame arrived)
    function size(f, h, persist) {
      h = Math.max(H_MIN, Math.min(H_MAX, Math.round(h)));
      const ar = f.img.naturalWidth && f.img.naturalHeight ? f.img.naturalWidth / f.img.naturalHeight : 4 / 3;
      f.box.style.height = h + "px"; f.box.style.width = Math.round(h * ar) + "px";
      f.h = h; f.fig.classList.toggle("big", h >= H_BIG);
      if (persist) { sizes[f.cam.name] = h; store("cams.size", sizes); }
    }
    function build(list) {
      cams = list;
      for (const c of list) {
        if (figs[c.name]) { figs[c.name].cam = c; apply(c.name); continue; }
        const lab = document.createElement("label"); lab.className = "cb";
        const cb = document.createElement("input"); cb.type = "checkbox"; cb.checked = prefs ? prefs[c.name] !== false : true;
        lab.append(cb, " " + c.label); panel.appendChild(lab);
        const fig = document.createElement("figure"); fig.className = "cam"; fig.classList.toggle("closed", !!closed[c.name]);
        const cap = document.createElement("figcaption"); cap.textContent = c.label; cap.title = "click to fold / unfold this feed";
        const box = document.createElement("div"); box.className = "box"; box.title = "click to enlarge / shrink · drag the handle below to resize";
        const img = document.createElement("img"); img.alt = c.label;
        const grip = document.createElement("div"); grip.className = "grip"; grip.title = "drag to resize";
        box.appendChild(img); fig.append(cap, box, grip); strip.appendChild(fig);
        const f = figs[c.name] = { cb, fig, box, img, cap, cam: c, h: H_DEFAULT };
        size(f, sizes[c.name] || H_DEFAULT, false);
        img.onload = () => size(f, f.h, false);                       // first frame: adopt the real aspect ratio
        cap.onclick = () => { fig.classList.toggle("closed"); closed[c.name] = fig.classList.contains("closed"); store("cams.closed", closed); apply(c.name); };
        box.onclick = () => size(f, f.h >= H_BIG ? H_DEFAULT : H_BIG, true);
        grip.onpointerdown = (e) => {                                  // vertical drag = resize
          e.preventDefault(); grip.setPointerCapture(e.pointerId); fig.classList.add("resizing");
          const y0 = e.clientY, h0 = f.h;
          grip.onpointermove = (ev) => size(f, h0 + (ev.clientY - y0), false);
          grip.onpointerup = grip.onpointercancel = () => { grip.onpointermove = null; fig.classList.remove("resizing"); size(f, f.h, true); };
        };
        cb.onchange = () => { apply(c.name); save(); };
        apply(c.name);
      }
    }
    async function poll() {
      try { const d = await fetch("/api/cameras").then((r) => r.json()); build(d.cameras || []); } catch (e) { /* server away: keep what we have */ }
      setTimeout(poll, 5000);
    }
    // after the page's own first loads (scripts, map, model): the streams are long-lived connections
    if (document.readyState === "complete") setTimeout(poll, 800); else window.addEventListener("load", () => setTimeout(poll, 800));
  };
})();
