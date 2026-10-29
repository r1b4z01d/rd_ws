// Camera strip for the map / 3D pages: the robot's camera feeds (MJPEG from /cam/<name>.mjpg — the three
// VLA cameras and the RealSense, or the simulator's renders) as thumbnails on the right edge of the
// view; a checkbox per feed in the side panel, click a thumbnail to enlarge it. Unchecked feeds close
// their stream. Choices are remembered per browser.
(function () {
  const STYLE = `
    #cams { position: absolute; right: 10px; display: flex; flex-direction: column; gap: 6px; align-items: flex-end; pointer-events: none; }
    #cams figure.cam { margin: 0; background: rgba(0,0,0,.6); border-radius: 6px; padding: 3px; pointer-events: auto; cursor: zoom-in; }
    #cams figure.cam.big { cursor: zoom-out; }
    #cams figure.cam img { display: block; width: 200px; min-height: 40px; border-radius: 4px; background: #0b0c10; }
    #cams figure.cam.big img { width: 560px; }
    #cams figure.cam figcaption { font-size: 11px; color: #9aa0ad; padding: 2px 3px 0; }
    #cams figure.cam.dead img { opacity: .25; }
    #cams figure.cam.dead figcaption { color: #c93c3c; }
  `;
  window.initCams = function initCams({ view, panel, top = 8 }) {
    const st = document.createElement("style"); st.textContent = STYLE; document.head.appendChild(st);
    const strip = document.createElement("div"); strip.id = "cams"; strip.style.top = top + "px"; view.appendChild(strip);
    const figs = {}; let cams = [];
    let prefs = null; try { prefs = JSON.parse(localStorage.getItem("cams.on") || "null"); } catch (e) { /* no storage */ }
    const save = () => { try { localStorage.setItem("cams.on", JSON.stringify(Object.fromEntries(cams.map((c) => [c.name, figs[c.name].cb.checked])))); } catch (e) { /* ignore */ } };
    function apply(name) {
      const f = figs[name], on = f.cb.checked;
      f.fig.hidden = !on;
      f.fig.classList.toggle("dead", !f.cam.alive);
      f.cap.textContent = f.cam.label + (f.cam.alive ? "" : " · no frames");
      const want = on && f.cam.alive;
      if (want && !f.img.dataset.on) { f.img.src = f.cam.url + "?fps=10"; f.img.dataset.on = "1"; }
      if (!want && f.img.dataset.on) { f.img.removeAttribute("src"); delete f.img.dataset.on; }
    }
    function build(list) {
      cams = list;
      for (const c of list) {
        if (figs[c.name]) { figs[c.name].cam = c; apply(c.name); continue; }
        const lab = document.createElement("label"); lab.className = "cb";
        const cb = document.createElement("input"); cb.type = "checkbox"; cb.checked = prefs ? prefs[c.name] !== false : true;
        lab.append(cb, " " + c.label); panel.appendChild(lab);
        const fig = document.createElement("figure"); fig.className = "cam";
        const img = document.createElement("img"); img.alt = c.label;
        const cap = document.createElement("figcaption"); cap.textContent = c.label;
        fig.append(img, cap); strip.appendChild(fig);
        fig.onclick = () => fig.classList.toggle("big");
        cb.onchange = () => { apply(c.name); save(); };
        figs[c.name] = { cb, fig, img, cap, cam: c };
        apply(c.name);
      }
    }
    async function poll() {
      try { const d = await fetch("/api/cameras").then((r) => r.json()); build(d.cameras || []); } catch (e) { /* server away: keep what we have */ }
      setTimeout(poll, 5000);
    }
    poll();
  };
})();
