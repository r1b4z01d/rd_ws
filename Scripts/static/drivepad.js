// Drive pad for the map / 3D pages: steer the robot from the browser. Three inputs, one setpoint:
//   - a virtual stick (drag with mouse or finger)
//   - WASD / arrow keys
//   - a REAL GAMEPAD plugged into the computer running the browser (Gamepad API)
// all streamed to POST /api/drive -> /cmd_vel_web -> twist_mux "web" (priority 90, below the 8BitDo
// wired to the robot itself, so the pad held at the robot always wins).
//
// This drives a real robot over WiFi, so it is a DEAD MAN, not a latch, with four layers between a
// mistake and a runaway:
//   1. an input only sends while it is held; release, pointer-capture loss, blur, tab-hide, page-hide
//      and gamepad disconnect all send an explicit stop
//   2. the editor republishes a setpoint for 0.4 s only, then zeros and goes quiet (route_editor.py
//      DRIVE_STALE_S) — a dropped link or a closed laptop looks exactly like a release
//   3. twist_mux drops the input 0.5 s after the last message
//   4. the collision monitor still gates the output, so its stop polygon stops the pad too
// The pad must be ARMED before it sends anything, so a stray click cannot drive the robot.
// Space = stop but stay armed, Escape = stop and disarm.
//
// Gamepad notes, from the W3C spec (https://w3c.github.io/gamepad/):
//   - getGamepads() "returns an empty list before a gamepad user gesture has been seen", to stop pages
//     fingerprinting people by their controllers. So a pad that is already plugged in stays invisible
//     until a button is pressed — hence the "press a button" prompt rather than a bug report.
//   - Standard Gamepad layout: axes[0] left stick X (negative left), axes[1] left stick Y (NEGATIVE UP),
//     buttons[4] top left front (LB), buttons[5] top right front (RB). Those are the same indices the
//     robot's own joystick.yaml uses for enable (5) and turbo (4), so the grip is identical.
//   - Pads that report mapping != "standard" number their buttons however they like, which is exactly
//     the drift joystick.yaml documents for the 8BitDo across its two links. Hence "bind": press the
//     button you want as the dead man and it is remembered.
//   - getGamepads() is NOT marked [SecureContext] in the spec; its only gate is the "gamepad"
//     Permissions Policy, whose default allowlist is *. Embedded in an iframe it still needs that
//     policy, so the manager's iframes carry allow="gamepad".
(function () {
  const STYLE = `
    #drivepad { position: absolute; left: 10px; bottom: 10px; z-index: 6; background: rgba(0,0,0,.62); border-radius: 8px; padding: 6px 8px 8px; font-size: 11px; color: #9aa0ad; user-select: none; }
    #drivepad[hidden] { display: none; }
    #drivepad .dp-head { display: flex; align-items: center; gap: 7px; white-space: nowrap; }
    #drivepad .dp-head b { color: #e8e9ee; font-weight: 600; }
    #drivepad .dp-ver { color: #4a4f5c; font-size: 9px; margin-left: -3px; }
    #drivepad label { display: inline-flex; align-items: center; gap: 4px; cursor: pointer; }
    #drivepad input[type=checkbox] { accent-color: #4f8cff; margin: 0; }
    #drivepad .dp-stick { position: relative; width: 116px; height: 116px; margin: 7px auto 0; border-radius: 50%; background: #15171d; border: 1px solid #2a2e38; touch-action: none; }
    #drivepad.armed .dp-stick { border-color: #2ea043; cursor: grab; }
    #drivepad.armed .dp-stick.live { cursor: grabbing; border-color: #f5d033; }
    #drivepad:not(.armed) .dp-stick { opacity: .4; cursor: not-allowed; }
    #drivepad .dp-stick::before, #drivepad .dp-stick::after { content: ""; position: absolute; background: #2a2e38; }
    #drivepad .dp-stick::before { left: 50%; top: 6px; bottom: 6px; width: 1px; }
    #drivepad .dp-stick::after { top: 50%; left: 6px; right: 6px; height: 1px; }
    #drivepad .dp-knob { position: absolute; width: 30px; height: 30px; border-radius: 50%; background: #4f8cff; border: 2px solid #0b0c10; left: 50%; top: 50%; transform: translate(-50%, -50%); pointer-events: none; transition: left .08s, top .08s; }
    #drivepad.armed .dp-stick.live .dp-knob { background: #f5d033; transition: none; }
    #drivepad.gp .dp-knob { background: #2ea043; }
    #drivepad .dp-read { text-align: center; margin-top: 5px; font-family: ui-monospace, Menlo, monospace; font-size: 10px; min-height: 13px; }
    #drivepad .dp-read.live { color: #f5d033; }
    #drivepad .dp-read.err { color: #ff7b7b; max-width: 150px; white-space: normal; }
    #drivepad .dp-gp { margin-top: 5px; max-width: 160px; display: flex; align-items: flex-start; gap: 5px; white-space: pre-line; line-height: 1.35; font-size: 10px; }
    #drivepad .dp-gp-txt { flex: 1; min-width: 0; overflow-wrap: anywhere; font-family: ui-monospace, Menlo, monospace; }
    #drivepad .dp-gp.on { color: #9fe3a6; }
    #drivepad .dp-gp.hold { color: #f5d033; }
    #drivepad .dp-bind { background: #1b1f27; color: #9aa0ad; border: 1px solid #2a2e38; border-radius: 5px; padding: 1px 6px; cursor: pointer; font-size: 10px; flex: none; }
    #drivepad .dp-bind:hover { color: #e8e9ee; border-color: #4f8cff; }
    #drivepad .dp-bind.binding { color: #f5d033; border-color: #f5d033; }
    #drivepad .dp-stop { background: #3a1d1d; color: #ff9a9a; border: 1px solid #5c2b2b; border-radius: 5px; padding: 2px 8px; cursor: pointer; font-size: 11px; }
    #drivepad .dp-stop:hover { background: #512828; }
  `;
  const HZ = 20;                       // matches route_editor.py DRIVE_HZ
  const MAX_VX = 0.25, MAX_WZ = 0.5;   // the robot's normal scales (joystick.yaml); the editor clamps too
  const TURBO = 4;                     // the 8BitDo's LB turbo: 1.0 m/s, 2.0 rad/s
  const DEAD = 0.08;                   // centre deadzone, as joy_node has one
  const GP_DEADMAN = 5, GP_TURBO = 4;  // Standard Gamepad: top right / top left front buttons (RB / LB)
  const KEY_VX = { w: 1, W: 1, ArrowUp: 1, s: -1, S: -1, ArrowDown: -1 };
  const KEY_WZ = { a: -1, A: -1, ArrowLeft: -1, d: 1, D: 1, ArrowRight: 1 };   // ax: +1 = right, like the stick

  // `enabled()` is the page-side switch (the 2D page only shows the pad in map mode, like the camera
  // strip). Returns {refresh} so the page can re-apply it when its mode changes.
  window.initDrivePad = function initDrivePad({ view, panel, enabled = null }) {
    const st = document.createElement("style"); st.textContent = STYLE; document.head.appendChild(st);
    const box = document.createElement("div"); box.id = "drivepad";
    box.innerHTML = `
      <div class="dp-head">
        <label title="arm the pad: nothing reaches the robot until this is on"><input type="checkbox" class="dp-arm"> <b>drive</b></label><span class="dp-ver" title="drive pad build">v3</span>
        <label title="4x the normal scales, the same as the 8BitDo's LB turbo (1.0 m/s, 2.0 rad/s). A gamepad's LB does this while held."><input type="checkbox" class="dp-turbo"> turbo</label>
        <button class="dp-stop" title="stop the robot now and disarm (Escape)">stop</button>
      </div>
      <div class="dp-stick"><div class="dp-knob"></div></div>
      <div class="dp-read">idle</div>
      <div class="dp-gp"><span class="dp-gp-txt"></span><button class="dp-bind" title="press this, then the button on the gamepad you want to hold while driving">bind</button></div>`;
    view.appendChild(box);

    const arm = box.querySelector(".dp-arm"), turbo = box.querySelector(".dp-turbo");
    const stick = box.querySelector(".dp-stick"), knob = box.querySelector(".dp-knob");
    const read = box.querySelector(".dp-read"), stopBtn = box.querySelector(".dp-stop");
    const gpRow = box.querySelector(".dp-gp"), gpTxt = box.querySelector(".dp-gp-txt"), bindBtn = box.querySelector(".dp-bind");
    const keys = new Set();
    let vx = 0, wz = 0;            // the current setpoint
    let fx = 0, fy = 0;            // the axes it came from, kept so turbo can re-scale what is held
    let owner = null;              // "stick" | "keys" | "gamepad": who is driving right now
    let timer = null, inFlight = false, errMsg = "";
    let gpTurbo = false;           // the gamepad's LB, OR-ed with the turbo checkbox

    const driving = () => !!(vx || wz);
    function show() {
      read.classList.toggle("err", !!errMsg);
      read.classList.toggle("live", driving() && !errMsg);
      stick.classList.toggle("live", driving());
      box.classList.toggle("gp", owner === "gamepad");
      read.textContent = errMsg ? errMsg
        : driving() ? `${vx.toFixed(2)} m/s  ${wz.toFixed(2)} rad/s`
        : arm.checked ? "armed" : "idle";
    }
    function knobTo(ax, ay) { knob.style.left = `${50 + ax * 34}%`; knob.style.top = `${50 - ay * 34}%`; }

    async function post(stop) {
      if (inFlight) return;        // one request at a time; the ticker retries with the latest value
      inFlight = true;
      try {
        const r = await fetch("/api/drive", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify(stop ? { stop: true } : { vx, wz, turbo: turbo.checked || gpTurbo }),
        });
        const d = await r.json().catch(() => ({}));
        errMsg = r.ok ? "" : (d.error || `drive: ${r.status}`);
      } catch (e) {
        errMsg = "drive: no connection";
      } finally { inFlight = false; show(); }
    }

    // One ticker runs while the setpoint is non-zero, renewing the editor's 0.4 s window. It is also
    // what ends a drive: once the axes fall to zero it sends a single stop and clears itself.
    function tick() {
      if (!arm.checked) { stop(); return; }
      if (!driving()) { clearInterval(timer); timer = null; post(true); show(); return; }
      post(false);
    }
    // One input owns the setpoint at a time, so letting go of the gamepad cannot be masked by a stale
    // key, and a key cannot steal the robot mid-drag.
    function setAxes(ax, ay, src) {
      // While one input owns the setpoint every other input is ignored outright, zeros included:
      // letting a key-up through here would zero a drag that the on-screen stick is still holding.
      if (owner && owner !== src) return;
      const squash = (f) => (Math.abs(f) < DEAD ? 0 : (f - Math.sign(f) * DEAD) / (1 - DEAD));
      const t = (turbo.checked || gpTurbo) ? TURBO : 1;
      fx = ax; fy = ay;
      // ax follows the screen: +1 = right (the on-screen stick, the Gamepad spec's axes[0], and the keys
      // all agree). The robot's yaw is counter-clockwise positive, so a push to the RIGHT is a NEGATIVE
      // wz — the sign was missing and left/right came out mirrored on the stick and the gamepad.
      vx = +(squash(ay) * MAX_VX * t).toFixed(3);    // forward = up
      wz = +(-squash(ax) * MAX_WZ * t).toFixed(3);   // right = clockwise = negative yaw rate
      knobTo(ax, ay);
      if (!arm.checked) return;
      if (driving()) { owner = src; if (!timer) timer = setInterval(tick, 1000 / HZ); post(false); }
      else if (owner === src) { stop(); return; }
      show();
    }
    // `tell` is false for the paths where nothing was ever sent (arming, showing the pad): there is no
    // point reporting "the ROS node is not up" for a stop the robot never needed.
    function stop(tell = true) {
      const was = driving() || timer;
      vx = wz = fx = fy = 0;
      owner = null;
      keys.clear();
      knobTo(0, 0);
      if (timer) { clearInterval(timer); timer = null; }
      if (was && tell) post(true); else { errMsg = ""; show(); }
    }

    // ---- the on-screen stick. Pointer capture keeps the drag alive outside the circle; losing that
    // capture for any reason (up, cancel, the element going away) is the release.
    stick.addEventListener("pointerdown", (e) => {
      if (!arm.checked || (owner && owner !== "stick")) return;
      stick.setPointerCapture(e.pointerId);
      fromPointer(e);
    });
    stick.addEventListener("pointermove", (e) => { if (stick.hasPointerCapture(e.pointerId)) fromPointer(e); });
    stick.addEventListener("pointerup", (e) => { if (stick.hasPointerCapture(e.pointerId)) stick.releasePointerCapture(e.pointerId); });
    stick.addEventListener("lostpointercapture", () => { if (owner === "stick") stop(); });
    function fromPointer(e) {
      const r = stick.getBoundingClientRect();
      let ax = (e.clientX - r.left - r.width / 2) / (r.width / 2);
      let ay = -(e.clientY - r.top - r.height / 2) / (r.height / 2);
      const m = Math.hypot(ax, ay);
      if (m > 1) { ax /= m; ay /= m; }   // clamp into the circle: a corner drag is no faster
      setAxes(ax, ay, "stick");
    }

    // ---- keyboard, while armed and not typing in a field
    const typing = (el) => el && (["INPUT", "TEXTAREA", "SELECT"].includes(el.tagName) || el.isContentEditable);
    window.addEventListener("keydown", (e) => {
      if (box.hidden || typing(document.activeElement) || e.metaKey || e.ctrlKey || e.altKey) return;
      if (e.key === "Escape") { arm.checked = false; box.classList.remove("armed"); stop(); return; }
      if (!arm.checked) return;
      if (e.key === " ") { stop(); e.preventDefault(); return; }
      if (!(e.key in KEY_VX) && !(e.key in KEY_WZ)) return;
      if (!e.repeat) { keys.add(e.key); fromKeys(); }
      e.preventDefault();
    });
    window.addEventListener("keyup", (e) => { if (keys.delete(e.key)) fromKeys(); });
    function fromKeys() {
      let ay = 0, ax = 0;
      for (const k of keys) { ay += KEY_VX[k] || 0; ax += KEY_WZ[k] || 0; }
      setAxes(Math.max(-1, Math.min(1, ax)), Math.max(-1, Math.min(1, ay)), "keys");
    }

    // ---- a real gamepad on the computer running this browser -------------------------------------
    let deadman = GP_DEADMAN, binding = false, gpWas = false, gpName = "", gpWarn = "";
    try { const v = parseInt(localStorage.getItem("drivepad.deadman"), 10); if (v >= 0) deadman = v; } catch (e) { /* storage blocked */ }

    const pads = () => {
      try { return (navigator.getGamepads ? navigator.getGamepads() : []) || []; }
      catch (e) { gpWarn = "gamepad blocked: " + (e.name || e); return []; }   // Permissions Policy
    };
    const firstPad = () => { for (const p of pads()) if (p && p.connected) return p; return null; };

    function gpShow(p, held) {
      gpRow.classList.toggle("on", !!p && !held);
      gpRow.classList.toggle("hold", !!held);
      if (!navigator.getGamepads) { gpTxt.textContent = "✗ no Gamepad API in this browser"; return; }
      if (gpWarn) { gpTxt.textContent = "✗ " + gpWarn; return; }
      if (!p) {
        // Indistinguishable from here: nothing plugged in, or plugged in but not yet "seen". The spec
        // hides a pad until a button is pressed with the page in front, and in the manager this page is
        // an iframe, so the click that gives it focus matters too.
        gpTxt.textContent = "○ no gamepad yet — click here, then press a button on the pad";
        return;
      }
      if (binding) { gpTxt.textContent = "press the button to use as the dead man…"; return; }
      const nm = (p.id || "pad").replace(/\s*\(.*?\)\s*/g, " ").trim().slice(0, 24);
      // A live readout, so "does the page see the pad?" is answered at a glance: the axes move as the
      // stick moves and every pressed button index is listed.
      const down = [];
      for (let i = 0; i < p.buttons.length; i++) {
        if (p.buttons[i] && (p.buttons[i].pressed || p.buttons[i].value > 0.5)) down.push(i);
      }
      const ax = (p.axes[0] || 0).toFixed(2), ay = (p.axes[1] || 0).toFixed(2);
      gpTxt.textContent = `✓ ${nm}\naxes ${ax}, ${ay} · btn ${down.length ? down.join(",") : "—"}`
        + `\n${held ? "HOLDING " + deadman + " — driving" : "hold button " + deadman + " to drive"}`
        + (p.mapping === "standard" ? "" : "\nnon-standard mapping: bind if wrong");
    }

    function gpPoll() {
      if (box.hidden) return;
      const p = firstPad();
      if (!p) {
        if (gpWas && owner === "gamepad") stop();     // unplugged mid-drive
        gpWas = false; gpShow(null, false); return;
      }
      gpWas = true;
      const btn = (i) => !!(p.buttons[i] && (p.buttons[i].pressed || p.buttons[i].value > 0.5));
      if (binding) {                                   // learn whichever button is pressed next
        for (let i = 0; i < p.buttons.length; i++) {
          if (btn(i)) {
            deadman = i; binding = false; bindBtn.classList.remove("binding");
            try { localStorage.setItem("drivepad.deadman", String(i)); } catch (e) { /* storage blocked */ }
            break;
          }
        }
        gpShow(p, false); return;
      }
      const held = btn(deadman);
      gpShow(p, held);
      if (!arm.checked) return;
      if (!held) { if (owner === "gamepad") stop(); gpTurbo = false; return; }
      if (owner && owner !== "gamepad") return;        // the screen stick or a key is driving
      gpTurbo = btn(GP_TURBO);
      // axes[1] is negative UP per the Standard Gamepad table, so forward is -axes[1]
      const ax = p.axes.length > 0 ? p.axes[0] : 0, ay = p.axes.length > 1 ? -p.axes[1] : 0;
      setAxes(ax, ay, "gamepad");
      if (!driving()) show();                          // held but centred: keep the ✓ visible
    }
    window.addEventListener("gamepadconnected", () => { gpWarn = ""; gpShow(firstPad(), false); });
    window.addEventListener("gamepaddisconnected", () => { if (owner === "gamepad") stop(); gpShow(null, false); });
    bindBtn.addEventListener("click", () => { binding = !binding; bindBtn.classList.toggle("binding", binding); gpShow(firstPad(), false); });
    setInterval(gpPoll, 1000 / HZ);

    // ---- anything that takes the page away from whoever is driving counts as a release
    for (const ev of ["blur", "pagehide"]) window.addEventListener(ev, () => { if (driving()) stop(); });
    document.addEventListener("visibilitychange", () => { if (document.hidden && driving()) stop(); });
    arm.addEventListener("change", () => { box.classList.toggle("armed", arm.checked); stop(arm.checked === false && driving()); });
    turbo.addEventListener("change", () => { if (driving()) setAxes(fx, fy, owner); });   // re-scale what is held
    stopBtn.addEventListener("click", () => { arm.checked = false; box.classList.remove("armed"); stop(); });

    // ---- the side-panel checkbox that shows the pad at all
    const lbl = document.createElement("label");
    lbl.className = "cb";
    lbl.title = "show the drive pad: steer the robot from this page with the on-screen stick, WASD, or a gamepad plugged into this computer (it still has to be armed)";
    lbl.innerHTML = `<input type="checkbox" class="dp-show"> drive pad`;
    if (panel) panel.appendChild(lbl);
    const showBox = lbl.querySelector("input");
    try { showBox.checked = localStorage.getItem("drivepad.show") === "1"; } catch (e) { /* storage blocked */ }
    function apply() {
      const on = showBox.checked && (!enabled || enabled());
      box.hidden = !on;
      if (!on) { const was = driving(); arm.checked = false; box.classList.remove("armed"); stop(was); }
      try { localStorage.setItem("drivepad.show", showBox.checked ? "1" : "0"); } catch (e) { /* storage blocked */ }
      if (on) gpShow(firstPad(), false);
      show();
    }
    showBox.addEventListener("change", apply);
    apply();
    return { refresh: apply };
  };
})();
