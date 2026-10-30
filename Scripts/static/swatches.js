// Press-and-hold a colour swatch (`.sw[data-key]`) to pick that layer's colour. A short click still
// toggles the checkbox the swatch sits in. Choices are remembered per page and per browser.
(function () {
  const HOLD_MS = 450;
  const toHex = (css) => {
    const m = /rgba?\((\d+),\s*(\d+),\s*(\d+)/.exec(css || "");
    return m ? "#" + [m[1], m[2], m[3]].map((v) => (+v).toString(16).padStart(2, "0")).join("") : null;
  };
  const st = document.createElement("style");
  st.textContent = ".sw[data-key] { cursor: pointer; transition: transform .12s; } .sw[data-key].holding { transform: scale(1.6); box-shadow: 0 0 0 2px rgba(255,255,255,.5); }";
  document.head.appendChild(st);
  window.initSwatches = function initSwatches({ page, apply, defaults = {} }) {
    const key = (k) => `colors.${page}.${k}`;
    const load = (k) => { try { return localStorage.getItem(key(k)); } catch (e) { return null; } };
    const save = (k, v) => { try { if (v) localStorage.setItem(key(k), v); else localStorage.removeItem(key(k)); } catch (e) { /* ignore */ } };
    const picker = document.createElement("input");
    picker.type = "color"; picker.style.cssText = "position:fixed;width:1px;height:1px;opacity:0;pointer-events:none";
    document.body.appendChild(picker);
    let target = null;
    picker.addEventListener("input", () => { if (!target) return; target.sw.style.background = picker.value; apply(target.k, picker.value); save(target.k, picker.value); });
    document.querySelectorAll(".sw[data-key]").forEach((sw) => {
      const k = sw.dataset.key;
      const saved = load(k);
      if (saved) { sw.style.background = saved; apply(k, saved); }
      let timer = null, held = false;
      const open = () => {
        held = true; target = { sw, k };
        picker.value = toHex(getComputedStyle(sw).backgroundColor) || defaults[k] || "#ffffff";
        const r = sw.getBoundingClientRect(); picker.style.left = r.left + "px"; picker.style.top = r.bottom + 4 + "px";
        try { picker.showPicker ? picker.showPicker() : picker.click(); } catch (e) { picker.click(); }
      };
      const cancel = () => { clearTimeout(timer); sw.classList.remove("holding"); };
      sw.addEventListener("pointerdown", (e) => { if (e.button !== 0) return; held = false; sw.classList.add("holding"); timer = setTimeout(() => { sw.classList.remove("holding"); open(); }, HOLD_MS); });
      sw.addEventListener("pointerup", cancel); sw.addEventListener("pointerleave", cancel); sw.addEventListener("pointercancel", cancel);
      sw.addEventListener("click", (e) => { if (held) { e.preventDefault(); e.stopPropagation(); held = false; } });   // a hold must not toggle the checkbox
      sw.title = (sw.title ? sw.title + " · " : "") + "hold to change the colour";
    });
  };
})();
