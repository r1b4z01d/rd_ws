"""Tiny web page for the people the robot knows ("Folks").

Served by the face node itself (stdlib HTTP server in a thread, default port
8096) so it always reflects the live gallery. The VLA manager embeds it as the
"Folks" tab; it also works on its own at http://<robot>:8096/.

Routes:
    GET  /                      the page
    GET  /api/people            [{id, name, seen_count, views, first_seen, last_seen, thumb, thumb_time, last_greeted, next_hello_in}]
    GET  /api/live              {"id", "name", "score"} of the person in view right now (or {})
    GET  /thumb/<id>.jpg        the person's face picture (404 → the page shows a placeholder)
    POST /api/people/<id>/name  {"name": "..."}  rename; a name that already exists merges the two
    POST /api/people/<id>/forget
    POST /api/people/<id>/snap    replace the picture with the next confirmed sighting
    POST /api/people/<id>/reset_greeting   forget the last hello, so the next sighting is greeted again
    POST /api/forget_all
    GET  /api/volume            {"sink", "volume": 0-100, "muted"} of the speaker (audio_sink.py)
    POST /api/volume            {"volume": 0-100} and/or {"muted": bool}; the VLA manager's header slider
"""

from __future__ import annotations

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional
from urllib.parse import unquote

from . import audio_sink

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Folks</title>
<style>
  :root { --bg:#0f1115; --panel:#171a21; --line:#262b36; --fg:#e8eaf0; --muted:#8a93a6; --acc:#8f6bff; --ok:#5bd38a; --warn:#ffb454; --bad:#ff6b6b; }
  * { box-sizing: border-box; } body { margin:0; background:var(--bg); color:var(--fg); font:14px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
  header { display:flex; align-items:center; gap:14px; padding:10px 16px; border-bottom:1px solid var(--line); position:sticky; top:0; background:var(--bg); z-index:2; }
  header h1 { font-size:16px; margin:0; font-weight:600; } header .muted { color:var(--muted); font-size:12px; }
  header .grow { flex:1; } button { background:var(--panel); color:var(--fg); border:1px solid var(--line); border-radius:8px; padding:6px 10px; cursor:pointer; font:inherit; }
  button:hover { border-color:var(--acc); } button.danger:hover { border-color:var(--bad); color:var(--bad); }
  main { padding:16px; display:grid; grid-template-columns:repeat(auto-fill, minmax(230px, 1fr)); gap:14px; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:14px; overflow:hidden; display:flex; flex-direction:column; }
  .card.live { border-color:var(--ok); box-shadow:0 0 0 1px var(--ok) inset; }
  .photo { position:relative; aspect-ratio:1; background:#0a0b0e; }
  .photo img { width:100%; height:100%; object-fit:cover; display:block; }
  .photo .ph { position:absolute; inset:0; display:flex; align-items:center; justify-content:center; color:var(--muted); font-size:48px; }
  .badge { position:absolute; left:10px; top:10px; background:var(--ok); color:#07130b; font-size:11px; font-weight:700; padding:2px 8px; border-radius:999px; }
  .body { padding:10px 12px 12px; display:flex; flex-direction:column; gap:8px; }
  .name { display:flex; gap:6px; } .name input { flex:1; min-width:0; background:#0f1115; color:var(--fg); border:1px solid var(--line); border-radius:8px; padding:6px 8px; font:inherit; font-size:15px; }
  .name input:focus { outline:none; border-color:var(--acc); } .name input.dirty { border-color:var(--warn); }
  .meta { color:var(--muted); font-size:12px; display:grid; grid-template-columns:auto 1fr; gap:3px 12px; } .meta b { color:var(--fg); font-weight:500; text-align:right; }
  .meta .id { font-family:ui-monospace,Menlo,monospace; font-size:11px; }
  .row { display:flex; gap:6px; } .row button { flex:1 1 auto; padding:7px 6px; white-space:nowrap; font-size:12.5px; }
  .empty { grid-column:1/-1; color:var(--muted); padding:40px; text-align:center; }
  .toast { position:fixed; bottom:16px; left:50%; transform:translateX(-50%); background:var(--panel); border:1px solid var(--line); border-radius:10px; padding:8px 14px; font-size:13px; opacity:0; transition:opacity .2s; }
  .toast.show { opacity:1; }
</style></head>
<body>
<header>
  <h1>Folks</h1><span id="count" class="muted">—</span><span class="grow"></span>
  <span id="live" class="muted">nobody in view</span>
  <button id="refresh" title="reload">↻</button>
  <button id="forget-all" class="danger" title="forget every person (cannot be undone)">Forget everyone</button>
</header>
<main id="grid"><div class="empty">loading…</div></main>
<div id="toast" class="toast"></div>
<script>
const $ = (s, r=document) => r.querySelector(s);
let people = [], live = {};
function ago(ts) {
  if (!ts) return "never"; const s = Date.now()/1000 - ts;
  if (s < 60) return "just now"; if (s < 3600) return Math.round(s/60) + " min ago";
  if (s < 86400) return Math.round(s/3600) + " h ago"; return Math.round(s/86400) + " d ago";
}
function nextHello(p) {
  const left = p.next_hello_in || 0;
  if (left <= 0) return "next sighting";
  return left >= 3600 ? Math.round(left/3600) + " h" : Math.max(1, Math.round(left/60)) + " min";
}
function day(ts) { return ts ? new Date(ts*1000).toLocaleDateString(undefined, {month:"short", day:"numeric"}) : "—"; }
function toast(msg, bad) { const t = $("#toast"); t.textContent = msg; t.style.borderColor = bad ? "var(--bad)" : "var(--ok)"; t.classList.add("show"); setTimeout(() => t.classList.remove("show"), 2600); }
async function api(path, body) {
  const r = await fetch(path, body ? {method:"POST", headers:{"Content-Type":"application/json"}, body: JSON.stringify(body)} : {cache:"no-store"});
  const j = await r.json().catch(() => ({})); if (!r.ok) throw new Error(j.error || r.statusText); return j;
}
function render() {
  const g = $("#grid"); const focused = document.activeElement && document.activeElement.tagName === "INPUT";
  if (focused) return;                       // don't yank the field while someone is typing
  $("#count").textContent = people.length ? `${people.length} known` : "nobody learned yet";
  $("#live").textContent = live.id ? `● in view: ${live.name || live.id} (${(live.score||0).toFixed(2)})` : "nobody in view";
  $("#live").style.color = live.id ? "var(--ok)" : "var(--muted)";
  if (!people.length) { g.innerHTML = '<div class="empty">No people learned yet.<br>Stand in front of the camera for a few seconds and a card appears here.</div>'; return; }
  g.innerHTML = "";
  for (const p of people) {
    const card = document.createElement("div"); card.className = "card" + (live.id === p.id ? " live" : "");
    card.innerHTML = `
      <div class="photo">${p.thumb ? `<img src="${p.thumb}?t=${p.thumb_time}" alt="">` : '<div class="ph" title="no picture yet: it is taken the next time this person is recognised">☺</div>'}${live.id === p.id ? '<span class="badge">IN VIEW</span>' : ""}</div>
      <div class="body">
        <div class="name"><input value="${(p.name||"").replace(/"/g,"&quot;")}" placeholder="unnamed — type a name" data-id="${p.id}" spellcheck="false"><button class="save" title="save name">Save</button></div>
        <div class="meta">
          <span>ID</span><b class="id">${p.id}</b>
          <span>First Seen</span><b>${day(p.first_seen)}</b>
          <span>Last Seen</span><b>${ago(p.last_seen)}</b>
          <span>Recognised</span><b>${p.seen_count}×</b>
          <span>Face Views</span><b>${p.views}</b>
          <span>Last Hello</span><b>${p.last_greeted ? ago(p.last_greeted) : "not yet"}</b>
          <span>Next Earliest Hello</span><b>${nextHello(p)}</b>
        </div>
        <div class="row"><button class="reset" title="forget the last hello so the robot greets this person again at the next sighting">👋 Reset hello</button><button class="snap" title="${live.id === p.id ? "take a new picture now" : "take a new picture the next time this person is recognised"}">📷 ${live.id === p.id ? "Snap now" : "Snap"}</button><button class="danger forget" data-id="${p.id}">Forget</button></div>
      </div>`;
    const inp = $("input", card), save = $(".save", card);
    const commit = async () => {
      const name = inp.value.trim(); if (name === (p.name || "")) { inp.classList.remove("dirty"); return; }
      try { const r = await api(`/api/people/${p.id}/name`, {name}); toast(r.message || "saved"); await load(true); }
      catch (e) { toast(e.message, true); }
    };
    inp.addEventListener("input", () => inp.classList.toggle("dirty", inp.value.trim() !== (p.name || "")));
    inp.addEventListener("keydown", (e) => { if (e.key === "Enter") { inp.blur(); } if (e.key === "Escape") { inp.value = p.name || ""; inp.blur(); } });
    inp.addEventListener("blur", commit); save.addEventListener("click", () => inp.blur());
    $(".reset", card).addEventListener("click", async () => {
      try { const r = await api(`/api/people/${p.id}/reset_greeting`, {}); toast(r.message || "ok"); await load(true); } catch (e) { toast(e.message, true); }
    });
    $(".snap", card).addEventListener("click", async () => {
      try { const r = await api(`/api/people/${p.id}/snap`, {}); toast(r.message || "ok"); } catch (e) { toast(e.message, true); }
    });
    $(".forget", card).addEventListener("click", async () => {
      if (!confirm(`Forget ${p.name || p.id}? The robot will treat them as a stranger again.`)) return;
      try { const r = await api(`/api/people/${p.id}/forget`, {}); toast(r.message || "forgotten"); await load(true); } catch (e) { toast(e.message, true); }
    });
    g.appendChild(card);
  }
}
async function load(force) {
  try { people = await api("/api/people"); live = await api("/api/live"); render(); } catch (e) { if (force) toast(e.message, true); }
}
$("#refresh").addEventListener("click", () => load(true));
$("#forget-all").addEventListener("click", async () => {
  if (!confirm("Forget everyone? Every learned face is deleted.")) return;
  try { const r = await api("/api/forget_all", {}); toast(r.message || "done"); await load(true); } catch (e) { toast(e.message, true); }
});
load(true); setInterval(() => load(false), 4000);
</script>
</body></html>
"""


class PeopleWebServer:
    """Serve the Folks page for a :class:`FaceIdentifier`.

    ``live`` returns the current ``Match`` (or None); ``on_change`` is called after a
    rename / forget so the node can reset its "who am I looking at" state.
    """

    def __init__(self, identifier, port: int = 8096, host: str = "0.0.0.0",
                 live: Optional[Callable[[], object]] = None,
                 on_change: Optional[Callable[[], None]] = None, log=None,
                 cooldown: Optional[Callable[[], float]] = None,
                 sink: Optional[Callable[[], str]] = None):
        self.identifier = identifier
        self.sink = sink or (lambda: "")                 # speaker sink pattern (node parameter audio_sink)
        self.cooldown = cooldown or (lambda: 0.0)      # greeting cooldown in seconds (node parameter)
        self.port = int(port)
        self.host = host
        self.live = live or (lambda: None)
        self.on_change = on_change or (lambda: None)
        self._log = log or (lambda m: None)
        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    # -- data ---------------------------------------------------------------
    def people_json(self) -> list:
        g = self.identifier.gallery
        out = []
        cd = float(self.cooldown() or 0.0)
        now = time.time()
        with g.lock:
            for p in sorted(g.people.values(), key=lambda q: q.last_seen, reverse=True):
                tp = g.thumb_path(p.person_id)
                remaining = max(0.0, p.last_greeted + cd - now) if (cd > 0 and p.last_greeted) else 0.0
                out.append({
                    "id": p.person_id, "name": p.name, "seen_count": p.seen_count,
                    "views": len(p.embeddings), "first_seen": p.created, "last_seen": p.last_seen,
                    "thumb": f"/thumb/{p.person_id}.jpg" if tp and os.path.exists(tp) else None,
                    "thumb_time": int(p.thumb_time),
                    "last_greeted": p.last_greeted, "next_hello_in": round(remaining), "cooldown": cd,
                })
        return out

    def live_json(self) -> dict:
        m = self.live()
        if m is None or not getattr(m, "known", False):
            return {}
        return {"id": m.person_id, "name": m.name, "score": round(float(m.score), 3)}

    def rename(self, person_id: str, name: str) -> dict:
        g = self.identifier.gallery
        with g.lock:
            if person_id not in g.people:
                raise KeyError(person_id)
            before = set(g.people)
            person = self.identifier.name_person(person_id, name)
            merged = before - set(g.people)
        self.on_change()
        if merged:
            return {"ok": True, "id": person.person_id, "name": person.name,
                    "message": f"{person_id} merged into {person.name} ({person.person_id})"}
        return {"ok": True, "id": person.person_id, "name": person.name,
                "message": f"{person.person_id} is now {person.name!r}" if person.name else f"{person.person_id} unnamed"}

    def forget(self, person_id: str) -> dict:
        if not self.identifier.forget(person_id):
            raise KeyError(person_id)
        self.on_change()
        return {"ok": True, "message": f"forgot {person_id}"}

    def snap(self, person_id: str) -> dict:
        g = self.identifier.gallery
        with g.lock:
            if person_id not in g.people:
                raise KeyError(person_id)
        self.identifier.request_snapshot(person_id)
        return {"ok": True, "message": f"{g.people[person_id].label}: picture will update on the next recognition"}

    def reset_greeting(self, person_id: str) -> dict:
        g = self.identifier.gallery
        if not g.reset_greeting(person_id):
            raise KeyError(person_id)
        return {"ok": True, "message": f"{g.people[person_id].label} will be greeted at the next sighting"}

    def volume_json(self) -> dict:
        return audio_sink.get_volume(self.sink())

    def set_volume(self, body: dict) -> dict:
        out = None
        if body.get("volume") is not None:
            out = audio_sink.set_volume(self.sink(), float(body["volume"]))
        if body.get("muted") is not None:
            out = audio_sink.set_mute(self.sink(), bool(body["muted"]))
        if out is None:
            raise ValueError("volume or muted required")
        if out.get("error"):
            raise RuntimeError(out["error"])
        out["ok"] = True
        return out

    def forget_all(self) -> dict:
        g = self.identifier.gallery
        with g.lock:
            ids = list(g.people)
        for pid in ids:
            self.identifier.forget(pid)
        self.on_change()
        return {"ok": True, "message": f"forgot {len(ids)} people"}

    # -- lifecycle ------------------------------------------------------------
    def start(self) -> bool:
        web = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "RobotFaceFolks/1.0"

            def log_message(self, fmt, *args):   # keep the node's log clean
                pass

            def _send(self, code: int, body: bytes, ctype: str = "application/json"):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

            def _json(self, code: int, obj):
                self._send(code, json.dumps(obj).encode("utf-8"))

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                try:
                    if path in ("/", "/index.html"):
                        self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
                    elif path == "/api/people":
                        self._json(200, web.people_json())
                    elif path == "/api/live":
                        self._json(200, web.live_json())
                    elif path == "/api/volume":
                        self._json(200, web.volume_json())
                    elif path.startswith("/thumb/") and path.endswith(".jpg"):
                        pid = unquote(path[len("/thumb/"):-4])
                        tp = web.identifier.gallery.thumb_path(pid) if pid in web.identifier.gallery.people else None
                        if tp and os.path.exists(tp):
                            with open(tp, "rb") as f:
                                self._send(200, f.read(), "image/jpeg")
                        else:
                            self._json(404, {"error": "no picture"})
                    else:
                        self._json(404, {"error": "not found"})
                except Exception as e:   # noqa: BLE001
                    self._json(500, {"error": str(e)})

            def do_POST(self):
                path = self.path.split("?", 1)[0]
                n = int(self.headers.get("Content-Length") or 0)
                try:
                    body = json.loads(self.rfile.read(n) or b"{}") if n else {}
                except json.JSONDecodeError:
                    body = {}
                try:
                    parts = [unquote(x) for x in path.strip("/").split("/")]
                    if parts == ["api", "forget_all"]:
                        self._json(200, web.forget_all())
                    elif parts == ["api", "volume"]:
                        self._json(200, web.set_volume(body))
                    elif len(parts) == 4 and parts[:2] == ["api", "people"] and parts[3] == "name":
                        self._json(200, web.rename(parts[2], str(body.get("name", ""))))
                    elif len(parts) == 4 and parts[:2] == ["api", "people"] and parts[3] == "forget":
                        self._json(200, web.forget(parts[2]))
                    elif len(parts) == 4 and parts[:2] == ["api", "people"] and parts[3] == "snap":
                        self._json(200, web.snap(parts[2]))
                    elif len(parts) == 4 and parts[:2] == ["api", "people"] and parts[3] == "reset_greeting":
                        self._json(200, web.reset_greeting(parts[2]))
                    else:
                        self._json(404, {"error": "not found"})
                except KeyError as e:
                    self._json(404, {"error": f"unknown person {e}"})
                except ValueError as e:
                    self._json(400, {"error": str(e)})
                except Exception as e:   # noqa: BLE001
                    self._json(500, {"error": str(e)})

        try:
            self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as e:
            self._log(f"Folks page could not bind {self.host}:{self.port}: {e}")
            return False
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        if self._server is not None:
            try:
                self._server.shutdown()
                self._server.server_close()
            except Exception:
                pass
            self._server = None


__all__ = ["PeopleWebServer", "PAGE"]
