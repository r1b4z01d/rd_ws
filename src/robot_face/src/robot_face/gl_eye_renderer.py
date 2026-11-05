"""Hardware-accelerated eye renderer — moderngl + GLFW + GLSL fragment shader."""

from __future__ import annotations

import time

import cv2
import glfw
import moderngl
import numpy as np

# ---------------------------------------------------------------------------
# Vertex shader — fullscreen quad, passes through position
# ---------------------------------------------------------------------------
_VERT = """
#version 330 core
in vec2 in_position;
void main() {
    gl_Position = vec4(in_position, 0.0, 1.0);
}
"""

# ---------------------------------------------------------------------------
# Fragment shader
# ---------------------------------------------------------------------------
_FRAG = """
#version 330 core

uniform vec2  u_resolution;     // design canvas size (pixels) — all layout is in these units
uniform vec2  u_viewport;       // actual framebuffer size (pixels)
uniform float u_scale;          // design px -> framebuffer px (letterbox scale)
uniform vec2  u_offset;         // letterbox offset in framebuffer px (top-left origin)
uniform vec2  u_eye_left;       // eye center in pixel coords (Y=down)
uniform vec2  u_eye_right;
uniform float u_eye_hw;         // half-width  (pixels)
uniform float u_eye_hh_base;    // half-height when fully open (pixels)
uniform float u_blink;          // 0 = open, 1 = closed (both eyes)
uniform float u_wink;           // 0 = open, 1 = closed, for the winking eye only
uniform int   u_wink_eye;       // 0 = left eye (screen left), 1 = right eye
uniform vec2  u_gaze;           // pupil offset (pixels, Y=down)
uniform int   u_mood;           // 0=neutral 1=tired 2=angry 3=happy 4=suspicious
uniform float u_time;
uniform sampler2D u_hand_tex;       // hand skeleton panel (RGB, black background)
uniform float u_hand_panel_frac;    // fraction of screen height used for panel (0 = hidden)

// --- mouth waveform ---
uniform vec2  u_mouth_center;   // waveform centre in pixel coords (Y=down)
uniform float u_mouth_hw;       // half-width of the waveform (pixels)
uniform float u_mouth_amp;      // peak vertical displacement at level 1.0 (pixels)
uniform float u_mouth_level;    // 0 = silent (flat line) .. 1 = full loudness
uniform int   u_mouth_mode;     // 0 = hidden, 1 = procedural from level, 2 = sampled audio
uniform float u_mouth_smile;    // upward curl of the ends in pixels (negative = frown)
uniform float u_mouth_skew;     // asymmetry in pixels: right end up / left end down
uniform sampler2D u_mouth_tex;  // Nx1 R32F audio samples in [-1, 1]
uniform float u_mouth_shape;    // 0 = resting curve / waveform, 1 = polyline shape below
uniform int   u_mouth_path_n;   // number of valid points in u_mouth_path
uniform sampler2D u_mouth_path; // Kx1 RG32F polyline points (pixels, relative to mouth centre)

out vec4 frag_color;

// --- palette (linear RGB) ---
const vec3 C_BASE = vec3(0.15, 0.03, 0.30);   // dark purple fill
const vec3 C_LINE = vec3(0.80, 0.20, 1.00);   // bright electric-purple scanlines
const vec3 C_GLOW = vec3(0.85, 0.25, 1.00);   // bloom / outer glow
const vec3 C_RIM  = vec3(0.90, 0.65, 1.00);   // bright lavender rim highlight
const vec3 C_MOUTH_CORE = vec3(0.95, 0.80, 1.00);   // waveform line
const vec3 C_MOUTH_GLOW = vec3(0.85, 0.25, 1.00);   // waveform halo

// Rounded-rectangle signed distance field (negative = inside)
float sdRoundRect(vec2 p, vec2 half_size, float r) {
    vec2 q = abs(p) - half_size + r;
    return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - r;
}

// Right-angle triangle corner cuts (eyelids for the moods).
// Each returns the SIGNED DISTANCE in pixels to the cut's diagonal edge: positive inside the
// cut-away triangle, negative on the kept side. renderEye folds it into the eye's own SDF with
// max(), so the lid line becomes part of the eye edge and the fill, the outer glow and the rim all
// stop there. (A 0/1 step here only masked the fill: the glow was still computed from the uncut
// rectangle, so a squinting eye kept showing its full square in glow colour.)
// hs  = eye half-size (hw, hh)
// cs  = cut size (width, height) measured inward from that corner
float cutTL(vec2 p, vec2 hs, vec2 cs) {          // top-left
    vec2 d = p - vec2(-hs.x, -hs.y);
    return (1.0 - (d.x / cs.x + d.y / cs.y)) / length(1.0 / cs);
}
float cutTR(vec2 p, vec2 hs, vec2 cs) {          // top-right
    vec2 d = p - vec2( hs.x, -hs.y);
    return (1.0 - (-d.x / cs.x + d.y / cs.y)) / length(1.0 / cs);
}
float cutBL(vec2 p, vec2 hs, vec2 cs) {          // bottom-left
    vec2 d = p - vec2(-hs.x,  hs.y);
    return (1.0 - (d.x / cs.x - d.y / cs.y)) / length(1.0 / cs);
}
float cutBR(vec2 p, vec2 hs, vec2 cs) {          // bottom-right
    vec2 d = p - vec2( hs.x,  hs.y);
    return (1.0 - (-d.x / cs.x - d.y / cs.y)) / length(1.0 / cs);
}

// Signed distance (px) to the current mood's eyelid cuts; positive = cut away.
float moodMask(vec2 p, vec2 hs, int mood) {
    vec2 ew = hs * 2.0;
    if (mood == 1) {                                       // TIRED — shallow top-outer
        vec2 cs = vec2(ew.x * 0.40, ew.y * 0.50);
        return max(cutTL(p, hs, cs), cutTR(p, hs, cs));
    } else if (mood == 2) {                                // ANGRY — steep top-inner
        vec2 cs = vec2(ew.x * 0.35, ew.y * 0.60);
        return max(cutTL(p, hs, cs), cutTR(p, hs, cs));
    } else if (mood == 3) {                                // HAPPY — bottom squint
        vec2 cs = vec2(ew.x * 0.35, ew.y * 0.55);
        return max(cutBL(p, hs, cs), cutBR(p, hs, cs));
    } else if (mood == 4) {                                // SUSPICIOUS
        vec2 ct = vec2(ew.x * 0.30, ew.y * 0.45);
        vec2 cb = vec2(ew.x * 0.20, ew.y * 0.30);
        return max(
            max(cutTL(p, hs, ct), cutTR(p, hs, ct)),
            max(cutBL(p, hs, cb), cutBR(p, hs, cb))
        );
    }
    return -1.0e5;                                         // neutral: no cut anywhere
}

vec3 renderEye(vec2 frag, vec2 center, float extra_close) {
    float hh = u_eye_hh_base * max(0.0, 1.0 - max(u_blink, extra_close));
    if (hh < 1.0) return vec3(0.0);

    vec2 hs       = vec2(u_eye_hw, hh);
    float corner_r = min(hs.x, hs.y) * 0.18;

    // Local coords: center of eye = origin, gaze shifts the pattern
    vec2 p = frag - center - u_gaze;

    // Eye shape = rounded rect with the mood's eyelid cuts folded into the same signed distance,
    // so everything below (fill, glow, rim) sees the squinted outline, not the full rectangle.
    float sdf    = max(sdRoundRect(p, hs, corner_r), moodMask(p, hs, u_mood));
    float inside = smoothstep(1.5, -1.5, sdf);           // 1 inside, 0 outside

    // --- wavy horizontal scanlines ---
    // Primary wave: horizontal with lateral sine displacement
    float wave = sin(p.y * 0.88 + sin(p.x * 0.020) * 3.2 + u_time * 0.7);
    float scan = wave * 0.28 + 0.72;                     // range [0.44, 1.00]

    // --- radial dimming toward the edges (keeps centre bright) ---
    vec2  np     = p / (hs + 1.0);
    float radial = 1.0 - clamp(dot(np, np), 0.0, 1.0) * 0.50;

    // --- interior colour ---
    vec3 col = mix(C_BASE, C_LINE, scan * radial) * inside;

    // --- outer bloom (fades with distance from edge, stronger just outside) ---
    float glow = exp(-max(sdf, 0.0) * 0.032) * (1.0 - inside * 0.82);
    col += C_GLOW * glow * 0.80;

    // --- top-edge rim highlight ---
    float rim_y  = smoothstep(4.0, 0.0, p.y + hs.y);
    float rim_x  = smoothstep(hs.x - corner_r + 2.0, hs.x - corner_r - 3.0, abs(p.x));
    float rim    = rim_y * rim_x * inside;
    col += C_RIM * rim * 1.0;

    return col;
}

// Waveform height (pixels, Y=down) at horizontal offset x from the mouth centre.
float mouthWave(float x) {
    float nx  = x / u_mouth_hw;                          // -1 .. 1 across the mouth
    float lvl = clamp(u_mouth_level, 0.0, 1.0);
    float y;
    if (u_mouth_mode == 2) {
        y = texture(u_mouth_tex, vec2(nx * 0.5 + 0.5, 0.5)).r;
    } else {
        // Three travelling sines: a slow carrier plus two faster harmonics.
        float t = u_time;
        y = sin(x * 0.045 + t * 9.0)  * 0.55
          + sin(x * 0.110 - t * 13.0) * 0.30
          + sin(x * 0.023 + t * 5.0)  * 0.15;
        y *= lvl;                                        // perfectly still when silent
    }
    // Window: full amplitude in the middle, pinched to a flat line at both ends.
    float win = 1.0 - smoothstep(0.45, 1.0, abs(nx));
    win *= 0.55 + 0.45 * (1.0 - nx * nx);
    // Resting shape: a parabola whose ends curl up by u_mouth_smile (Y is down,
    // so negative = up), plus a linear tilt for smirks.
    float rest = -u_mouth_smile * nx * nx + u_mouth_skew * nx;
    return rest + y * u_mouth_amp * win;
}

float sdSegment(vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a, ba = b - a;
    float h = clamp(dot(pa, ba) / max(dot(ba, ba), 1e-6), 0.0, 1.0);
    return length(pa - ba * h);
}

// Distance from p to the polyline stored in u_mouth_path
float mouthPathDist(vec2 p) {
    float d = 1e9;
    for (int i = 0; i < u_mouth_path_n - 1; i++) {
        vec2 a = texelFetch(u_mouth_path, ivec2(i, 0), 0).xy;
        vec2 b = texelFetch(u_mouth_path, ivec2(i + 1, 0), 0).xy;
        d = min(d, sdSegment(p, a, b));
    }
    return d;
}

vec3 renderMouth(vec2 frag) {
    if (u_mouth_mode == 0) return vec3(0.0);             // uniform branch: safe for dFdx below

    vec2  p  = frag - u_mouth_center;
    float nx = p.x / u_mouth_hw;

    float wy    = mouthWave(p.x);
    float dv    = abs(p.y - wy);                          // plain vertical distance
    // Divide by the local slope so steep sections keep the same stroke thickness.
    // Only valid close to the curve, so it is used for the core stroke alone.
    float slope = dFdx(wy);
    float d     = dv / sqrt(1.0 + slope * slope);

    if (abs(nx) > 1.25) return vec3(0.0);

    // Morph toward an arbitrary shape (gestures) by blending distance fields
    if (u_mouth_shape > 0.001 && u_mouth_path_n > 1) {
        float dp = mouthPathDist(p);
        d  = mix(d,  dp, u_mouth_shape);
        dv = mix(dv, dp, u_mouth_shape);
    }

    float lvl   = clamp(u_mouth_level, 0.0, 1.0);
    float ends  = 1.0 - smoothstep(0.90, 1.08, abs(nx));
    float width = 1.6 + 1.4 * lvl;                        // stroke thickens as it gets louder
    float core  = smoothstep(width + 1.2, width - 0.6, d);
    float glow  = exp(-dv * 0.10) * (0.35 + 0.65 * lvl);

    return (C_MOUTH_CORE * core + C_MOUTH_GLOW * glow * 0.8) * ends;
}

void main() {
    // OpenGL origin is bottom-left; flip Y to match our top-left pixel layout,
    // then map framebuffer pixels back to the design canvas (fullscreen letterboxing).
    vec2 fb   = vec2(gl_FragCoord.x, u_viewport.y - gl_FragCoord.y);
    vec2 frag = (fb - u_offset) / u_scale;
    // 1 inside the canvas, 0 in the letterbox bars (a multiply, not a return,
    // keeps the dFdx inside renderMouth in uniform control flow)
    float on_canvas = step(0.0, frag.x) * step(0.0, frag.y)
                    * step(frag.x, u_resolution.x) * step(frag.y, u_resolution.y);

    vec3 col = renderEye(frag, u_eye_left,  u_wink_eye == 0 ? u_wink : 0.0)
             + renderEye(frag, u_eye_right, u_wink_eye == 1 ? u_wink : 0.0);
    col += renderMouth(frag);

    // Hand skeleton panel — bottom u_hand_panel_frac of the screen
    if (u_hand_panel_frac > 0.01) {
        float panel_h     = u_resolution.y * u_hand_panel_frac;
        float panel_start = u_resolution.y - panel_h;
        if (frag.y >= panel_start) {
            float px       = frag.x / u_resolution.x;
            float py       = (frag.y - panel_start) / panel_h;
            vec3 hand_col  = texture(u_hand_tex, vec2(px, py)).rgb;
            col += hand_col;   // additive over black background
        }
    }

    // Subtle full-canvas vignette
    vec2  uv  = frag / u_resolution - 0.5;
    float vig = 1.0 - dot(uv, uv) * 1.6;
    col *= clamp(vig, 0.0, 1.0) * on_canvas;

    frag_color = vec4(clamp(col, 0.0, 1.0), 1.0);
}
"""


def _resample_polyline(pts: np.ndarray, n: int) -> np.ndarray:
    """Resample an (M, 2) polyline to n points evenly spaced along its length."""
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0.0:
        return np.repeat(pts[:1], n, axis=0)
    u = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(u, s, pts[:, 0]), np.interp(u, s, pts[:, 1])]).astype(np.float32)


def morph_from_rest(shape: np.ndarray, blend: float, smile_norm: float, skew_norm: float,
                    n: int = 96) -> np.ndarray:
    """Geometric morph between the resting mouth curve and a gesture ``shape``.

    Both are expressed in normalised units (x in half-widths, y in amplitudes).
    The resting curve is the same parabola/tilt the shader draws, so at
    ``blend`` = 0 the polyline coincides with the normal mouth and at 1 it is
    the gesture shape. Intermediate frames bend one into the other.
    """
    xs = np.linspace(-1.0, 1.0, n, dtype=np.float32)
    rest = np.column_stack([xs, -smile_norm * xs * xs + skew_norm * xs]).astype(np.float32)
    target = _resample_polyline(np.asarray(shape, dtype=np.float32).reshape(-1, 2), n)
    b = float(min(max(blend, 0.0), 1.0))
    return (rest * (1.0 - b) + target * b).astype(np.float32)


class GlEyeRenderer:
    """
    Renders animated robot eyes via a GLSL fragment shader.

    All geometry is expressed in pixel coordinates matching the window size,
    with Y increasing downward (top-left origin), matching OpenCV convention.

    Call ``render()`` once per frame from your ROS image callback.
    ``should_close()`` returns True when the user closes the window.
    """

    def __init__(
        self,
        width: int = 1080,
        height: int = 1900,
        title: str = "Robot Disco",
        eye_hw: float = 150.0,          # half-width of each eye (pixels)
        eye_hh: float = 150.0,          # half-height when fully open (pixels)
        eye_side_margin: float = 80.0,  # canvas edge → eye centre (pixels)
        eye_top_margin: float = 80.0,   # canvas top  → eye centre (pixels)
        hand_panel_frac: float = 0.0,   # fraction of screen height for hand panel (0 = off)
        mouth_enabled: bool = True,     # draw the mouth waveform below the eyes
        mouth_gap: float = 150.0,       # eye bottom edge → waveform centre line (pixels)
        mouth_hw: float | None = None,  # waveform half-width (None = span between eye centres)
        mouth_amp: float = 70.0,        # peak displacement at level 1.0 (pixels)
        mouth_samples: int = 512,       # resolution of the sampled-audio texture
        mouth_path_points: int = 128,   # max points in a gesture shape polyline
        fullscreen: bool = False,       # start fullscreen on the primary monitor
        toggle_on_click: bool = True,   # a mouse click (or the F key) toggles fullscreen
        grab_focus_seconds: float = 10.0,  # keep asking the window manager for focus this long after start
        log=None,                       # an rclpy logger (``.info`` / ``.warning``); None prints
    ):
        if not glfw.init():
            raise RuntimeError("glfw.init() failed")
        self._log = log

        # Design canvas: every layout value below is in these pixel units. In
        # fullscreen the canvas is scaled to fit the monitor and letterboxed.
        self._design = (int(width), int(height))
        self._fullscreen = False
        self._windowed_geometry = (100, 100, int(width), int(height))   # x, y, w, h to restore
        self._toggle_on_click = bool(toggle_on_click)

        # GLFW minimises a fullscreen window the moment it loses focus (AUTO_ICONIFY). On the robot
        # that is exactly what happens at start: GNOME's focus-stealing prevention keeps the focus on
        # the desktop when a process it did not launch (supervisor) opens a window, GLFW sees the
        # focus loss and iconifies the face, and it stays hidden until somebody clicks it. The face
        # must never minimise itself; focus is requested separately below (``grab_focus``).
        glfw.window_hint(glfw.AUTO_ICONIFY, False)
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, True)  # required on macOS
        glfw.window_hint(glfw.RESIZABLE, False)

        self._window = glfw.create_window(width, height, title, None, None)
        if not self._window:
            glfw.terminate()
            raise RuntimeError("GLFW window creation failed")

        glfw.make_context_current(self._window)
        glfw.swap_interval(0)  # no vsync — ROS callback rate controls timing

        self._ctx  = moderngl.create_context()
        self._prog = self._ctx.program(vertex_shader=_VERT, fragment_shader=_FRAG)

        # Window events: resize re-letterboxes; click / F toggles fullscreen; Esc leaves it
        glfw.set_framebuffer_size_callback(self._window, lambda win, w, h: self._update_viewport())
        glfw.set_mouse_button_callback(self._window, self._on_mouse_button)
        glfw.set_key_callback(self._window, self._on_key)
        glfw.set_window_focus_callback(self._window, self._on_focus)
        glfw.set_window_iconify_callback(self._window, self._on_iconify)

        # Focus grab (see AUTO_ICONIFY above): until the deadline, every half second the window is
        # restored if minimised and asked to be activated; ``_pursue_focus`` runs from ``render``.
        self._focus_deadline: float | None = None
        self._focus_next_try = 0.0
        self._focus_tries = 0
        self._focus_unsupported = False

        # Fullscreen quad (two triangles as TRIANGLE_STRIP)
        quad     = np.array([-1, -1,  1, -1,  -1, 1,  1, 1], dtype="f4")
        self._vbo = self._ctx.buffer(quad)
        self._vao = self._ctx.simple_vertex_array(self._prog, self._vbo, "in_position")

        # Eye layout
        eye_cy   = eye_top_margin + eye_hh
        left_cx  = eye_side_margin + eye_hw
        right_cx = width - eye_side_margin - eye_hw

        self._prog["u_resolution"].value  = (float(width), float(height))
        self._prog["u_eye_left"].value    = (left_cx, eye_cy)
        self._prog["u_eye_right"].value   = (right_cx, eye_cy)
        self._prog["u_eye_hw"].value      = float(eye_hw)
        self._prog["u_eye_hh_base"].value = float(eye_hh)

        # Hand panel texture (always created; visibility controlled by uniform)
        panel_h = max(1, int(height * hand_panel_frac)) if hand_panel_frac > 0 else 1
        self._hand_tex = self._ctx.texture((width, panel_h), 3)
        self._hand_tex.filter = (moderngl.LINEAR, moderngl.LINEAR)
        self._hand_tex.write(bytes(width * panel_h * 3))  # initialise to black
        self._hand_tex.use(location=0)
        self._prog["u_hand_tex"].value        = 0
        self._prog["u_hand_panel_frac"].value = float(hand_panel_frac)
        self._panel_size = (width, panel_h)

        # Mouth waveform layout + sample texture (texture unit 1)
        self._mouth_enabled = bool(mouth_enabled)
        mouth_cy = eye_cy + eye_hh + mouth_gap
        mouth_cx = width / 2.0
        if mouth_hw is None:
            mouth_hw = (right_cx - left_cx) / 2.0
        self._mouth_n = max(2, int(mouth_samples))
        self._mouth_tex = self._ctx.texture((self._mouth_n, 1), 1, dtype="f4")
        self._mouth_tex.filter   = (moderngl.LINEAR, moderngl.LINEAR)
        self._mouth_tex.repeat_x = False
        self._mouth_tex.repeat_y = False
        self._mouth_tex.write(np.zeros(self._mouth_n, dtype="f4").tobytes())
        self._mouth_tex.use(location=1)
        self._prog["u_mouth_tex"].value    = 1
        self._prog["u_mouth_center"].value = (float(mouth_cx), float(mouth_cy))
        self._prog["u_mouth_hw"].value     = float(max(mouth_hw, 1.0))
        self._prog["u_mouth_amp"].value    = float(mouth_amp)
        self._prog["u_mouth_level"].value  = 0.0
        self._prog["u_mouth_smile"].value  = 0.0
        self._prog["u_mouth_skew"].value   = 0.0
        self._prog["u_mouth_mode"].value   = 1 if self._mouth_enabled else 0
        self._mouth_hw  = float(max(mouth_hw, 1.0))
        self._mouth_amp = float(mouth_amp)

        # Gesture shape polyline texture (texture unit 2)
        self._path_k = max(2, int(mouth_path_points))
        self._path_tex = self._ctx.texture((self._path_k, 1), 2, dtype="f4")
        self._path_tex.filter = (moderngl.NEAREST, moderngl.NEAREST)
        self._path_tex.write(np.zeros(self._path_k * 2, dtype="f4").tobytes())
        self._path_tex.use(location=2)
        self._prog["u_mouth_path"].value   = 2
        self._prog["u_mouth_path_n"].value = 0
        self._prog["u_mouth_shape"].value  = 0.0

        self._t = 0.0

        self._update_viewport()
        if fullscreen:
            self.set_fullscreen(True)
        self.grab_focus(grab_focus_seconds)

    # ------------------------------------------------------------------
    # Focus: a window the user did not open gets no focus from GNOME, so ask for it
    # ------------------------------------------------------------------
    def _say(self, msg: str) -> None:
        if self._log is not None:
            self._log.info(msg)
        else:
            print(msg)

    def _complain(self, msg: str) -> None:
        if self._log is not None:
            self._log.warning(msg)
        else:
            print(msg)

    @property
    def focused(self) -> bool:
        return bool(glfw.get_window_attrib(self._window, glfw.FOCUSED))

    def grab_focus(self, seconds: float = 10.0) -> None:
        """Ask the window manager to activate the window, retrying for ``seconds``
        (requests are sent from ``render``, the GLFW thread)."""
        if seconds <= 0 or self._focus_unsupported:
            return
        self._focus_deadline = time.monotonic() + float(seconds)
        self._focus_next_try = 0.0
        self._focus_tries = 0

    def _pursue_focus(self) -> None:
        if self._focus_deadline is None:
            return
        now = time.monotonic()
        if self.focused:
            if self._focus_tries:
                self._say(f"window: focused after {self._focus_tries} request(s)")
            self._focus_deadline = None
            return
        if now >= self._focus_deadline:
            self._complain(f"window: still not focused after {self._focus_tries} requests; "
                           "the window manager refused it — click the face")
            self._focus_deadline = None
            return
        if now < self._focus_next_try:
            return
        self._focus_next_try = now + 0.5
        self._focus_tries += 1
        try:
            if glfw.get_window_attrib(self._window, glfw.ICONIFIED):
                glfw.restore_window(self._window)
            glfw.focus_window(self._window)
        except Exception as e:  # Wayland: GLFW cannot request focus at all
            self._focus_unsupported = True
            self._focus_deadline = None
            self._complain(f"window: cannot request focus on this platform ({e})")

    def _on_focus(self, win, focused) -> None:
        self._say("window: focus gained" if focused else "window: focus lost")

    def _on_iconify(self, win, iconified) -> None:
        if iconified:
            self._complain("window: minimised by the window manager")
        else:
            self._say("window: restored")

    # ------------------------------------------------------------------
    # Fullscreen / viewport
    # ------------------------------------------------------------------
    @property
    def fullscreen(self) -> bool:
        return self._fullscreen

    def set_fullscreen(self, on: bool) -> None:
        """Switch between a window and fullscreen on the primary monitor. The
        design canvas is scaled to fit and letterboxed, so layout is unchanged."""
        on = bool(on)
        if on == self._fullscreen:
            return
        win = self._window
        if on:
            monitor = glfw.get_primary_monitor()
            mode = glfw.get_video_mode(monitor) if monitor else None
            if monitor is None or mode is None:
                return
            x, y = glfw.get_window_pos(win)
            w, h = glfw.get_window_size(win)
            self._windowed_geometry = (x, y, w, h)
            glfw.set_window_monitor(win, monitor, 0, 0, mode.size.width, mode.size.height, mode.refresh_rate)
            glfw.set_input_mode(win, glfw.CURSOR, glfw.CURSOR_HIDDEN)
        else:
            x, y, w, h = self._windowed_geometry
            glfw.set_window_monitor(win, None, x, y, w, h, 0)
            glfw.set_input_mode(win, glfw.CURSOR, glfw.CURSOR_NORMAL)
        self._fullscreen = on
        self._update_viewport()
        if on:
            # mutter keeps a fullscreen window above the panels only while it is focused
            self.grab_focus()

    def toggle_fullscreen(self) -> None:
        self.set_fullscreen(not self._fullscreen)

    def _update_viewport(self) -> None:
        """Fit the design canvas into the current framebuffer (centred letterbox)."""
        fbw, fbh = glfw.get_framebuffer_size(self._window)
        fbw, fbh = max(int(fbw), 1), max(int(fbh), 1)
        dw, dh = self._design
        scale = min(fbw / dw, fbh / dh)
        off_x = (fbw - dw * scale) / 2.0
        off_y = (fbh - dh * scale) / 2.0
        self._ctx.viewport = (0, 0, fbw, fbh)
        self._prog["u_viewport"].value = (float(fbw), float(fbh))
        self._prog["u_scale"].value    = float(scale)
        self._prog["u_offset"].value   = (float(off_x), float(off_y))

    def _on_mouse_button(self, win, button, action, mods) -> None:
        if self._toggle_on_click and button == glfw.MOUSE_BUTTON_LEFT and action == glfw.PRESS:
            self.toggle_fullscreen()

    def _on_key(self, win, key, scancode, action, mods) -> None:
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_F and self._toggle_on_click:
            self.toggle_fullscreen()
        elif key == glfw.KEY_ESCAPE and self._fullscreen:
            self.set_fullscreen(False)

    # ------------------------------------------------------------------
    def set_mouth_path(self, points: np.ndarray) -> None:
        """Upload a gesture shape as an (N, 2) polyline in normalised units:
        x in mouth half-widths (-1..1), y in waveform amplitudes (down = +)."""
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        n = pts.shape[0]
        if n > self._path_k:                                   # too dense: subsample evenly
            idx = np.linspace(0, n - 1, self._path_k).round().astype(int)
            pts = pts[idx]
            n = pts.shape[0]
        buf = np.zeros((self._path_k, 2), dtype=np.float32)
        buf[:n, 0] = pts[:, 0] * self._mouth_hw
        buf[:n, 1] = pts[:, 1] * self._mouth_amp
        self._path_tex.write(buf.tobytes())
        self._prog["u_mouth_path_n"].value = int(n)

    # ------------------------------------------------------------------
    def update_mouth_samples(self, samples: np.ndarray) -> None:
        """Upload a 1-D array of audio samples in [-1, 1] as the mouth waveform.
        The array is resampled to the texture width if its length differs."""
        data = np.asarray(samples, dtype=np.float32).ravel()
        if data.size == 0:
            data = np.zeros(self._mouth_n, dtype=np.float32)
        elif data.size != self._mouth_n:
            src = np.linspace(0.0, 1.0, data.size, dtype=np.float32)
            dst = np.linspace(0.0, 1.0, self._mouth_n, dtype=np.float32)
            data = np.interp(dst, src, data).astype(np.float32)
        np.clip(data, -1.0, 1.0, out=data)
        self._mouth_tex.write(np.ascontiguousarray(data).tobytes())

    # ------------------------------------------------------------------
    def update_hand_panel(self, img_bgr: np.ndarray | None) -> None:
        """Upload a BGR numpy image as the hand skeleton panel texture.
        Pass None to clear the panel to black."""
        w, h = self._panel_size
        if img_bgr is None:
            self._hand_tex.write(bytes(w * h * 3))
            return
        # Resize to texture dimensions if needed, convert BGR→RGB
        if img_bgr.shape[0] != h or img_bgr.shape[1] != w:
            img_bgr = cv2.resize(img_bgr, (w, h), interpolation=cv2.INTER_LINEAR)
        rgb = np.ascontiguousarray(img_bgr[:, :, ::-1])  # BGR→RGB
        self._hand_tex.write(rgb.tobytes())

    # ------------------------------------------------------------------
    def render(
        self,
        blink: float,
        gaze_offset: tuple[float, float],
        mood: int,
        dt: float = 0.033,
        mouth_level: float = 0.0,
        mouth_samples: np.ndarray | None = None,
        mouth_smile: float = 0.0,
        mouth_skew: float = 0.0,
        mouth_shape: float = 0.0,
        mouth_path: np.ndarray | None = None,
        wink: float = 0.0,
        wink_eye: int = 1,
    ) -> None:
        """Render one frame.  Call from your ROS image callback.

        ``wink`` (0..1) closes one eye on top of ``blink``; ``wink_eye`` picks it
        (0 = screen-left eye, 1 = screen-right eye).

        ``mouth_level`` (0..1) drives the waveform amplitude and glow. When
        ``mouth_samples`` is given (1-D array in [-1, 1]) the waveform traces
        those samples; otherwise a procedural wave is synthesised from the level.
        ``mouth_smile`` curls the ends of the mouth up by that many pixels
        (negative for a frown) and ``mouth_skew`` tilts it for a smirk.
        ``mouth_path`` is an optional (N, 2) normalised polyline (see
        ``set_mouth_path``) and ``mouth_shape`` (0..1) morphs the mouth into it.
        """
        if glfw.window_should_close(self._window):
            return

        glfw.poll_events()
        self._pursue_focus()
        self._t += dt

        self._ctx.clear(0.0, 0.0, 0.0, 1.0)

        if not self._mouth_enabled:
            mouth_mode = 0
        elif mouth_samples is not None:
            self.update_mouth_samples(mouth_samples)
            mouth_mode = 2
        else:
            mouth_mode = 1
        self._prog["u_mouth_mode"].value  = mouth_mode
        self._prog["u_mouth_level"].value = float(min(max(mouth_level, 0.0), 1.0))
        self._prog["u_mouth_smile"].value = float(mouth_smile)
        self._prog["u_mouth_skew"].value  = float(mouth_skew)
        if mouth_path is not None and mouth_shape > 0.0:
            b = float(min(max(mouth_shape, 0.0), 1.0))
            if b >= 0.999:
                pts = mouth_path                                  # exact shape (sharp corners)
            else:
                pts = morph_from_rest(
                    mouth_path, b,
                    smile_norm=float(mouth_smile) / self._mouth_amp,
                    skew_norm=float(mouth_skew) / self._mouth_amp,
                )
            self.set_mouth_path(pts)
            # The polyline already encodes the morph, so draw it fully
            self._prog["u_mouth_shape"].value = 1.0
        else:
            self._prog["u_mouth_shape"].value = 0.0

        self._prog["u_blink"].value = float(blink)
        self._prog["u_wink"].value = float(max(0.0, min(wink, 1.0)))
        self._prog["u_wink_eye"].value = 1 if int(wink_eye) else 0
        # Flip gaze Y: OpenCV Y is down, OpenGL Y is up — we already flipped in shader,
        # so keep Y consistent with the pixel-coord layout (Y down).
        self._prog["u_gaze"].value  = (float(gaze_offset[0]), float(gaze_offset[1]))
        self._prog["u_mood"].value  = int(mood)
        self._prog["u_time"].value  = float(self._t)

        self._vao.render(moderngl.TRIANGLE_STRIP)
        glfw.swap_buffers(self._window)

    # ------------------------------------------------------------------
    def should_close(self) -> bool:
        return bool(glfw.window_should_close(self._window))

    # ------------------------------------------------------------------
    def destroy(self) -> None:
        self._path_tex.release()
        self._mouth_tex.release()
        self._hand_tex.release()
        self._vao.release()
        self._vbo.release()
        self._prog.release()
        self._ctx.release()
        glfw.terminate()


__all__ = ["GlEyeRenderer", "morph_from_rest"]
