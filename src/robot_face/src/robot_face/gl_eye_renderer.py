"""Hardware-accelerated eye renderer — moderngl + GLFW + GLSL fragment shader."""

from __future__ import annotations

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

uniform vec2  u_resolution;
uniform vec2  u_eye_left;       // eye center in pixel coords (Y=down)
uniform vec2  u_eye_right;
uniform float u_eye_hw;         // half-width  (pixels)
uniform float u_eye_hh_base;    // half-height when fully open (pixels)
uniform float u_blink;          // 0 = open, 1 = closed
uniform vec2  u_gaze;           // pupil offset (pixels, Y=down)
uniform int   u_mood;           // 0=neutral 1=tired 2=angry 3=happy 4=suspicious
uniform float u_time;
uniform sampler2D u_hand_tex;       // hand skeleton panel (RGB, black background)
uniform float u_hand_panel_frac;    // fraction of screen height used for panel (0 = hidden)

out vec4 frag_color;

// --- palette (linear RGB) ---
const vec3 C_BASE = vec3(0.15, 0.03, 0.30);   // dark purple fill
const vec3 C_LINE = vec3(0.80, 0.20, 1.00);   // bright electric-purple scanlines
const vec3 C_GLOW = vec3(0.85, 0.25, 1.00);   // bloom / outer glow
const vec3 C_RIM  = vec3(0.90, 0.65, 1.00);   // bright lavender rim highlight

// Rounded-rectangle signed distance field (negative = inside)
float sdRoundRect(vec2 p, vec2 half_size, float r) {
    vec2 q = abs(p) - half_size + r;
    return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - r;
}

// Right-angle triangle corner cuts.
// Each returns 1.0 if fragment p (in local eye coords) falls inside the cut.
// hs  = eye half-size (hw, hh)
// cs  = cut size (width, height) measured inward from that corner
float cutTL(vec2 p, vec2 hs, vec2 cs) {          // top-left
    vec2 d = p - vec2(-hs.x, -hs.y);
    if (d.x < 0.0 || d.y < 0.0) return 0.0;
    return step(d.x / cs.x + d.y / cs.y, 1.0);
}
float cutTR(vec2 p, vec2 hs, vec2 cs) {          // top-right
    vec2 d = p - vec2( hs.x, -hs.y);
    if (d.x > 0.0 || d.y < 0.0) return 0.0;
    return step(-d.x / cs.x + d.y / cs.y, 1.0);
}
float cutBL(vec2 p, vec2 hs, vec2 cs) {          // bottom-left
    vec2 d = p - vec2(-hs.x,  hs.y);
    if (d.x < 0.0 || d.y > 0.0) return 0.0;
    return step(d.x / cs.x - d.y / cs.y, 1.0);
}
float cutBR(vec2 p, vec2 hs, vec2 cs) {          // bottom-right
    vec2 d = p - vec2( hs.x,  hs.y);
    if (d.x > 0.0 || d.y > 0.0) return 0.0;
    return step(-d.x / cs.x - d.y / cs.y, 1.0);
}

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
    return 0.0;
}

vec3 renderEye(vec2 frag, vec2 center) {
    float hh = u_eye_hh_base * max(0.0, 1.0 - u_blink);
    if (hh < 1.0) return vec3(0.0);

    vec2 hs       = vec2(u_eye_hw, hh);
    float corner_r = min(hs.x, hs.y) * 0.18;

    // Local coords: center of eye = origin, gaze shifts the pattern
    vec2 p = frag - center - u_gaze;

    float sdf    = sdRoundRect(p, hs, corner_r);
    float inside = smoothstep(1.5, -1.5, sdf);           // 1 inside, 0 outside

    // Mood: zero out inside mask where corners are cut
    float cut    = moodMask(p, hs, u_mood);
    inside      *= (1.0 - cut);

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

void main() {
    // OpenGL origin is bottom-left; flip Y to match our top-left pixel layout
    vec2 frag = vec2(gl_FragCoord.x, u_resolution.y - gl_FragCoord.y);

    vec3 col = renderEye(frag, u_eye_left) + renderEye(frag, u_eye_right);

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
    col *= clamp(vig, 0.0, 1.0);

    frag_color = vec4(clamp(col, 0.0, 1.0), 1.0);
}
"""


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
    ):
        if not glfw.init():
            raise RuntimeError("glfw.init() failed")

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

        self._t = 0.0

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
    ) -> None:
        """Render one frame.  Call from your ROS image callback."""
        if glfw.window_should_close(self._window):
            return

        glfw.poll_events()
        self._t += dt

        self._ctx.clear(0.0, 0.0, 0.0, 1.0)

        self._prog["u_blink"].value = float(blink)
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
        self._vao.release()
        self._vbo.release()
        self._prog.release()
        self._ctx.release()
        glfw.terminate()


__all__ = ["GlEyeRenderer"]
