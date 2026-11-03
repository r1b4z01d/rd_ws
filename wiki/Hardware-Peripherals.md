# WLED panels, audio, T265

---

## WLED LED Matrix Panels

**Location**: Left and right sides of the robot body
**Quantity**: 2 panels (one per side)
**Resolution**: 32 × 48 pixels (W × H) per panel — 1,536 LEDs each

Each panel is an addressable RGB LED matrix running [WLED](https://kno.wled.ge/) firmware over Wi-Fi. WLED exposes an HTTP/JSON API and a WebSocket interface, allowing real-time control of animations, colors, and brightness from any device on the network. The panels can be driven independently or mirrored for symmetric effects.

### Integration Points

- **WLED HTTP API**: `POST /json/state` to set colors, effects, brightness, and segments
- **WLED WebSocket**: `ws://<ip>/ws` for real-time streaming updates
- **E1.31 / Art-Net**: For high-framerate pixel-perfect content streamed from a host PC or Raspberry Pi
- **Foxglove / ROS2**: Can be commanded via a ROS2 node publishing to a WLED HTTP bridge (not yet implemented)

### Example: Set both panels to solid purple

```bash
# Left panel
curl -X POST http://<left-panel-ip>/json/state \
  -H "Content-Type: application/json" \
  -d '{"on":true,"bri":200,"seg":[{"col":[[138,0,96]]}]}'

# Right panel
curl -X POST http://<right-panel-ip>/json/state \
  -H "Content-Type: application/json" \
  -d '{"on":true,"bri":200,"seg":[{"col":[[138,0,96]]}]}'
```

### Panel Layout

| Property | Value |
|---|---|
| Width | 32 pixels |
| Height | 48 pixels |
| Total LEDs per panel | 1,536 |
| Firmware | WLED |
| Interface | Wi-Fi (HTTP + WebSocket + E1.31) |
| Mounting | Left side and right side of robot body |

## Audio: USB speaker and microphone

**Speaker**: the dock's C-Media USB audio adapter (PulseAudio sink `C-Media`). **Microphone**: a TONOR G11 USB microphone (source `TONOR`). There is no Bluetooth audio.

`provision/pulse/default.pa` (installed to `~/.config/pulse/default.pa` by `provision.sh audio`) sets the adapter as the default sink and unloads `module-switch-on-connect` (a freshly plugged USB mic once stole the output) and `module-suspend-on-idle` (clipped first syllables); the phase also unmutes the sink and runs `alsactl store`, because the adapter came back muted at boot on 2026-10-31. At runtime the face node's `SinkWatchdog` (`audio_sink` parameter) keeps the adapter the default sink whenever it is present and pins `aplay` to it, and the voice assistant's volume skill drives the same sink (`volume_sink`). Anything that should come out of the speaker (`ffplay`, `aplay`, `paplay`) just uses the default sink. "No audio" usually means the sink is muted: `pactl set-sink-mute @DEFAULT_SINK@ 0`, then `alsactl store`.

## RealSense T265 Tracking Camera (retired)

A T265 is still mounted on the left side but is not used: the librealsense that ships for the Jetson (≥ 2.54) dropped T265 support, no launch file starts it and nothing fuses its odometry. Localization is AMCL on the lidars with the AprilTag relocalizer as the corrector. Keep it in mind only if the camera is ever swapped for a supported tracking device.
