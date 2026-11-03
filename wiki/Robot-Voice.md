# robot_voice (the voice assistant)

## robot_voice

**Path**: `src/robot_voice/`
**License**: CC BY-NC-SA

Voice assistant: say **"Robot Disco, …"** and the robot answers out loud through the face (Kokoro voice, mouth in sync). Full details in [src/robot_voice/README.md](https://github.com/r1b4z01d/rd_ws/blob/main/src/robot_voice/README.md).

```
microphone (TONOR G11) ─► ears_worker (venv: energy VAD + faster-whisper) ─► transcript
          ─► wake phrase "robot disco" ─► Brain (rules → Claude with tools, or a local Ollama model) ─► /combined_face_hand/say
```

| Say | Does |
|---|---|
| "…what's the weather tomorrow?" | Open-Meteo forecast for `weather_place` |
| "…any news?" | Headlines from the RSS feeds in `news_feeds` |
| "…turn on the kitchen light" / "is the garage door open?" | Home Assistant REST (`hass_url`, `hass_token`, fuzzy entity names + `hass_aliases`) |
| "…play some music" / "play Daft Punk" / "stop the music" | Local files in `music_dir` through `ffplay`: a shuffled queue that keeps playing (`music_continuous`) |
| "…next song" / "skip" / "previous song" / "what's playing?" | Queue transport. Music ducks by about 13 dB (`music_duck: 0.6`) while the robot listens, thinks and talks, so the wake word works over it |
| "…set your volume to the max" / "volume 40" / "louder" / "lower the volume a bit" / "mute" / "unmute" / "how loud are you?" | Speaker volume on the PulseAudio sink `volume_sink` (the C-Media USB adapter), `volume_step` percent per step, capped at `volume_max` |
| "…move to your charger" / "go to the kitchen" / "go to the right bookshelf" / "stop" | Nav2 `NavigateToPose` to a named pose in `places_file` (`Scripts/waypoints.yaml`: charger, kitchen, dining, couch…) or a landmark from the manager's Landmarks tab (`landmarks_file`, `Scripts/landmarks/landmarks.yaml`) |
| "…pick up tigger" / "stop" | A **landmark task** (`tasks.py`): drive to the landmark the task belongs to (blocking), say "Here goes: pick up tigger", run `<vla_dir>/run.sh scripts/vla_action.py --model … --task … --landmark …` (preflight, serve the model, `eval_hw --auto`) and report how it ended. One job at a time; "stop" cancels the drive or SIGTERMs the run (the eval's EXIT: hand open, arm home). Tasks are defined on the Landmarks tab, whose ▶ Run publishes `{landmark, task, say, from}` on `~/run_task` |
| "…what time is it?" | Clock |
| "Robot Disco?" … "turn off the lamp" | A bare wake word opens a `follow_up_seconds` (6 s) window for the command |
| anything else | Claude (`llm_model`, `claude-opus-5-5`) answers in one or two spoken sentences with the skills as tools; without an API key a local Ollama model (`ollama_model`, `qwen2.5:7b`) does the same, tried at the `ollama_url` list in order: the GPU box (`192.168.11.119:11434`) first, then RD's own Orin (`127.0.0.1:11434`, the `ollama` supervisor unit) |

Simple commands (lights, go to, stop, play, transport, volume, time) are matched by rules instantly; the rest goes to the language model (`llm_backend: auto|claude|ollama|none`), which keeps a short conversation memory and knows who is talking from the face node's `~/person` topic. Without any model the rules still work. The microphone is muted while the robot speaks (`~/speaking` from the face node). Test without a microphone by publishing to `/robot_voice/text_in`.

**Node `robot_voice`** — topics `~/heard`, `~/command`, `~/reply`, `~/state` (idle / listening / thinking / speaking), `~/text_in` (sub), `~/run_task` (sub, from the manager's Landmarks tab). The face topics are hard-wired to the launch-file node name (`/combined_face_hand/say`, `/speaking`, `/person`), so the face must run through its launch file or supervisor unit, not `start_face.sh`. Parameters in `config/voice_assistant.yaml`: ears (`ears_python`, `asr_model` base.en, `asr_device`, `asr_compute_type`, `asr_threads`, `audio_source` "TONOR", `vad_min_rms`, `vad_end_ms`), wake (`wake_phrases`, `wake_threshold`, `follow_up_seconds`, `ack_sound`), brain (`llm_enabled`, `llm_model`, `llm_backend`, `ollama_url`, `ollama_model`, `persona`, `location_name`), skills (weather lat/lon/place/units, `news_feeds`, `hass_*`, `music_dir`, `player_cmd`, `music_continuous`, `music_duck`, `volume_sink`, `volume_step`, `volume_max`, `places_file`, `landmarks_file`, `nav_action`, `nav_frame`), tasks (`vla_dir` `~/rd_ws/VLA`, `task_nav_timeout` 300 s, `task_run_timeout` 600 s).

**Install on RD**: `provision/setup_voice.sh` (venv with faster-whisper and the Jetson onnxruntime pin, anthropic SDK, model download), `provision/provision.sh llm` for the on-robot Ollama fallback (supervisor unit `ollama`, loopback `:11434`; the GPU box's copy is `provision/gpu/provision_gpu.sh llm`), put `ANTHROPIC_API_KEY`, `HASS_URL`, `HASS_TOKEN` in `~/.robot_face/voice.env`, then `provision/deploy_to_rd.sh` builds and starts the unit (`services/robot_voice.conf`).
