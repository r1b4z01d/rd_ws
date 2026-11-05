# robot_voice

Voice assistant for Robot Disco. Say **"Robot Disco, …"** and the robot answers out loud through the
face (Kokoro voice, mouth in sync):

```
microphone ─► ears_worker (venv: energy VAD + faster-whisper) ─► transcript
          ─► wake phrase "robot disco" ─► Brain ─► reply ─► /combined_face_hand/say
```

| What you say | What happens |
|---|---|
| "Robot Disco, what's the weather tomorrow?" | Open-Meteo forecast for the configured place |
| "Robot Disco, any news?" | Top headlines from the configured RSS feeds (NPR, BBC by default) |
| "Robot Disco, turn on the kitchen light." | Home Assistant `light.turn_on` on the best-matching entity |
| "Robot Disco, is the garage door open?" | Home Assistant state query |
| "Robot Disco, play some music." / "play Daft Punk" / "stop the music" | Local files from `music_dir` through `ffplay`: a shuffled queue (library, an artist/album folder, or the requested song then the rest) that keeps playing when a song ends |
| "Robot Disco, next song." / "skip" / "previous song" / "what's playing?" | Queue transport; music ducks by about 13 dB (`music_duck: 0.6`, live-tunable) while the robot listens, thinks and talks, so the wake word works over it |
| "Robot Disco, set your volume to the max." / "full volume" / "volume 40" / "louder" / "lower the volume a bit" / "mute" / "unmute" / "how loud are you?" | Speaker volume on the PulseAudio default sink (the dock's USB audio adapter); "Muting" is spoken before the speaker goes quiet |
| "Robot Disco, move to your charger." / "go to the kitchen" / "go to the right bookshelf" / "stop" | Nav2 `NavigateToPose` to a named pose from `waypoints.yaml` or a landmark from the manager's Landmarks tab (`Scripts/landmarks/landmarks.yaml`) |
| "Robot Disco, pick up tigger." / "stop" | A **landmark task** (`tasks.py`): the landmark the task belongs to says where to drive; the robot goes there (Nav2, blocking), says "Here goes: pick up tigger", runs `~/rd_ws/VLA/run.sh scripts/vla_action.py --model … --task … --landmark …` (preflight, serve the model, `eval_hw --auto`) and reports how it ended. One job at a time; "stop" cancels the drive or SIGTERMs the run (= the eval's EXIT: hand open, arm home). Tasks are added on the Landmarks tab, whose ▶ Run publishes `{landmark, task}` on `~/run_task` |
| "Robot Disco, what time is it?" | Clock |
| "Robot Disco, how far away is the moon?" | Claude answers (one or two spoken sentences) |
| "Robot Disco?" … "turn off the lamp" | Bare wake word opens a 6 s follow-up window |

**Brain.** Three tiers, chosen by `llm_backend` (`auto` by default): Claude (`claude-opus-5-5`) when an
`ANTHROPIC_API_KEY` is present, otherwise a **local model through Ollama** (`qwen2.5:7b`, tool calling included, no internet or key needed) — `ollama_url`
is a comma-separated list tried in order, by default the GPU box (`192.168.11.119`, fast, bigger models fit) and then RD's own
Orin GPU as the fallback when the box is off — otherwise rules only. A handful of unambiguous commands (lights, go to, stop, play, volume, time) are matched by rules with no
network round trip. Everything else goes to Claude (`claude-opus-5-5`) with the skills exposed as tools, so
loose phrasing and follow-up questions work and the robot can answer general questions. Without an API key
the rules still run and open questions get a short "my language model isn't connected" reply.

The brain knows who is talking: the face node's `~/person` topic (face recognition) supplies the name.

## Install on the robot

```bash
# local LLM fallback (no key, no internet): a supervisor unit `ollama` on each box, qwen2.5:7b on :11434
#   GPU box:  rd_ws/provision/gpu/provision_gpu.sh llm      (serves on every NIC; the robot asks it first)
#   RD:       ~/rd_ws/provision/provision.sh llm            (the Orin's own copy, loopback; the fallback when the box is off)
provision/setup_voice.sh            # venv with faster-whisper (+ Jetson onnxruntime pin), anthropic SDK, model download
nano ~/.robot_face/voice.env        # ANTHROPIC_API_KEY=…  HASS_URL=…  HASS_TOKEN=…
provision/deploy_to_rd.sh           # from the workstation: push, build (--symlink-install), (re)start robot_voice, tail the log
# or by hand on RD: colcon build --packages-select robot_voice --symlink-install && sudo supervisorctl start robot_voice
```

Configuration lives in `config/voice_assistant.yaml`, loaded by the launch file and the supervisor unit:

| Group | Parameters |
|---|---|
| ears | `ears_enabled`, `ears_python` (`~/.robot_face/voice-venv/bin/python`), `asr_model` (`base.en`; `small.en` is more accurate once its model is on the robot), `asr_device` (`cpu`), `asr_compute_type` (`int8`), `asr_threads` (4), `audio_source` (`TONOR`, a PulseAudio source substring; the node's own default `""` = the first USB mic), `vad_min_rms` (0.012), `vad_end_ms` (900) |
| wake | `wake_phrases` (`["robot disco"]`), `wake_threshold` (0.74, fuzzy score), `follow_up_seconds` (6.0), `ack_sound` |
| face | `say_topic`, `speaking_topic`, `person_topic` (all under `/combined_face_hand`, the launch-file node name: a face started with `ros2 run` is not reachable) |
| brain | `llm_enabled`, `llm_model` (`claude-opus-5-5`), `llm_backend` (`auto` \| `claude` \| `ollama` \| `none`), `ollama_url` (comma-separated, tried in order: `http://192.168.11.119:11434,http://127.0.0.1:11434`), `ollama_model` (`qwen2.5:7b`, must support tool calling), `persona`, `location_name` |
| skills | `weather_latitude` / `weather_longitude` / `weather_place` / `units`, `news_feeds` (`Name=url`), `hass_url` / `hass_token` / `hass_aliases` (`"kitchen light=light.kitchen_main"`), `music_dir`, `player_cmd`, `music_continuous`, `music_duck` (0.6; live: `ros2 param set /robot_voice music_duck 0.7`), `volume_sink` (`C-Media`), `volume_step` (10), `volume_max` (100), `places_file` (`~/rd_ws/Scripts/waypoints.yaml`), `landmarks_file` (`~/rd_ws/Scripts/landmarks/landmarks.yaml`), `nav_action`, `nav_frame` |
| tasks | `vla_dir` (`~/rd_ws/VLA`: tasks run `<vla_dir>/run.sh scripts/vla_action.py`), `task_nav_timeout` (300 s to reach the landmark), `task_run_timeout` (600 s on top of the task's own seconds before the run is terminated) |

## Try it without a microphone

```bash
ros2 topic pub --once /robot_voice/text_in std_msgs/msg/String "{data: 'Robot Disco, what is the weather?'}"
ros2 topic echo /robot_voice/reply
```

`~/heard` shows every transcript, `~/command` the text after the wake phrase, `~/state` idle / listening /
thinking / speaking. While the robot speaks the microphone is muted (face node `~/speaking`). `~/run_task`
(`std_msgs/String`, JSON `{"landmark": "<name or id>", "task": "<task name>", "say": true, "from": "manager"}`) is what the
manager's Landmarks tab publishes with ▶ Run; the node drives there and runs the task exactly as if it had been asked aloud.

## Audio routing on RD

The speaker is the dock's USB audio adapter (C-Media / Unitek), not Bluetooth. It feeds a 2.1 amplifier driving a pair of Cerwin-Vega 6x9 speakers and a 3" HexiBase subwoofer (parts and links: the workspace wiki, Hardware-Peripherals). `~/.config/pulse/default.pa` on RD
sets it as the default sink and unloads `module-switch-on-connect` (a freshly plugged USB mic once stole the output and
silenced the robot) and `module-suspend-on-idle` (clipped first syllables). A new microphone shows up as a PulseAudio
source; set `audio_source` to a substring of its name (`ears_worker.py --list-sources`) and check its level with
`parecord --device=<source> --raw … | python3 -c "…rms…"`: room noise around 0.003 RMS and speech above 0.02 is a usable mic.
`vad_min_rms` is an absolute floor; the VAD also needs three times the measured room noise.

## Test the ears alone

```bash
~/.robot_face/voice-venv/bin/python src/robot_voice/robot_voice/ears_worker.py --list-sources
~/.robot_face/voice-venv/bin/python src/robot_voice/robot_voice/ears_worker.py --print        # live transcripts
~/.robot_face/voice-venv/bin/python src/robot_voice/robot_voice/ears_worker.py --file x.wav   # one file
```

## Files

| File | Purpose |
|---|---|
| `robot_voice/ears_worker.py` | Capture (`parecord`), streaming energy VAD, faster-whisper; JSON events on stdout |
| `robot_voice/wake.py` | Fuzzy "robot disco" matcher (tolerates "robo disco", "robodisco", "robot discos") |
| `robot_voice/skills.py` | Weather (Open-Meteo), news (RSS), Home Assistant (REST), music (ffplay), speaker volume (pactl), clock |
| `robot_voice/brain.py` | Rules + Claude tool-use brain with short conversation memory |
| `robot_voice/voice_assistant_node.py` | ROS node: ears process, wake logic, Nav2 navigator (waypoints + landmarks), face integration, `~/run_task` |
| `robot_voice/tasks.py` | Landmark tasks: the landmarks file, the phrase matcher ("could you pick up tigger please" → the task), the drive-then-run worker |
| `../../services/robot_voice.conf` | Supervisor unit (sources `~/.robot_face/voice.env` for secrets) |
| `../../provision/setup_voice.sh` | Installs the speech venv and the anthropic SDK |
