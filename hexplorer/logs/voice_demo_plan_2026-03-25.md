# Voice-Controlled YOLO-World Demo for Hexplorer

## Context
The Hexplorer robot dog has working YOLO-World object detection, object following, search, obstacle avoidance, and dance behaviors — but they all require manually typing commands and restarting scripts. This demo adds a voice interface so a person can walk up, say a wake word, and give natural language commands like "follow that person" or "go find the red ball". The robot interprets the command, switches its vision target and behavior, and speaks back to confirm.

## Architecture — ElevenLabs Conversational AI Agent

Uses the ElevenLabs agentic approach (proven in beer-tending `main_agent.py`). The agent handles STT + LLM + TTS in one integrated duplex audio session. Robot actions are exposed as **client tools** that the agent calls.

```
Microphone (Mini PC)
    │
    ▼
Wake Word Detector (local Whisper, energy VAD)
    │  "hey robot" detected
    ▼
ElevenLabs Conversational AI Agent (cloud)
    ├── STT (built-in)
    ├── LLM (built-in, with system prompt + tool definitions)
    └── TTS (built-in) → Speaker
         │
         │ agent calls client tools:
         ▼
Client Tool: execute_robot_action(action, target, detect_mode)
    ├──► SSH: restart Jetson tracker with new YOLO-World target (if changed)
    └──► Subprocess: launch/kill behavior scripts (follower, searcher, dance, etc.)
```

### Why ElevenLabs Agent (vs custom pipeline)
- **Single API**: STT + LLM + TTS handled in one duplex session — lower latency
- **Natural conversation**: Agent maintains context, can ask follow-ups, handle interruptions
- **Tool callbacks**: Agent decides when to trigger robot actions via registered client tools
- **Proven**: Same pattern works in the beer-tending robot (`main_agent.py`)

## New Files

```
~/hexplorer/voice/
    voice_demo.py            # Main orchestrator + agent session (~350 lines)
    .env                     # ELEVENLABS_API_KEY

~/hexplorer/scripts/
    start_voice_demo.sh      # Launch script (~80 lines)
```

Only 2 Python files needed (the agent replaces separate STT/LLM/TTS modules).

## Component Details

### 1. `voice_demo.py` — Main Orchestrator

#### Wake Word Detection (local, before agent session)
- `faster-whisper` small model, int8, CPU — same as beer-tending
- Energy-based VAD: RMS > 0.015, 3-frame start, 1.2s silence end, 500ms pre-roll
- Listens for "hey robot" in transcription
- On wake word: starts ElevenLabs agent session
- On 30s of agent silence / user says "goodbye": ends session, returns to wake word listening

#### ElevenLabs Agent Session
- Uses `elevenlabs.conversational_ai.conversation.Conversation`
- `DefaultAudioInterface()` for mic/speaker
- Agent ID configured on ElevenLabs dashboard (or created via API)
- Callbacks: `callback_user_transcript`, `callback_agent_response`

#### Client Tool: `execute_robot_action`
Registered with the agent session. The agent's system prompt instructs it to call this tool when the user wants the robot to do something.

```python
# Tool receives from agent:
{
    "action": "follow" | "search" | "dance" | "stand" | "sit" | "come_here" | "stop",
    "target": "red ball",           # for follow/search
    "detect_mode": "yolo-world",    # yolo or yolo-world
    "use_smart": false              # obstacle avoidance
}
```

Tool implementation (BehaviorManager):
- `stop_current()`: SIGINT to current behavior subprocess → waits for sit-down
- `update_jetson_tracker(target, mode)`: SSH restart tracker only if target/mode changed (~8s reload)
- `start_follow()`: launches `object_follower.py` or `smart_follower.py` as subprocess
- `start_search()`: launches `object_searcher.py` as subprocess
- `start_dance()`: launches `macarena_full.py` as subprocess
- `do_stand/sit/come_here/stop()`: inline ROS2 publish at 20Hz

Returns status string to agent so it can speak context-aware confirmation (e.g., "Now following the red ball!" or "Hmm, I couldn't switch my camera, try again").

#### ElevenLabs Agent System Prompt
Configure on ElevenLabs dashboard (or via API):
```
You are a friendly robot dog called Hexplorer. You're enthusiastic, helpful, and a little playful.
Keep responses short (1-2 sentences). You're having a real-time voice conversation.

You can perform these actions using the execute_robot_action tool:
- follow: Follow a visible object (requires target). Example: "follow that person", "follow the bottle"
- search: Actively search the area for an object (requires target). Example: "find the red ball", "search for a cup"
- dance: Do the Macarena dance. No target needed.
- stand: Stand up.
- sit: Sit down / rest.
- come_here: Walk toward the speaker briefly.
- stop: Stop whatever you're currently doing.

For target, use the exact object description the user gives.
For detect_mode: use "yolo" for common objects (person, bottle, chair, cup, dog, cat, backpack, etc.)
Use "yolo-world" for descriptive or unusual targets ("red ball", "yellow duck", "fire extinguisher").
Set use_smart=true when navigating complex environments or when the user mentions obstacles.

When you execute an action, tell the user what you're doing. If switching targets takes a moment, let them know.
If you don't understand a command, ask for clarification.
```

### 2. `start_voice_demo.sh` — Launch Script

Follows pattern from `start_object_tracking.sh`, sources `~/hexplorer/scripts/common.sh`:
1. Source ROS2 + robot + Fast-LIO2 workspaces
2. Check Jetson SSH connectivity
3. Start Jetson LiDAR services (persistent, for smart follow/search)
4. Start local LiDAR + IMU TCP receivers
5. Start Fast-LIO2 + odom relay
6. Start detection receiver
7. Start TF publisher
8. Run `voice_demo.py` (manages Jetson tracker lifecycle itself)
9. Cleanup trap on Ctrl+C (kill local processes, kill Jetson camera processes)

Infrastructure is pre-started so behavior switching is fast.

## Existing Files Used (no modification needed)

| File | Launched as | Purpose |
|------|-------------|---------|
| `~/hexplorer/tracking/object_follower.py` | subprocess | Follow detected object |
| `~/hexplorer/tracking/smart_follower.py` | subprocess | Follow with obstacle avoidance |
| `~/hexplorer/navigation/object_searcher.py` | subprocess | Scan-navigate search |
| `~/hexplorer/navigation/macarena_full.py` | subprocess | Dance |
| `~/hexplorer/tracking/detection_receiver.py` | started by launch script | TCP → ROS2 bridge for detections |
| `~/hexplorer/scripts/common.sh` | sourced | Jetson SSH helpers, process management |

## Key Design Decisions

1. **ElevenLabs agent**: Single integrated STT+LLM+TTS with tool callbacks — proven in beer-tending project
2. **Subprocess model**: Existing behavior scripts run as subprocesses — zero modification needed
3. **Jetson tracker managed by tool callback**: SSH-restarts tracker when target changes at runtime
4. **Wake word via local Whisper**: "hey robot" detection before starting agent session (prevents always-on cloud streaming)
5. **Session lifecycle**: Wake word → agent session → 30s idle or "goodbye" → back to wake word

## Dependencies to Install on Mini PC

```bash
pip3 install faster-whisper sounddevice scipy numpy elevenlabs python-dotenv
```

## ElevenLabs Agent Setup

Before running, create an agent on the ElevenLabs dashboard:
1. Go to ElevenLabs → Conversational AI → Create Agent
2. Set the system prompt (see above)
3. Add a client tool `execute_robot_action` with parameters: `action` (string), `target` (string), `detect_mode` (string), `use_smart` (boolean)
4. Choose a voice (or use default)
5. Copy the Agent ID into `.env`

## Verification / Testing

1. **Phase 1 — Wake word only**: Run wake word detection standalone, verify "hey robot" triggers reliably
2. **Phase 2 — Agent conversation only** (no robot): Start agent session, talk to it, verify tool calls are received with correct parameters. Set `DEBUG_MODE=True` to skip robot commands.
3. **Phase 3 — Simple commands**: Enable robot, test "stand up", "sit down", "stop". Verify 20Hz publish, graceful transitions.
4. **Phase 4 — Follow**: Test "follow that person" → verify Jetson tracker starts, `object_follower.py` launches, robot follows.
5. **Phase 5 — Target switching**: "follow person" then "find the red ball" → verify tracker restarts with YOLO-World, behavior switches cleanly.
6. **Phase 6 — Full demo flow**: Wake → "stand up" → "follow that person" → "now find the red ball" → "do a dance" → "sit down" → "goodbye". Verify smooth transitions throughout.
