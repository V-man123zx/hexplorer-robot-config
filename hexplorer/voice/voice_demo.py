#!/usr/bin/env python3
"""
Voice-Controlled YOLO-World Demo for Hexplorer Robot Dog

Wake word ("hey robot") activates an ElevenLabs Conversational AI Agent session.
The agent interprets natural language commands and calls client tools to control
the robot: follow objects, search, dance, stand, sit, etc.

Usage:
    source /opt/ros/humble/setup.bash
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    source ~/fastlio_ws/install/setup.bash
    python3 voice_demo.py [--debug]

Requires:
    - ELEVENLABS_API_KEY in .env or environment
    - AGENT_ID in .env or environment (from ElevenLabs dashboard)
    - ROS2 workspace sourced (for robot control)
    - Jetson services running (managed by start_voice_demo.sh)
"""

import os
import sys
import time
import signal
import json
import queue
import threading
import subprocess
import numpy as np
import argparse

# ─── Configuration ───────────────────────────────────────────────────────────

HEXPLORER_DIR = os.path.expanduser("~/hexplorer")
SCRIPT_DIR = os.path.join(HEXPLORER_DIR, "scripts")

# Jetson SSH
JETSON_IP = "192.168.1.20"
JETSON_USER = "robot"
JETSON_PASS = "123"

# Wake word detection
WAKE_PHRASE = "hey robot"
SAMPLE_RATE = 16000
FRAME_MS = 50
FRAME_SAMPLES = int(SAMPLE_RATE * FRAME_MS / 1000)
ENERGY_THRESHOLD = 0.002
START_FRAMES_REQUIRED = 3
END_SILENCE_MS = 1200
PRE_ROLL_MS = 500
MAX_UTTERANCE_SEC = 10

# Robot state constants
STATE_PASSIVE = 0
STATE_STANDDOWN = 1
STATE_STANDUP = 2
STATE_BALANCESTAND = 3
STATE_WALK = 4

# ─── Wake Word Listener ─────────────────────────────────────────────────────

class WakeWordListener:
    """Listens for wake word using energy-based VAD + local Whisper."""

    def __init__(self):
        import sounddevice as sd
        from faster_whisper import WhisperModel

        self.sd = sd
        self.model = WhisperModel("small", compute_type="int8", device="cpu")
        self.paused = False
        self._lock = threading.Lock()

    def pause(self):
        with self._lock:
            self.paused = True

    def resume(self):
        with self._lock:
            self.paused = False

    def _is_paused(self):
        with self._lock:
            return self.paused

    def record_utterance(self):
        """Record a single utterance using energy-based VAD. Returns int16 numpy array or None."""
        frames = []
        pre_roll_frames = int(PRE_ROLL_MS / FRAME_MS)
        pre_roll_buffer = []
        end_silence_frames = int(END_SILENCE_MS / FRAME_MS)
        max_frames = int(MAX_UTTERANCE_SEC * 1000 / FRAME_MS)

        recording = False
        silent_count = 0
        start_count = 0

        stream = self.sd.InputStream(
            samplerate=SAMPLE_RATE, channels=1, dtype='int16',
            blocksize=FRAME_SAMPLES
        )
        stream.start()

        try:
            while True:
                if self._is_paused():
                    time.sleep(0.05)
                    continue

                data, overflowed = stream.read(FRAME_SAMPLES)
                audio_float = data.flatten().astype(np.float32) / 32768.0
                rms = np.sqrt(np.mean(audio_float ** 2))

                if not recording:
                    pre_roll_buffer.append(data.copy())
                    if len(pre_roll_buffer) > pre_roll_frames:
                        pre_roll_buffer.pop(0)

                    if rms > ENERGY_THRESHOLD:
                        start_count += 1
                        if start_count >= START_FRAMES_REQUIRED:
                            recording = True
                            frames = list(pre_roll_buffer)
                            frames.append(data.copy())
                    else:
                        start_count = 0
                else:
                    frames.append(data.copy())

                    if rms < ENERGY_THRESHOLD:
                        silent_count += 1
                        if silent_count >= end_silence_frames:
                            break
                    else:
                        silent_count = 0

                    if len(frames) >= max_frames:
                        break
        finally:
            stream.stop()
            stream.close()

        if not frames:
            return None

        return np.concatenate(frames).flatten()

    def transcribe(self, audio_int16):
        """Transcribe int16 audio with Whisper."""
        audio_float = audio_int16.astype(np.float32) / 32768.0
        segments, info = self.model.transcribe(audio_float, beam_size=3, language="en")
        text = " ".join(seg.text for seg in segments).strip().lower()
        return text

    def wait_for_wake_word(self):
        """Block until wake word is detected. Returns True on detection."""
        print("Listening for wake word: 'hey robot'...")
        while True:
            audio = self.record_utterance()
            if audio is None:
                continue
            text = self.transcribe(audio)
            if text:
                print(f"  Heard: '{text}'")
            if WAKE_PHRASE in text.replace(",", "").replace(".", ""):
                print("  Wake word detected!")
                return True


# ─── Behavior Manager ────────────────────────────────────────────────────────

class BehaviorManager:
    """Manages robot behavior subprocesses and inline ROS2 commands."""

    def __init__(self, debug=False):
        self.debug = debug
        self.current_process = None
        self.current_action = None
        self.jetson_target = None
        self.jetson_detect_mode = None
        self.ros_env = self._build_ros_env()

        if not debug:
            self._init_ros()

    def _build_ros_env(self):
        """Build environment dict with ROS2 sourced."""
        env = os.environ.copy()
        return env

    def _init_ros(self):
        """Initialize ROS2 node for inline commands."""
        import rclpy
        from custom_msg.msg import RobotCommand
        from geometry_msgs.msg import Twist

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('voice_controller')
        self.cmd_pub = self.node.create_publisher(RobotCommand, '/robot_cmd', 10)
        self.vel_pub = self.node.create_publisher(Twist, '/vel_cmd', 10)
        self.RobotCommand = RobotCommand
        self.Twist = Twist
        time.sleep(0.3)

    def _publish_cmd(self, state, duration, vx=0.0, vz=0.0):
        """Publish robot command at 20Hz for given duration."""
        if self.debug:
            print(f"  [DEBUG] Would publish state={state} vx={vx} vz={vz} for {duration}s")
            return
        cmd = self.RobotCommand()
        cmd.target_state = state
        vel = self.Twist()
        vel.linear.x = vx
        vel.angular.z = vz
        ticks = int(duration / 0.05)
        for _ in range(ticks):
            self.cmd_pub.publish(cmd)
            self.vel_pub.publish(vel)
            time.sleep(0.05)

    def _ssh_cmd(self, cmd):
        """Run command on Jetson via SSH."""
        full_cmd = (
            f"sshpass -p '{JETSON_PASS}' ssh -o StrictHostKeyChecking=no "
            f"-o ConnectTimeout=5 {JETSON_USER}@{JETSON_IP} \"{cmd}\""
        )
        return subprocess.run(full_cmd, shell=True, capture_output=True, text=True, timeout=15)

    def stop_current(self):
        """Stop current behavior subprocess, let it sit down gracefully."""
        if self.current_process and self.current_process.poll() is None:
            print(f"  Stopping current behavior ({self.current_action})...")
            self.current_process.send_signal(signal.SIGINT)
            try:
                self.current_process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                self.current_process.kill()
                self.current_process.wait()
        self.current_process = None
        self.current_action = None

    def update_jetson_tracker(self, target, detect_mode):
        """Restart Jetson tracker if target/mode changed. Returns True on success."""
        if target == self.jetson_target and detect_mode == self.jetson_detect_mode:
            print(f"  Tracker already set to {detect_mode}:{target}")
            return True

        if self.debug:
            print(f"  [DEBUG] Would restart Jetson tracker: {detect_mode}:{target}")
            self.jetson_target = target
            self.jetson_detect_mode = detect_mode
            return True

        print(f"  Restarting Jetson tracker: {detect_mode}:{target}")

        # Kill existing tracker
        self._ssh_cmd("pkill -9 -f jetson_object_tracker.py 2>/dev/null; true")
        time.sleep(1)

        # Start new tracker
        self._ssh_cmd(
            f"nohup /home/robot/start_tracker.sh {detect_mode} '{target}' '--stream-images' "
            f"> /tmp/tracker.log 2>&1 < /dev/null &"
        )

        # Wait for model load
        if detect_mode in ("yolo", "yolo-world"):
            print("  Waiting for YOLO model to load (~8s)...")
            time.sleep(8)
        else:
            time.sleep(3)

        self.jetson_target = target
        self.jetson_detect_mode = detect_mode
        return True

    def start_follow(self, target, detect_mode, use_smart=False):
        """Start following an object."""
        self.stop_current()
        if not self.update_jetson_tracker(target, detect_mode):
            return "Failed to switch vision target"

        if use_smart:
            script = os.path.join(HEXPLORER_DIR, "tracking", "smart_follower.py")
            cmd = ["python3", script,
                   "--target-distance", "800", "--max-speed", "0.3",
                   "--turn-speed", "0.8"]
        else:
            script = os.path.join(HEXPLORER_DIR, "tracking", "object_follower.py")
            cmd = ["python3", script,
                   "--target-distance", "800", "--max-speed", "0.3",
                   "--turn-speed", "0.15"]

        if self.debug:
            print(f"  [DEBUG] Would run: {' '.join(cmd)}")
        else:
            self.current_process = subprocess.Popen(cmd, env=self.ros_env)

        self.current_action = "follow"
        mode_label = "smart follow" if use_smart else "follow"
        return f"Now {mode_label}ing {target}"

    def start_search(self, target, detect_mode):
        """Start searching for an object."""
        self.stop_current()
        if not self.update_jetson_tracker(target, detect_mode):
            return "Failed to switch vision target"

        script = os.path.join(HEXPLORER_DIR, "navigation", "object_searcher.py")
        cmd = ["python3", script,
               "--search-speed", "0.15", "--scan-speed", "0.15",
               "--navigate-distance", "2.0", "--stop-distance", "0.8"]

        if self.debug:
            print(f"  [DEBUG] Would run: {' '.join(cmd)}")
        else:
            self.current_process = subprocess.Popen(cmd, env=self.ros_env)

        self.current_action = "search"
        return f"Searching for {target}"

    def start_dance(self):
        """Start the Macarena dance."""
        self.stop_current()

        script = os.path.join(HEXPLORER_DIR, "navigation", "macarena_full.py")
        cmd = ["python3", script]

        if self.debug:
            print(f"  [DEBUG] Would run: {' '.join(cmd)}")
        else:
            self.current_process = subprocess.Popen(cmd, env=self.ros_env)

        self.current_action = "dance"
        return "Dancing the Macarena!"

    def do_stand(self):
        """Stand up sequence."""
        self.stop_current()
        for state in [STATE_STANDDOWN, STATE_STANDUP, STATE_BALANCESTAND]:
            self._publish_cmd(state, 2.0)
        self.current_action = "standing"
        return "Standing up"

    def do_sit(self):
        """Sit down sequence."""
        self.stop_current()
        for state in [STATE_BALANCESTAND, STATE_STANDDOWN, STATE_PASSIVE]:
            self._publish_cmd(state, 2.0)
        self.current_action = None
        return "Sitting down"

    def do_come_here(self):
        """Walk forward briefly."""
        self.stop_current()
        # Stand up first if needed
        for state in [STATE_STANDDOWN, STATE_STANDUP, STATE_BALANCESTAND]:
            self._publish_cmd(state, 2.0)
        # Walk forward for 3 seconds
        self._publish_cmd(STATE_WALK, 3.0, vx=0.2)
        # Back to balance stand
        self._publish_cmd(STATE_BALANCESTAND, 1.0)
        self.current_action = "standing"
        return "Coming toward you"

    def do_stop(self):
        """Stop current behavior."""
        self.stop_current()
        self._publish_cmd(STATE_BALANCESTAND, 1.0)
        self.current_action = "standing"
        return "Stopped"

    def shutdown(self):
        """Full shutdown: stop behavior, sit down."""
        self.stop_current()
        if not self.debug:
            self.do_sit()
            self.node.destroy_node()

    def execute_action(self, action, target=None, detect_mode="yolo-world", use_smart=False):
        """Execute a robot action. Returns status string."""
        print(f"  Executing: action={action} target={target} mode={detect_mode} smart={use_smart}")

        if action == "follow":
            if not target:
                return "I need to know what to follow. What should I look for?"
            return self.start_follow(target, detect_mode, use_smart)
        elif action == "search":
            if not target:
                return "I need to know what to search for. What should I find?"
            return self.start_search(target, detect_mode)
        elif action == "dance":
            return self.start_dance()
        elif action == "stand":
            return self.do_stand()
        elif action == "sit":
            return self.do_sit()
        elif action == "come_here":
            return self.do_come_here()
        elif action == "stop":
            return self.do_stop()
        else:
            return f"Unknown action: {action}"


# ─── ElevenLabs Agent Session ────────────────────────────────────────────────

class AgentSession:
    """Manages an ElevenLabs Conversational AI Agent session."""

    def __init__(self, behavior_manager, agent_id, api_key, log_dir=None):
        self.behavior = behavior_manager
        self.agent_id = agent_id
        self.api_key = api_key
        self.conversation = None
        self.last_activity = time.time()
        self.session_active = False
        self.log_file = None

        # Open conversation log file
        if log_dir is None:
            log_dir = os.path.join(HEXPLORER_DIR, "logs", "voice_sessions")
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(log_dir, f"session_{ts}.log")
        self.log_file = open(log_path, "w")
        self._log("SYS  ", f"Session log: {log_path}")

    def _log(self, tag, msg):
        """Print timestamped log line and write to log file."""
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] [{tag}] {msg}"
        print(line, flush=True)
        if self.log_file:
            self.log_file.write(line + "\n")
            self.log_file.flush()

    def _handle_agent_response(self, response):
        """Callback when agent speaks."""
        self._log("AGENT", response)
        self.last_activity = time.time()

    def _handle_user_transcript(self, transcript):
        """Callback when user speech is transcribed."""
        self._log("USER ", transcript)
        self.last_activity = time.time()

    def _handle_tool_call(self, parameters):
        """Handle client tool call from agent. Receives parameters dict, returns result string.

        Runs the action in a background thread so the tool returns immediately
        (robot actions like stand-up take 6+ seconds, which would timeout the agent).
        """
        self._log("TOOL ", f"execute_robot_action({parameters})")

        action = parameters.get("action", "unknown")
        target = parameters.get("target", None)
        detect_mode = parameters.get("detect_mode", "yolo-world")
        use_smart = parameters.get("use_smart", False)

        # Run action in background thread so we return fast
        def _run():
            result = self.behavior.execute_action(action, target, detect_mode, use_smart)
            self._log("RESULT", result)

        threading.Thread(target=_run, daemon=True).start()

        # Return immediately with confirmation
        confirmations = {
            "follow": f"Following {target or 'object'}",
            "search": f"Searching for {target or 'object'}",
            "dance": "Starting the Macarena",
            "stand": "Standing up now",
            "sit": "Sitting down now",
            "come_here": "Coming toward you",
            "stop": "Stopping",
        }
        return confirmations.get(action, f"Executing {action}")

    def start(self):
        """Start the agent conversation session."""
        from elevenlabs import ElevenLabs
        from elevenlabs.conversational_ai.conversation import Conversation, ClientTools
        from elevenlabs.conversational_ai.default_audio_interface import DefaultAudioInterface

        self.session_active = True
        self.last_activity = time.time()

        # Create client and client tools
        client = ElevenLabs(api_key=self.api_key)

        client_tools = ClientTools()
        client_tools.register(
            "execute_robot_action",
            self._handle_tool_call
        )

        self.conversation = Conversation(
            client=client,
            agent_id=self.agent_id,
            requires_auth=False,
            audio_interface=DefaultAudioInterface(),
            client_tools=client_tools,
            callback_agent_response=lambda response: self._handle_agent_response(response),
            callback_user_transcript=lambda transcript: self._handle_user_transcript(transcript),
        )

        self._log("SYS  ", "Starting agent session...")
        self.conversation.start_session()
        self._log("SYS  ", "Agent session started — listening for commands")

    def wait_for_end(self, idle_timeout=60):
        """Wait for session to end (idle timeout or explicit end)."""
        try:
            while self.session_active:
                time.sleep(1.0)
                idle = time.time() - self.last_activity
                if idle > idle_timeout:
                    print(f"  Session idle for {idle_timeout}s, ending...")
                    break
        except KeyboardInterrupt:
            pass

    def end(self):
        """End the agent session. Ensures cloud WebSocket is closed."""
        self.session_active = False
        if self.conversation:
            try:
                self._log("SYS  ", "Ending agent session...")
                self.conversation.end_session()
                # wait_for_session_end joins the thread — use a timeout
                # to avoid blocking forever
                if self.conversation._thread and self.conversation._thread.is_alive():
                    self.conversation._thread.join(timeout=5)
                    if self.conversation._thread.is_alive():
                        self._log("SYS  ", "Session thread didn't stop in 5s, forcing...")
            except Exception as e:
                print(f"  Session end error: {e}")
            self.conversation = None
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        print("Agent session ended.", flush=True)


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Voice-controlled Hexplorer demo")
    parser.add_argument("--debug", action="store_true",
                        help="Debug mode: no robot commands, print actions only")
    args = parser.parse_args()

    # Load environment
    try:
        from dotenv import load_dotenv
        env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
        load_dotenv(env_path)
    except ImportError:
        pass

    api_key = os.environ.get("ELEVENLABS_API_KEY")
    agent_id = os.environ.get("AGENT_ID")

    if not api_key:
        print("ERROR: ELEVENLABS_API_KEY not set. Add it to ~/hexplorer/voice/.env")
        sys.exit(1)
    if not agent_id:
        print("ERROR: AGENT_ID not set. Create an agent at elevenlabs.io and add ID to .env")
        sys.exit(1)

    print("=========================================")
    print("  Hexplorer Voice-Controlled Demo")
    print("=========================================")
    print(f"  Debug mode: {args.debug}")
    print(f"  Wake word: '{WAKE_PHRASE}'")
    print("")

    # Initialize components
    behavior = BehaviorManager(debug=args.debug)
    listener = WakeWordListener()

    # Track active session for cleanup
    active_session = None
    running = True

    def cleanup_and_exit():
        """Ensure cloud session and robot are shut down."""
        nonlocal active_session
        if active_session:
            print("Ending cloud agent session...")
            try:
                active_session.end()
            except Exception as e:
                print(f"  Session cleanup error: {e}")
            active_session = None
        print("Shutting down robot...")
        behavior.shutdown()
        print("Done.")

    # Register atexit so cleanup runs even on unexpected exit
    import atexit
    atexit.register(cleanup_and_exit)

    def handle_signal(signum, frame):
        nonlocal running
        running = False
        print(f"\nSignal {signum} received, shutting down...")
        cleanup_and_exit()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while running:
            # Wait for wake word
            listener.wait_for_wake_word()

            if not running:
                break

            # Start agent session
            active_session = AgentSession(behavior, agent_id, api_key)
            active_session.start()

            # Pause wake word listener during agent session (agent has its own mic)
            listener.pause()

            # Wait for session to end
            active_session.wait_for_end(idle_timeout=60)

            # End session cleanly
            active_session.end()
            active_session = None
            listener.resume()

            print("\nSession ended. Listening for wake word again...")
            print("")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup_and_exit()


if __name__ == "__main__":
    main()
