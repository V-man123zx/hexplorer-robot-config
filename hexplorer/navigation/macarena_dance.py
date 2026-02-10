#!/usr/bin/env python3
"""
Macarena Dance for the Dobot Hexplorer!

Choreographs a Macarena-inspired dance using walking movements:
  - Side steps (strafe left/right)
  - Forward/back steps
  - Spins (the signature Macarena jump-turn)

Usage:
    source ~/robot_controller_release/ros2_packages/setup.bash
    python3 ~/hexplorer/navigation/macarena_dance.py
"""
import rclpy
from rclpy.node import Node
from custom_msg.msg import RobotCommand
from geometry_msgs.msg import Twist
import time
import signal
import sys


class MacarenaDancer:
    def __init__(self):
        rclpy.init()
        self.node = Node('macarena_dancer')
        self.cmd_pub = self.node.create_publisher(RobotCommand, '/robot_cmd', 10)
        self.vel_pub = self.node.create_publisher(Twist, '/vel_cmd', 10)
        time.sleep(0.3)

        self.cmd = RobotCommand()
        self.vel = Twist()
        self.running = True

        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        print("\n[Macarena] Stopping dance...")
        self.running = False

    def _publish(self, state, lx=0.0, ly=0.0, az=0.0, duration=1.0):
        """Publish command at 20Hz for given duration."""
        self.cmd.target_state = state
        self.vel.linear.x = lx
        self.vel.linear.y = ly
        self.vel.angular.z = az
        ticks = int(duration / 0.05)
        for _ in range(ticks):
            if not self.running:
                return
            self.cmd_pub.publish(self.cmd)
            self.vel_pub.publish(self.vel)
            time.sleep(0.05)

    def _stop_motion(self, duration=0.3):
        """Brief pause between moves."""
        self._publish(4, 0.0, 0.0, 0.0, duration)

    def stand_up(self):
        """Stand up sequence."""
        print("[Macarena] Standing up...")
        for state in [1, 2, 3]:
            self._publish(state, duration=2.0)
            if not self.running:
                return

    def sit_down(self):
        """Sit down safely."""
        print("[Macarena] Taking a bow... sitting down.")
        self._publish(3, duration=1.0)
        self._publish(1, duration=2.0)
        self._publish(0, duration=1.0)

    def dance_macarena(self, cycles=2):
        """
        Macarena choreography!

        Each cycle roughly follows the Macarena beat:
          1. Right arm out  -> step right
          2. Left arm out   -> step left
          3. Right arm flip -> lean forward
          4. Left arm flip  -> lean forward again
          5. Right to left shoulder -> turn right
          6. Left to right shoulder -> turn left
          7. Right to head  -> step forward
          8. Left to head   -> step forward
          9. Right to left hip  -> step back
         10. Left to right hip  -> step back
         11. Hip wiggle     -> wiggle (quick left-right turns)
         12. JUMP & TURN!   -> 90-degree spin!
        """
        beat = 0.55  # Duration per beat (~110 BPM Macarena tempo)

        # Enter walk mode
        print("[Macarena] Entering walk mode...")
        self._publish(4, duration=1.5)

        for cycle in range(cycles):
            if not self.running:
                break
            quarter = cycle + 1
            print(f"\n[Macarena] === Cycle {quarter}/{cycles} === HEEEY MACARENA!")

            # 1. Step right (right arm out)
            print("  -> Right arm out! (step right)")
            self._publish(4, ly=-0.15, duration=beat)
            self._stop_motion(0.2)

            # 2. Step left (left arm out)
            print("  -> Left arm out! (step left)")
            self._publish(4, ly=0.15, duration=beat)
            self._stop_motion(0.2)

            # 3. Right arm flip -> lean forward
            print("  -> Flip right! (forward)")
            self._publish(4, lx=0.15, duration=beat)
            self._stop_motion(0.2)

            # 4. Left arm flip -> lean forward again
            print("  -> Flip left! (forward)")
            self._publish(4, lx=0.15, duration=beat)
            self._stop_motion(0.2)

            # 5. Right hand to left shoulder -> turn right
            print("  -> Cross right! (turn right)")
            self._publish(4, az=-0.3, duration=beat)
            self._stop_motion(0.2)

            # 6. Left hand to right shoulder -> turn left
            print("  -> Cross left! (turn left)")
            self._publish(4, az=0.3, duration=beat)
            self._stop_motion(0.2)

            # 7. Right hand to head -> step forward
            print("  -> Right to head! (forward)")
            self._publish(4, lx=0.2, duration=beat)
            self._stop_motion(0.2)

            # 8. Left hand to head -> step forward
            print("  -> Left to head! (forward)")
            self._publish(4, lx=0.2, duration=beat)
            self._stop_motion(0.2)

            # 9. Right hand to hip -> step back
            print("  -> Right hip! (back)")
            self._publish(4, lx=-0.15, duration=beat)
            self._stop_motion(0.2)

            # 10. Left hand to hip -> step back
            print("  -> Left hip! (back)")
            self._publish(4, lx=-0.15, duration=beat)
            self._stop_motion(0.2)

            # 11. Hip wiggle! (quick alternating turns)
            print("  -> HIP WIGGLE!")
            for _ in range(3):
                if not self.running:
                    break
                self._publish(4, az=-0.4, duration=0.25)
                self._publish(4, az=0.4, duration=0.25)
            self._stop_motion(0.2)

            # 12. HEEEY MACARENA! Jump turn (90-degree spin!)
            print("  -> HEEEY MACARENA! (quarter turn!)")
            self._publish(4, az=0.5, duration=1.2)
            self._stop_motion(0.5)

        print("\n[Macarena] Dance complete!")

    def run(self):
        print("=" * 50)
        print("   MACARENA DANCE - Hexplorer Edition!")
        print("=" * 50)
        print("Press Ctrl+C to stop at any time.\n")

        self.stand_up()
        if self.running:
            time.sleep(0.5)
            self.dance_macarena(cycles=3)
        self.sit_down()

        self.node.destroy_node()
        rclpy.shutdown()
        print("[Macarena] Done! Aight!")


if __name__ == '__main__':
    dancer = MacarenaDancer()
    dancer.run()
