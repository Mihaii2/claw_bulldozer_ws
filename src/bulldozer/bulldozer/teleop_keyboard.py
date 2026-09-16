#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

BANNER = """
=======================================================
 🚜 BULLDOZER CLAW ROS 2 TELEOP (FULL DUPLEX)
=======================================================
 Drive (Left Hand):
    [W] Forward    [S] Reverse    [A] Left    [D] Right
    Combinations (W+A, W+D, S+A, S+D) Supported.

 Gears:
    [1] 55% PWM    [2] 75% PWM    [3] 100% PWM
    [SHIFT] Hold for 100% Boost

 Arm / Cup / Claw (Hold to move continuously):
    [U] Cup Down   [J] Cup Up
    [I] Arm Up     [K] Arm Down
    [O] Claw Open  [L] Claw Close

 [CTRL-C] : Quit
=======================================================
"""

class BulldozerTeleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.drive_pub = self.create_publisher(String, '/bulldozer/drive_cmd', 10)
        self.arm_pub = self.create_publisher(String, '/bulldozer/arm_cmd', 10)

        # Track active keys
        self.pressed_keys = set()
        self.last_drive_cmd = "STOP"

        self.base_gear_cmd = "GEAR_1"
        self.base_gear_label = "1 (55%)"
        self.is_boosted = False

        print(BANNER)
        # Dedicated 20 Hz tick loop (50 ms)
        self.timer = self.create_timer(0.05, self.control_loop)

    def publish_drive(self, cmd_str: str):
        msg = String()
        msg.data = cmd_str
        self.drive_pub.publish(msg)

    def publish_arm(self, cmd_str: str):
        msg = String()
        msg.data = cmd_str
        self.arm_pub.publish(msg)

    def on_press(self, key):
        if key in [keyboard.Key.shift, keyboard.Key.shift_r]:
            if not self.is_boosted:
                self.is_boosted = True
                self.publish_drive("GEAR_3")
                sys.stdout.write(f"\r>>> ⚡ BOOST ON\n")
                sys.stdout.flush()
            return

        try:
            if hasattr(key, 'char') and key.char:
                ch = key.char.lower()
                if ch in ['1', '2', '3']:
                    gears = {'1': ('GEAR_1', '1 (55%)'), '2': ('GEAR_2', '2 (75%)'), '3': ('GEAR_3', '3 (100%)')}
                    self.base_gear_cmd, self.base_gear_label = gears[ch]
                    if not self.is_boosted:
                        self.publish_drive(self.base_gear_cmd)
                        sys.stdout.write(f"\r>>> GEAR: {self.base_gear_label}\n")
                        sys.stdout.flush()
                else:
                    self.pressed_keys.add(ch)
        except AttributeError:
            pass

    def on_release(self, key):
        if key in [keyboard.Key.shift, keyboard.Key.shift_r]:
            if self.is_boosted:
                self.is_boosted = False
                self.publish_drive(self.base_gear_cmd)
                sys.stdout.write(f"\r>>> BOOST OFF\n")
                sys.stdout.flush()
            return

        try:
            if hasattr(key, 'char') and key.char:
                ch = key.char.lower()
                self.pressed_keys.discard(ch)
        except AttributeError:
            pass

    def resolve_drive_cmd(self) -> str:
        w = 'w' in self.pressed_keys
        s = 's' in self.pressed_keys
        a = 'a' in self.pressed_keys
        d = 'd' in self.pressed_keys

        if w and not s:
            if a and not d: return "FWD_LEFT"
            if d and not a: return "FWD_RIGHT"
            return "FORWARD"
        if s and not w:
            if a and not d: return "REV_LEFT"
            if d and not a: return "REV_RIGHT"
            return "REVERSE"
        if a and not d: return "SPIN_LEFT"
        if d and not a: return "SPIN_RIGHT"
        return "STOP"

    def control_loop(self):
        # 1. Handle Drive
        current_drive = self.resolve_drive_cmd()
        if current_drive != "STOP":
            self.publish_drive(current_drive)
        elif self.last_drive_cmd != "STOP":
            self.publish_drive("STOP")
        self.last_drive_cmd = current_drive

        # 2. Handle Manipulator (Processed concurrently every 50ms while key is held)
        if 'u' in self.pressed_keys: self.publish_arm("CUP_DOWN")
        if 'j' in self.pressed_keys: self.publish_arm("CUP_UP")
        if 'i' in self.pressed_keys: self.publish_arm("ARM_UP")
        if 'k' in self.pressed_keys: self.publish_arm("ARM_DOWN")
        if 'o' in self.pressed_keys: self.publish_arm("CLAW_OPEN")
        if 'l' in self.pressed_keys: self.publish_arm("CLAW_CLOSE")

def main(args=None):
    rclpy.init(args=args)
    teleop_node = BulldozerTeleop()
    listener = keyboard.Listener(on_press=teleop_node.on_press, on_release=teleop_node.on_release)
    listener.start()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.publish_drive("STOP")
        listener.stop()
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()