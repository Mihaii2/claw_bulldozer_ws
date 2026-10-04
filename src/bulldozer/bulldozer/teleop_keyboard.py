#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

BANNER = """
=======================================================
 🚜 BULLDOZER CLAW ROS 2 TELEOP (2-GEAR MODE)
=======================================================
 Drive (Left Hand):
    [W] Forward    [S] Reverse    [A] Left    [D] Right
    Combinations (W+A, W+D, S+A, S+D) Supported.

 Gears:
    [1] Gear 1 (65% PWM)
    [2] Gear 2 (100% PWM - DEFAULT)

 Arm / Cup / Claw / Phone:
    [I] Arm Up     [K] Arm Down
    [U] Cup Up     [J] Cup Down
    [O] Claw Close [L] Claw Open
    [H] Phone Up   [Y] Phone Down

 [CTRL-C] : Quit
=======================================================
"""

class BulldozerTeleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.drive_pub = self.create_publisher(String, '/bulldozer/drive_cmd', 10)
        self.arm_pub = self.create_publisher(String, '/bulldozer/arm_cmd', 10)

        self.pressed_keys = set()
        self.last_drive_cmd = "STOP"

        # Start default at 255 (Gear 2)
        self.current_duty = 255
        self.current_gear_label = "2 (100%)"

        print(BANNER)
        sys.stdout.write(f"\r>>> CURRENT GEAR: {self.current_gear_label}\n")
        sys.stdout.flush()

        self.timer = self.create_timer(0.05, self.control_loop)

    def publish_drive(self, cmd_str: str):
        msg = String()
        if cmd_str == "STOP":
            msg.data = "STOP"
        else:
            msg.data = f"{cmd_str}:{self.current_duty}"
        
        # Log to terminal so you can physically see the duty cycle being emitted
        sys.stdout.write(f"\r[PUB DRIVE] -> {msg.data:20s}\n")
        sys.stdout.flush()
        
        self.drive_pub.publish(msg)

    def publish_arm(self, cmd_str: str):
        msg = String()
        msg.data = cmd_str
        self.arm_pub.publish(msg)

    def on_press(self, key):
        # Support both regular '1'/'2' and numpad keys
        key_str = None
        try:
            if hasattr(key, 'char') and key.char:
                key_str = key.char.lower()
        except Exception:
            pass

        if key_str is None:
            # Check for KeyCode or Key representation
            key_repr = str(key).strip("'")
            if '1' in key_repr:
                key_str = '1'
            elif '2' in key_repr:
                key_str = '2'

        if key_str == '1':
            self.current_duty = 166
            self.current_gear_label = "1 (65%)"
            sys.stdout.write(f"\r>>> GEAR SELECTED: {self.current_gear_label} (DUTY={self.current_duty})\n")
            sys.stdout.flush()
            return
        elif key_str == '2':
            self.current_duty = 255
            self.current_gear_label = "2 (100%)"
            sys.stdout.write(f"\r>>> GEAR SELECTED: {self.current_gear_label} (DUTY={self.current_duty})\n")
            sys.stdout.flush()
            return

        if key_str:
            self.pressed_keys.add(key_str)

    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                self.pressed_keys.discard(key.char.lower())
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
        current_drive = self.resolve_drive_cmd()
        if current_drive != "STOP":
            self.publish_drive(current_drive)
        elif self.last_drive_cmd != "STOP":
            self.publish_drive("STOP")
        self.last_drive_cmd = current_drive

        if 'i' in self.pressed_keys: self.publish_arm("ARM_UP")
        if 'k' in self.pressed_keys: self.publish_arm("ARM_DOWN")
        if 'u' in self.pressed_keys: self.publish_arm("CUP_UP")
        if 'j' in self.pressed_keys: self.publish_arm("CUP_DOWN")
        if 'o' in self.pressed_keys: self.publish_arm("CLAW_CLOSE")
        if 'l' in self.pressed_keys: self.publish_arm("CLAW_OPEN")
        if 'h' in self.pressed_keys: self.publish_arm("CAM_UP")
        if 'y' in self.pressed_keys: self.publish_arm("CAM_DOWN")

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