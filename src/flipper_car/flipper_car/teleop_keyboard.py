#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

BANNER = """
-------------------------------------------------------
🎮 Flipper Car Teleop (Physical Hardware Controller)
-------------------------------------------------------
Controls:
   W          : Forward
   S          : Reverse
   A          : Spin Left
   D          : Spin Right
   W + A      : Forward-Left
   W + D      : Forward-Right
   S + A      : Reverse-Left
   S + D      : Reverse-Right
   (No keys)  : STOP

Balancing Mode (Disabled by Default):
   B          : Toggle Self-Balancing (ON / OFF)

Gears & Boost:
   1, 2, 3    : 55%, 75%, 100% Speed
   SHIFT      : Instant Boost (100%)

   CTRL-C     : Quit
-------------------------------------------------------
"""

class MultiKeyTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_pub = self.create_publisher(String, '/flipper/command', 10)
        
        self.pressed_keys = set()
        self.last_cmd = "STOP"
        
        self.base_gear_cmd = "GEAR_1"
        self.base_gear_label = "1 (55%)"
        self.is_boosted = False
        self.last_sent_gear_cmd = "GEAR_1"
        self.balance_active = False  # Disabled by default
        
        print(BANNER)
        self.timer = self.create_timer(0.05, self.update_and_publish)

    def on_press(self, key):
        if key in [keyboard.Key.shift, keyboard.Key.shift_r]:
            if not self.is_boosted:
                self.is_boosted = True
                self.send_command("GEAR_3")
                sys.stdout.write(f"\r>>> ⚡ BOOST (100%) | Motion: {self.last_cmd:<12}\n")
                sys.stdout.flush()
            return

        try:
            if hasattr(key, 'char') and key.char:
                ch = key.char.lower()
                if ch == 'b':
                    self.balance_active = not self.balance_active
                    cmd = "BALANCE_ON" if self.balance_active else "BALANCE_OFF"
                    self.send_command(cmd)
                    status = "✅ ENABLED" if self.balance_active else "❌ DISABLED"
                    sys.stdout.write(f"\r>>> ⚖️ SELF-BALANCING: {status:<15}\n")
                    sys.stdout.flush()
                elif ch in ['1', '2', '3']:
                    gear_map = {
                        '1': ('GEAR_1', '1 (55%)'),
                        '2': ('GEAR_2', '2 (75%)'),
                        '3': ('GEAR_3', '3 (100%)')
                    }
                    self.base_gear_cmd, self.base_gear_label = gear_map[ch]
                    if not self.is_boosted:
                        self.send_command(self.base_gear_cmd)
                        sys.stdout.write(f"\r>>> GEAR: {self.base_gear_label:<10} | Motion: {self.last_cmd:<12}\n")
                        sys.stdout.flush()
                else:
                    self.pressed_keys.add(ch)
        except AttributeError:
            pass

    def on_release(self, key):
        if key in [keyboard.Key.shift, keyboard.Key.shift_r]:
            if self.is_boosted:
                self.is_boosted = False
                self.send_command(self.base_gear_cmd)
                sys.stdout.write(f"\r>>> BOOST RELEASED -> {self.base_gear_label:<10} | Motion: {self.last_cmd:<12}\n")
                sys.stdout.flush()
            return

        try:
            if hasattr(key, 'char') and key.char:
                ch = key.char.lower()
                self.pressed_keys.discard(ch)
        except AttributeError:
            pass

    def send_command(self, cmd_str: str):
        msg = String()
        msg.data = cmd_str
        self.cmd_pub.publish(msg)

    def resolve_command(self) -> str:
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

    def update_and_publish(self):
        current_cmd = self.resolve_command()
        
        cmd_msg = String()
        cmd_msg.data = current_cmd
        self.cmd_pub.publish(cmd_msg)

        if current_cmd != self.last_cmd:
            active_label = "⚡ BOOST" if self.is_boosted else self.base_gear_label
            bal_str = "ON" if self.balance_active else "OFF"
            sys.stdout.write(f"\rCmd: {current_cmd:<10} | Gear: {active_label:<10} | Bal: {bal_str:<3}")
            sys.stdout.flush()
            self.last_cmd = current_cmd

def main(args=None):
    rclpy.init(args=args)
    teleop_node = MultiKeyTeleop()

    listener = keyboard.Listener(
        on_press=teleop_node.on_press,
        on_release=teleop_node.on_release
    )
    listener.start()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = String()
        stop_msg.data = 'STOP'
        teleop_node.cmd_pub.publish(stop_msg)
        listener.stop()
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()