#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current velocities
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # Control parameters
        self.max_linear_speed = 0.5   # m/s (start slow)
        self.max_angular_speed = 0.5  # rad/s
        self.speed_increment = 0.1
        
        # Terminal settings for keyboard input
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Timer for publishing commands
        self.publish_timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info("🎮 Velocity Controller started!")
        self.print_instructions()
        
        # Start keyboard input in separate thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*50)
        print("🤖 QUADRUPED VELOCITY CONTROL")
        print("="*50)
        print("MOVEMENT:")
        print("  w/s : Forward/Backward")
        print("  a/d : Left/Right") 
        print("  q/e : Turn Left/Right")
        print("  x   : Stop")
        print("  SPACE : Emergency Stop")
        print("  CTRL+C : Quit")
        print("="*50)
        print(f"Max Linear Speed: {self.max_linear_speed} m/s")
        print(f"Max Angular Speed: {self.max_angular_speed} rad/s")
        print("="*50)

    def get_key(self):
        """Get single keypress without Enter"""
        tty.setcbreak(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_loop(self):
        """Keyboard input loop running in separate thread"""
        while self.running:
            try:
                key = self.get_key()
                if key:
                    self.process_key(key)
            except:
                break

    def process_key(self, key):
        """Process keyboard input"""
        # Movement keys
        if key == 'w':
            self.linear_x = min(self.linear_x + self.speed_increment, self.max_linear_speed)
        elif key == 's':
            self.linear_x = max(self.linear_x - self.speed_increment, -self.max_linear_speed)
        elif key == 'a':
            self.linear_y = min(self.linear_y + self.speed_increment, self.max_linear_speed)
        elif key == 'd':
            self.linear_y = max(self.linear_y - self.speed_increment, -self.max_linear_speed)
        elif key == 'q':
            self.angular_z = min(self.angular_z + self.speed_increment, self.max_angular_speed)
        elif key == 'e':
            self.angular_z = max(self.angular_z - self.speed_increment, -self.max_angular_speed)
        elif key == 'x':
            # Gradual stop
            self.linear_x *= 0.5
            self.linear_y *= 0.5
            self.angular_z *= 0.5
            if abs(self.linear_x) < 0.01: self.linear_x = 0.0
            if abs(self.linear_y) < 0.01: self.linear_y = 0.0
            if abs(self.angular_z) < 0.01: self.angular_z = 0.0
        elif key == ' ':
            # Emergency stop
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
            print("\n🛑 EMERGENCY STOP!")

        self.print_status()

    def publish_velocity(self):
        """Publish velocity command"""
        cmd = Twist()
        cmd.linear.x = self.linear_x
        cmd.linear.y = self.linear_y
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.angular_z
        
        self.cmd_vel_pub.publish(cmd)

    def print_status(self):
        """Print current velocity status"""
        print(f"\r🎮 Vel: [{self.linear_x:+.2f}, {self.linear_y:+.2f}] m/s | Turn: {self.angular_z:+.2f} rad/s", end='', flush=True)

    def stop_robot(self):
        """Send stop command"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.running = False
        print("\n🛑 Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    
    controller = None
    try:
        controller = VelocityController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    finally:
        if controller:
            controller.stop_robot()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
