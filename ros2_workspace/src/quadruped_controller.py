#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import serial
import time
import math

class QuadrupedController(Node):
    def __init__(self):
        super().__init__('quadruped_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # Joint name to servo channel mapping
        self.joint_to_channel = {
            'front_left_hip': 0,
            'front_left_knee': 1, 
            'front_left_ankle': 2,
            'front_right_hip': 3,
            'front_right_knee': 4,
            'front_right_ankle': 5,
            'rear_left_hip': 9,
            'rear_left_knee': 10,
            'rear_left_ankle': 11,
            'rear_right_hip': 6,
            'rear_right_knee': 7,
            'rear_right_ankle': 8
        }
        
        # Reverse mapping for feedback
        self.channel_to_joint = {v: k for k, v in self.joint_to_channel.items()}
        
        # Current joint positions (in degrees)
        self.current_positions = {}
        for joint in self.joint_to_channel.keys():
            if 'hip' in joint:
                self.current_positions[joint] = 0.0  # Hip joints at 90°
            else:  # knee and ankle joints
                self.current_positions[joint] = 0.0  # Knee/ankle joints at 0 degrees
        
        # Initialize serial connection
        self.serial_connection = None
        self.connect_to_arduino()
        
        # ROS2 Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Individual joint command subscriber for testing
        self.joint_cmd_sub = self.create_subscription(
            Float64,
            '/quadruped_joint_cmd',
            self.test_joint_callback,
            10
        )
        
        # Publisher for joint feedback
        self.joint_feedback_pub = self.create_publisher(
            JointState,
            '/quadruped_feedback',
            10
        )
        
        # Timer for status updates
        self.status_timer = self.create_timer(2.0, self.publish_feedback)
        
        self.get_logger().info(f'Quadruped Controller started on {self.serial_port}')
        self.get_logger().info(f'Joint mapping: {list(self.joint_to_channel.keys())}')
        self.get_logger().info(f'Listening for joint states on /joint_states')
        self.get_logger().info(f'Test commands on /quadruped_joint_cmd')
        
    def connect_to_arduino(self):
        """Connect to Arduino via serial"""
        try:
            self.serial_connection = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=1
            )
            time.sleep(3)  # Wait for Arduino to initialize
            
            # Read any startup messages
            while self.serial_connection.in_waiting > 0:
                msg = self.serial_connection.readline().decode('utf-8').strip()
                if msg:
                    self.get_logger().info(f'Arduino: {msg}')
                
            self.get_logger().info('Successfully connected to Arduino')
            
            # Test connection
            if self.send_arduino_command('echo test'):
                self.get_logger().info('Arduino communication verified')

            # Move to default positions to match URDF
            self.get_logger().info('Setting default joint positions...')
            for joint_name, default_angle in self.current_positions.items():
                self.move_joint(joint_name, default_angle)
                time.sleep(0.1)  # Brief delay between servo commands
            self.get_logger().info('Default positions set')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial_connection = None
    
    def send_arduino_command(self, command):
        """Send command to Arduino and read response"""
        if self.serial_connection is None:
            self.get_logger().error('No Arduino connection')
            return False

        try:
            # Send command
            self.serial_connection.write(f'{command}\n'.encode('utf-8'))
            self.get_logger().info(f'ACTUAL COMMAND SENT: {command}')  # ADD THIS LINE
        
            # Give Arduino time to process and respond
            time.sleep(0.2)
        
            # Read ALL available responses
            while self.serial_connection.in_waiting > 0:
                response = self.serial_connection.readline().decode('utf-8').strip()
                if response:
                    self.get_logger().info(f'Arduino: {response}')  # Changed to INFO level

            return True
        except Exception as e:
            self.get_logger().error(f'Serial communication error: {e}')
            return False
    
    def test_joint_callback(self, msg):
        """Handle test joint command (for testing) - expects angle in degrees"""
        angle_degrees = msg.data
        
        # Test command - move front_left_hip
        test_joint = 'front_left_hip'
        self.move_joint(test_joint, angle_degrees)
    
    def joint_state_callback(self, msg):
        """Handle joint state messages from joint_state_publisher or other sources"""
        
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_to_channel and i < len(msg.position):
                # Convert radians to degrees
                angle_radians = msg.position[i]
                angle_degrees = math.degrees(angle_radians)
                
                # Joint-specific angle mapping to match robot's natural pose
                if 'hip' in joint_name:
                    # Hip joints: URDF 0° = standing position (servo 90°)
                    servo_angle = angle_degrees + 90
                else:
                    # Knee/ankle joints: URDF 0° = straight legs (servo 0°)
                    servo_angle = angle_degrees

                # Expanded clamping to handle negative URDF values
                # Map URDF range (-180° to +180°) to servo range (0° to 180°)
                servo_angle = max(-180, min(360, servo_angle))  # First clamp to reasonable range
                servo_angle = max(-180, min(360, servo_angle))     # Then clamp to servo range
                
                self.get_logger().info(f'{joint_name}: {angle_degrees:.1f}° -> servo {servo_angle:.1f}°')
                
                # Move the joint
                self.move_joint(joint_name, servo_angle)
    
    def move_joint(self, joint_name, angle_degrees):
        """Move a specific joint to the given angle (in degrees)"""
        
        if joint_name not in self.joint_to_channel:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return False
        
        # Get servo channel
        channel = self.joint_to_channel[joint_name]
        
        # Constrain angle
        angle_degrees = max(-180, min(360, angle_degrees))

        # List of joints that need servo direction inversion
        inverted_joints = {
            'rear_left_knee',        # Keep this one
            'front_right_knee',      # ADD: was 180° off, needs inversion
            'rear_right_knee',       # ADD: was 180° off, needs inversion  
            'front_right_ankle',     # Keep this one
            'rear_right_ankle',      # Keep this one
            'rear_left_ankle'        # Keep this one
        }
    
        # Invert servo angle for problematic joints
        if joint_name in inverted_joints:
            servo_angle = 180 - abs(angle_degrees)  # Use absolute value before inversion
            self.get_logger().debug(f'Inverting {joint_name}: {angle_degrees}° -> {servo_angle}°')
        else:
            servo_angle = angle_degrees
        
        # Send command to Arduino
        command = f'servo {channel} {int(servo_angle)}'
        success = self.send_arduino_command(command)
        
        if success:
            self.current_positions[joint_name] = angle_degrees
            self.get_logger().info(f'Moved {joint_name} (ch{channel}) to {angle_degrees}°')
        
        return success
    
    def publish_feedback(self):
        """Publish current joint positions as feedback"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Add all joints
        joint_msg.name = []
        joint_msg.position = []
        
        for joint_name, angle_degrees in self.current_positions.items():
            joint_msg.name.append(joint_name)
            # Convert back to radians (servo angle 0-180 -> joint angle -90 to +90)
            angle_radians = math.radians(angle_degrees - 90)
            joint_msg.position.append(angle_radians)
        
        self.joint_feedback_pub.publish(joint_msg)
    
    def destroy_node(self):
        """Clean shutdown"""
        if self.serial_connection:
            self.get_logger().info('Closing Arduino connection')
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = QuadrupedController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
