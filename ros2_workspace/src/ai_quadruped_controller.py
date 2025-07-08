#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import sys
import os

# Add the virtual environment path for PyTorch
venv_path = os.path.expanduser("~/quadruped_ai_env/lib/python3.12/site-packages")
if venv_path not in sys.path:
    sys.path.insert(0, venv_path)

import torch
import numpy as np
import time

class AIQuadrupedController(Node):
    def __init__(self):
        super().__init__('ai_quadruped_controller')
        
        # ROS2 Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Load trained PyTorch policy
        self.policy_path = "/mnt/d/IsaacLab/logs/rsl_rl/my_quadruped/2025-06-16_22-35-31/exported/policy.pt"
        self.load_policy()
        
        # Robot state initialization
        self.init_robot_state()
        
        # Control loop timer (20Hz for smooth motion)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Command velocities (from joystick/keyboard)
        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_angular_z = 0.0
        
        self.get_logger().info("🤖 AI Quadruped Controller initialized!")
        self.get_logger().info(f"📦 Loaded policy from: {self.policy_path}")

    def load_policy(self):
        """Load the trained PyTorch policy"""
        try:
            # Load the exported policy
            self.policy = torch.jit.load(self.policy_path, map_location='cpu')
            self.policy.eval()
            
            self.get_logger().info("✅ PyTorch policy loaded successfully!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load policy: {e}")
            # Continue without policy for testing
            self.policy = None

    def init_robot_state(self):
        """Initialize robot state variables"""
        # Joint names (matching your URDF)
        self.joint_names = [
            'front_left_hip', 'front_left_knee', 'front_left_ankle',
            'front_right_hip', 'front_right_knee', 'front_right_ankle', 
            'rear_left_hip', 'rear_left_knee', 'rear_left_ankle',
            'rear_right_hip', 'rear_right_knee', 'rear_right_ankle'
        ]
        
        # Current joint positions (initialize to neutral stance)
        self.joint_positions = np.array([
            0.0, 0.8, -1.6,  # front_left
            0.0, 0.8, -1.6,  # front_right  
            0.0, 0.8, -1.6,  # back_left
            0.0, 0.8, -1.6   # back_right
        ])
        
        # Step counter for gait timing
        self.step_count = 0

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from joystick/keyboard"""
        self.cmd_linear_x = msg.linear.x
        self.cmd_linear_y = msg.linear.y  
        self.cmd_angular_z = msg.angular.z
        
        self.get_logger().info(f"🎮 CMD: vx={self.cmd_linear_x:.2f}, vy={self.cmd_linear_y:.2f}, wz={self.cmd_angular_z:.2f}")

    def build_observation(self):
        """Build observation vector matching Isaac Lab training (72 dimensions)"""
    
        # 1. Base linear velocity (3) - simulated for now
        base_lin_vel = np.array([self.cmd_linear_x * 0.5, self.cmd_linear_y * 0.5, 0.0])
    
        # 2. Base angular velocity (3) - simulated for now  
        base_ang_vel = np.array([0.0, 0.0, self.cmd_angular_z * 0.5])
    
        # 3. Projected gravity (3) - gravity vector in robot frame
        projected_gravity = np.array([0.0, 0.0, -1.0])
    
        # 4. Velocity commands (3) - current commands
        velocity_commands = np.array([self.cmd_linear_x, self.cmd_linear_y, self.cmd_angular_z])
    
        # 5. Joint positions relative (12) - current joint positions
        joint_pos_rel = self.joint_positions.copy()
    
        # 6. Joint velocities relative (12) - placeholder zeros for now
        joint_vel_rel = np.zeros(12)
    
        # 7. Last actions (12) - previous joint commands
        if not hasattr(self, 'last_actions'):
            self.last_actions = self.joint_positions.copy()
        actions = self.last_actions.copy()
    
        # 8. Feet forces (24) - 4 feet × 6 force/torque components, placeholder zeros
        feet_forces = np.zeros(24)
    
        # Concatenate all observations (should be 72 dimensions)
        observation = np.concatenate([
            base_lin_vel,      # 3
            base_ang_vel,      # 3  
            projected_gravity, # 3
            velocity_commands, # 3
            joint_pos_rel,     # 12
            joint_vel_rel,     # 12
            actions,           # 12
            feet_forces        # 24
        ])
    
        # Debug: print observation size (remove this after testing)
        if self.step_count % 100 == 0:  # Print every 100 steps
            self.get_logger().info(f"📊 Observation shape: {observation.shape}")
    
        return observation

    def control_loop(self):
        """Main control loop"""
        try:
            if self.policy is None:
                # Fallback: simple walking pattern for testing
                self.simple_walking_pattern()
            else:
                # AI control
                self.ai_control()
            
            # Publish joint states
            self.publish_joint_states()
            self.step_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Control loop error: {e}")

    def simple_walking_pattern(self):
        """Simple walking pattern for testing without policy"""
        if abs(self.cmd_linear_x) > 0.01 or abs(self.cmd_angular_z) > 0.01:
            # Simple leg movement
            phase = self.step_count * 0.1
            self.joint_positions[1] = 0.8 + 0.3 * np.sin(phase)  # front_left_knee
            self.joint_positions[4] = 0.8 + 0.3 * np.sin(phase + np.pi)  # front_right_knee

    def ai_control(self):
        """AI policy control"""
        obs = self.build_observation()
        obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
        
        with torch.no_grad():
            action = self.policy(obs_tensor)
            action = action.squeeze().cpu().numpy()
        
        # Save last actions for next observation
        self.last_actions = action.copy()

        # Scale actions to joint ranges (simplified)
        self.joint_positions = np.clip(action * 0.5, -2.0, 2.0)

    def publish_joint_states(self):
        """Publish current joint states to ROS"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions.tolist()
        
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = AIQuadrupedController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down AI controller...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
