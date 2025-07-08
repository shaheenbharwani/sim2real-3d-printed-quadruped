# AI Quadruped Robot: Complete Sim-to-Real Pipeline

![Robot Demo](media/robot_hero_shot.jpg)

**Complete open-source quadruped robot with AI-driven locomotion trained in Isaac Lab and deployed on physical hardware via ROS2.**

## 🤖 Overview

This project demonstrates a full sim-to-real transfer pipeline: training a quadruped robot to walk in Isaac Lab simulation, then deploying the trained neural network to a physical 3D-printed robot via ROS2 and Arduino.

## ✨ Features

- **Full sim-to-real pipeline**: Isaac Lab → PyTorch → ROS2 → Physical robot
- **AI-powered locomotion**: Neural network trained with reinforcement learning
- **Real-time control**: 20Hz inference with responsive keyboard control
- **Modular design**: Easy to modify and extend
- **Complete documentation**: Hardware assembly to software deployment

## 🎯 What You'll Build

A 12-DOF quadruped robot controlled by an AI trained in simulation that responds to velocity commands and walks autonomously.

## 📊 Performance Metrics

- **Training**: 999 episodes in Isaac Lab
- **Control Frequency**: 20Hz AI inference, 5Hz Arduino communication  
- **Servo Count**: 12x MG996R servos
- **Power**: External 6V supply
- **Communication**: USB serial (Windows → WSL → Arduino)
- **Observation Space**: 72 dimensions
- **Action Space**: 12 joint position targets

## 🛠️ Hardware Specifications

### Core Components
- **12x MG996R Servos** (3 per leg: hip, knee, ankle)
- **Arduino UNO R3** with PCA9685 I2C servo driver
- **External 6V power supply** for servos
- **3D printed chassis** (STL files included)

### Joint Configuration
- **DOF**: 12 total (3 per leg)
- **Joint Names**: `front_left_hip`, `front_left_knee`, `front_left_ankle`, etc.
- **Range**: 0-180 degrees with custom angle conversions
- **Control**: Position control via PWM

## 💻 Software Stack

### Training Environment
- **Isaac Lab**: NVIDIA's robotics simulation platform
- **Robot Model**: Custom URDF with 165 mesh components
- **Training Algorithm**: Reinforcement learning via RSL-RL
- **Export Format**: PyTorch JIT (.pt)

### Deployment Stack
- **ROS2 Jazzy**: Real-time robot control system
- **Ubuntu 22.04 WSL**: Development environment
- **PyTorch**: Neural network inference engine
- **Arduino**: Hardware interface and servo control

## 🚀 Quick Start

### 1. Hardware Assembly
```bash
# See docs/hardware/assembly.md for detailed instructions
# Required: 3D printer, basic electronics tools
```

### 2. Software Setup
```bash
# Install ROS2 Jazzy
sudo apt update && sudo apt install ros-jazzy-desktop

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone <this-repo>
cd ..

# Build packages
colcon build
source install/setup.bash
```

### 3. AI Training (Optional)
```bash
# Install Isaac Lab
# See docs/training/isaac_lab_setup.md

# Train model (or use provided pre-trained model)
python scripts/train_quadruped.py
```

### 4. Robot Deployment
```bash
# Terminal 1: Hardware interface
ros2 run arduino_control quadruped_controller

# Terminal 2: AI controller  
ros2 run arduino_control ai_controller

# Terminal 3: Keyboard control
ros2 run arduino_control velocity_controller
```

## 🎮 Controls

- **W**: Forward motion
- **S**: Backward motion  
- **A**: Strafe left
- **D**: Strafe right
- **Q**: Turn left
- **E**: Turn right
- **X**: Gradual stop
- **SPACE**: Emergency stop

## 📁 Repository Structure

```
sim2real-3d-printed-quadruped/
├── README.md                   # This file
├── docs/                       # Documentation
│   ├── hardware/              # Assembly guides, BOM
│   ├── software/              # Installation, setup  
│   └── training/              # Isaac Lab guides
├── hardware/                   # CAD files, URDF, electronics
├── ros2_workspace/            # ROS2 packages
│   └── src/                   # AI controller, hardware interface
├── arduino/                    # Arduino firmware
├── simulation/                 # Isaac Lab files, trained models
└── media/                     # Videos, images, demos
```

## 🔬 Technical Achievements

1. **Sim-to-Real Transfer**: Successfully deployed Isaac Lab trained model on physical hardware
2. **Real-time AI Inference**: PyTorch model running at 20Hz on physical robot  
3. **Robust Communication**: Reliable ROS2 → Arduino → Servo pipeline
4. **Modular Architecture**: Separate controllers for AI, hardware, and user input
5. **Production-Ready**: Stable, error-free operation for extended periods

## 📈 System Architecture

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌─────────────┐
│ Isaac Lab   │───▶│ PyTorch      │───▶│ ROS2        │───▶│ Arduino     │
│ Training    │    │ Model (.pt)  │    │ AI Node     │    │ + PCA9685   │
└─────────────┘    └──────────────┘    └─────────────┘    └─────────────┘
                                              │                    │
┌─────────────┐    ┌──────────────┐          │                    ▼
│ Keyboard    │───▶│ Velocity     │──────────┘            ┌─────────────┐
│ Input       │    │ Commands     │                       │ 12x Servos  │
└─────────────┘    └──────────────┘                       │ (Physical   │
                                                          │ Robot)      │
                                                          └─────────────┘
```

## 🧠 AI Model Details

- **Observation Vector**: 72 dimensions
  - Base velocity (6D)
  - Gravity vector (3D)  
  - Velocity commands (3D)
  - Joint positions (12D)
  - Joint velocities (12D)
  - Previous actions (12D)
  - Foot forces (24D)

- **Action Vector**: 12 dimensions (joint position targets)
- **Network Architecture**: Policy network exported from Isaac Lab
- **Training Time**: 999 episodes (~2 hours)

## 🔧 Hardware Interface

### Arduino Firmware Features
- **Serial Communication**: Receives joint commands via USB
- **Servo Control**: PCA9685 I2C driver for 12 servos
- **Angle Conversion**: Handles negative angles for specific joints
- **Safety Limits**: Constrains servo range to prevent damage

### ROS2 Controllers
1. **AI Controller**: Loads PyTorch model, runs inference, publishes joint commands
2. **Velocity Controller**: Processes keyboard input, publishes velocity commands  
3. **Hardware Controller**: Interfaces with Arduino, converts ROS messages to serial

## 🎯 Future Enhancements

- **Vision Integration**: Camera-based navigation
- **Advanced Gaits**: Implement trot, bound, gallop
- **Terrain Adaptation**: Rough terrain walking
- **Mobile Control**: Smartphone app interface
- **Sensor Fusion**: IMU, encoders, force sensors

## 🤝 Contributing

Contributions welcome! Areas of focus:
- **Hardware**: CAD improvements, new components
- **Software**: Better algorithms, new features  
- **Documentation**: Tutorials, troubleshooting
- **Testing**: Hardware validation, performance optimization

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## 📜 License

MIT License - see [LICENSE](LICENSE)

## 🙏 Acknowledgments

- **NVIDIA Isaac Lab** for simulation platform
- **ROS2 Community** for robotics middleware
- **Arduino Community** for hardware interface tools

## 📞 Support

- **Issues**: Use GitHub Issues for bug reports
- **Discussions**: GitHub Discussions for questions
- **Wiki**: Detailed technical documentation

---

**Built with ❤️ for the open-source robotics community**