# Phone2Act: Smartphone-Based Teleoperation for Robotic Arms

<div align="center">

**Real-time robot control from your phone. No wires. No constraints.**

[![Teleoperation Demo - Dobot](docs/images/dobot_demo.png)](https://www.youtube.com/watch?v=...)
[![Teleoperation Demo - LeRobot](docs/images/lerobot_demo.png)](https://www.youtube.com/watch?v=...)

[📄 Paper](link/to/arxiv) | [📱 Android App](#phone2act-app) | [🛠️ Setup Guide](#setup-guide) | [📦 API](#configuration)

</div>

---

## 📋 Overview

Phone2Act is an open-source framework that enables intuitive robot teleoperation using a smartphone. Users hold their phone and move naturally—the system automatically translates their motion into robot end-effector commands. 

**Key Features:**
- 📱 **Smartphone-based control** — Use your Android device as a 6-DOF controller
- 🤖 **Multi-robot support** — Works with Dobot, LeRobot, and custom robots
- ⚙️ **Zero-code customization** — Configure via YAML, no code changes needed
- 🔒 **Safety-first design** — Workspace limits, collision avoidance, rate limiting
- 🌐 **WebSocket-based** — Low-latency communication over WiFi
- 🎯 **Precise IK solving** — Real-time inverse kinematics on target hardware

---

## System Architecture

```
┌─────────────────────┐
│   Smartphone        │
│  (Phone2Act App)    │
│  - AR Pose Est.     │
│  - Phone Sensors    │
└────────┬────────────┘
         │ WebSocket (WiFi)
         │ /bros2/ar_pose
         │ /bros2/volume
         ▼
┌─────────────────────────────────────┐
│   ROS 2 (Laptop/Computer)           │
│  ┌──────────────────────────────┐   │
│  │ Phone2Act Planner Node       │   │
│  │ - Pose → Target Transform    │   │
│  │ - Workspace Clamping         │   │
│  │ - Safety Filtering           │   │
│  └──────────┬───────────────────┘   │
│             │ /phone2act/target_pose
│             │ /phone2act/gripper_cmd
│  ┌──────────▼───────────────────┐   │
│  │ Hardware Bridge              │   │
│  │ - IK Solver (ikpy)           │   │
│  │ - Joint Control              │   │
│  │ - Feedback Loop              │   │
│  └──────────┬───────────────────┘   │
└─────────────┼──────────────────────┘
              │ Serial/USB
              │ ROS Services
              ▼
        ┌──────────────┐
        │  Robot ARM   │
        │  Dobot/      │
        │  LeRobot/    │
        │  Custom      │
        └──────────────┘
```

---

## 📦 Installation

### Prerequisites
- Ubuntu 22.04 (ROS 2 Humble) or Ubuntu 24.04 (ROS 2 Jazzy)
- Python 3.10+
- ROS 2 workspace set up

### 1. Clone Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/Phone2Act.git
cd Phone2Act
```

### 2. Install Dependencies
```bash
pip install -r requirements.txt
```

### 3. Build ROS Packages
```bash
cd ~/ros2_ws
colcon build --packages-select phone2act_core phone2act_dobot phone2act_lerobot
source install/setup.bash
```

---

## 🚀 Quick Start Guide

### Step 0: Setup ROS Bridge (All robots)

The ROS bridge enables WebSocket communication between your phone and the ROS 2 network:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

This launches the WebSocket server on `ws://YOUR_LAPTOP_IP:9090`.

### Step 1: Install Phone2Act App

1. Download `phone2act.apk` from the [Releases](link/to/releases) page
2. Install on your Android device (enable unknown sources if needed)
3. Open the app

### Step 2: Connect Your Phone

1. In the Phone2Act app, enter your laptop's IP address (find it using `hostname -I`)
2. Tap **Connect**
3. You should see "Connected ✓" and the AR pose visualization

### Step 3: Hold Your Phone Correctly

```
     📱  ← Phone camera faces the robot
     |
    /|\  ← Hold steady, move naturally
   / | \
```

Position the phone so the camera can see the robot's workspace. Move the phone to control the end-effector position and orientation.

### Step 4: Activate Control

- **ARPose block**: Publishes `/bros2/ar_pose` (end-effector target)
- **Volume buttons**: 
  - Press to toggle **Volume-Up** for gripper OPEN
  - Press to toggle **Volume-Down** for gripper CLOSE

---

## 🛠️ Setup by Robot Type

### Option A: Custom Robot (Recommended for beginners)

Custom robot setup is the most flexible and requires only a URDF and a YAML config file.

#### 1. Prepare URDF File
Create a URDF for your robot (or use an existing one). Place it in:
```
src/phone2act_core/config/my_robot.urdf
```

Ensure your URDF has:
- A `base_link` 
- Joint definitions with bounds
- An end-effector link (typically `tool0` or `tcp`)

#### 2. Create Hardware Bridge
Implement a custom hardware bridge by copying the template:

```bash
cp src/phone2act_core/phone2act_core/template_hardware_bridge.py \
   src/phone2act_core/phone2act_core/my_robot_hardware_bridge.py
```

Edit `my_robot_hardware_bridge.py` to:
- Connect to your robot hardware (serial, ROS services, etc.)
- Read/write joint positions
- Publish feedback (current pose via forward kinematics)

#### 3. Create Configuration File
Copy and edit the template config:

```bash
cp src/phone2act_core/config/template.yaml \
   src/phone2act_core/config/my_robot.yaml
```

**Key parameters to customize:**

```yaml
phone2act_planner:
  ros__parameters:
    # ===== Scaling =====
    scale_pos: 800.0         # Sensitivity: higher = more responsive
    scale_rot: 1.0           # Rotation sensitivity

    # ===== Safety =====
    safe_limit_jump: 100.0   # Max position jump (mm) before reject
    cmd_rate_hz: 30.0        # Command update rate

    # ===== Workspace Limits (mm) =====
    # Define the reachable volume of your robot
    limits.x: [-800.0, 800.0]
    limits.y: [-800.0, 800.0]
    limits.z: [-800.0, 800.0]

    # ===== Position Mapping =====
    # How phone motion maps to robot motion
    # Format: "+x", "-y", "+z" (axis from phone)
    # Available: x, y, z (phone accelerometer axes)
    mapping.position.x: "+x"    # Robot X ← Phone X
    mapping.position.y: "+y"    # Robot Y ← Phone Y
    mapping.position.z: "+z"    # Robot Z ← Phone Z

    # ===== Rotation Mapping =====
    # How phone rotation maps to robot rotation
    # Available: roll, pitch, yaw (phone gyroscope)
    mapping.rotation.rx: "+roll"     # Robot RX ← Phone Roll
    mapping.rotation.ry: "+pitch"    # Robot RY ← Phone Pitch
    mapping.rotation.rz: "+yaw"      # Robot RZ ← Phone Yaw
```

**Understanding the Mapping:**

The `mapping.position` parameters define which phone axis controls which robot axis:
- `+x` → Phone X-axis motion increases robot's X position
- `-y` → Phone Y-axis motion decreases robot's Y position (inverted)
- Phone axes: X (side-to-side), Y (forward-back), Z (up-down)

The `mapping.rotation` parameters define how phone orientation controls robot orientation:
- `+roll` → Phone roll directly controls robot roll
- `-pitch` → Phone pitch (inverted) controls robot pitch
- Phone rotations are measured by the IMU/gyroscope

#### 4. Launch Your Robot

```bash
# Terminal 1: Start hardware bridge
ros2 run your_package my_robot_hardware_bridge

# Terminal 2: Start planner with your config
ros2 launch phone2act_core phone2act_teleop_planner.launch.py config:=src/phone2act_core/config/my_robot.yaml

# Then activate from phone app (see Step 2-4 above)
```

---

### Option B: Dobot CR Series

Dobot robots (CR-3, CR-5, CR-10, etc.) have industrial-grade control systems.

#### 1. Launch Dobot Bringup

```bash
# Terminal 1: Start ROS bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: Start Dobot firmware bridge
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

#### 2. Start Hardware Bridge

```bash
# Terminal 3: Start Dobot hardware bridge
ros2 run dobot_hardware_bridge dobot_hardware_bridge
```

This bridge:
- Connects to the Dobot robot via the ROS services launched above
- Reads/writes joint angles
- Publishes current pose

#### 3. Start Teleop Planner

```bash
# Terminal 4: Start planner with Dobot config
ros2 run phone2act_core phone2act_teleop_planner \
  --ros-args -p config:=src/phone2act_core/config/dobot.yaml
```

#### 4. Use Phone App

- Connect from app (Step 2 above)
- Hold phone in front of robot
- Use volume buttons for gripper control

**Dobot Configuration Breakdown:**

```yaml
scale_pos: 900.0          # Dobot workspace is large
scale_rot: 1.0
safe_limit_jump: 60.0     # Conservative safety threshold
cmd_rate_hz: 45.0         # Dobot can handle 45 Hz

limits.x: [-800.0, 0.0]    # Dobot CR-5 workspace (mm)
limits.y: [-500.0, 500.0]
limits.z: [13.0, 750.0]

# Phone motion → Dobot motion
# Dobot is mounted upright, so we rotate the axes
mapping.position.x: "-z"   # Phone Z → Dobot X (inverted)
mapping.position.y: "-x"   # Phone X → Dobot Y (inverted)
mapping.position.z: "+y"   # Phone Y → Dobot Z

mapping.rotation.rx: "+pitch"  # Intuitive mapping
mapping.rotation.ry: "+yaw"
mapping.rotation.rz: "+roll"
```

---

### Option C: LeRobot SO-100/SO-101

LeRobot arms use Feetech STS servos with lighter weight and lower latency.

#### 1. Prepare Hardware

- Connect all 6 servos (5 joints + gripper) to the USB-serial adapter
- Verify motor IDs match the config: 1-5 for arm, 6 for gripper
- Update `CALIBRATION_PATH` if using a different calibration file

#### 2. Launch Hardware Bridge

```bash
# Terminal 1: Start ROS bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: Start LeRobot hardware bridge
ros2 run phone2act_lerobot lerobot_hardware_bridge
```

This bridge:
- Connects to the motor bus via `/dev/ttyACM0` (configurable)
- Performs real-time IK on the hardware thread
- Reads joint positions at 45 Hz
- Publishes pose feedback

#### 3. Start Teleop Planner

```bash
# Terminal 3: Start planner with LeRobot config
ros2 run phone2act_core phone2act_teleop_planner \
  --ros-args -p config:=src/phone2act_core/config/lerobot.yaml
```

#### 4. Use Phone App

- Connect from app
- The LeRobot is lighter, so use gentler movements
- Volume buttons control gripper (80° open, -80° close)

**LeRobot Configuration:**

```yaml
scale_pos: 800.0           # Smaller workspace than Dobot
scale_rot: 1.0
safe_limit_jump: 40.0      # More conservative (lighter robot)
cmd_rate_hz: 20.0          # LeRobot servo bus is slower

limits.x: [-800.0, 800.0]  # Full workspace
limits.y: [-800.0, 800.0]
limits.z: [-800.0, 800.0]

# LeRobot mounting: standard (camera facing forward)
mapping.position.x: "-x"   # Phone X → LeRobot X (inverted)
mapping.position.y: "+z"   # Phone Z → LeRobot Y
mapping.position.z: "+y"   # Phone Y → LeRobot Z

mapping.rotation.rx: "+pitch"
mapping.rotation.ry: "+yaw"
mapping.rotation.rz: "+roll"
```

**LeRobot Hardware Bridge Parameters:**

Edit `phone2act_lerobot/phone2act_lerobot/lerobot_hardware_bridge.py` to adjust:

```python
ROBOT_NAME = "so100_follower"      # Robot identifier
CONTROL_FREQ = 45.0                # Feedback loop frequency
URDF_PATH = "phone2act_lerobot/urdf/so101_new_calib.urdf"
CALIBRATION_PATH = "phone2act_lerobot/config/my_awesome_follower_arm .json"
GRIPPER_OPEN = 80.0                # Servo position for open
GRIPPER_CLOSE = -80.0              # Servo position for closed
```

For custom USB port:

```bash
ros2 run phone2act_lerobot lerobot_hardware_bridge \
  --ros-args -p usb_port:=/dev/ttyUSB0
```

---

## 📋 Configuration Reference

### `mapping.position.*` Parameters

Controls how phone axes map to robot position:

| Phone Axis | Description | Robot Axes |
|-----------|-------------|-----------|
| `x` | Side-to-side tilt | Any robot axis |
| `y` | Forward-backward tilt | Any robot axis |
| `z` | Up-down motion | Any robot axis |

Example: `mapping.position.x: "-z"` means "robot's X position is controlled by the negative of phone's Z axis"

### `mapping.rotation.*` Parameters

Controls how phone rotation maps to robot rotation:

| Phone Rotation | Description | Robot Rotations |
|--------------|-------------|-----------------|
| `roll` | Rotation around length axis (L-R) | rx, ry, rz |
| `pitch` | Rotation around width axis (F-B) | rx, ry, rz |
| `yaw` | Rotation around vertical axis | rx, ry, rz |

### Safety Parameters

- **`scale_pos`** (default: 800) — Position scale factor. Higher = faster, more responsive
- **`scale_rot`** (default: 1.0) — Rotation scale factor
- **`safe_limit_jump`** (mm) — Max distance to jump before rejecting command (protects against noise)
- **`cmd_rate_hz`** — Command publish frequency (limits CPU usage and latency)

### Workspace Parameters

Define the safe reachable volume for your robot (in mm):

```yaml
limits.x: [-800.0, 800.0]  # X range
limits.y: [-800.0, 800.0]  # Y range
limits.z: [-800.0, 800.0]  # Z range
```

If target goes outside limits, it's clamped to the boundary.

---

## 📱 Phone2Act App Guide

### Installation
1. Download `phone2act.apk` from [Releases](link/to/releases)
2. Enable "Unknown Sources" in Android Settings → Security
3. Install the APK

### First Launch

**Connection Tab:**
1. Enter your laptop's IP (get via `hostname -I` on Linux)
2. Enter port: `9090` (default ROS bridge port)
3. Tap **Connect**
4. Wait for "Connected ✓"

**Control Tab:**
- **AR Pose**: Publishes your phone's pose
- **Volume Up**: Publish gripper OPEN command
- **Volume Down**: Publish gripper CLOSE command
- **Reset**: Reset initial pose (clutch engagement)

### Physical Holding

Hold the phone naturally:
- **Camera facing the robot** (so AR can track)
- **Steady grip** but allow natural wrist rotation
- **Start position**: Arm relaxed at your side

### Tips for Smooth Control

1. **Calibrate**: Press "Reset" when in a comfortable neutral position
2. **Smooth motions**: Avoid jerky movements; move fluidly
3. **Workspace awareness**: Watch the robot to stay within limits
4. **Gripper control**: Toggle volume buttons (don't hold)

---

## 🏗️ Architecture Details

### Planner Node (`phone2act_teleop_planner`)

**Inputs:**
- `/bros2/ar_pose` — Phone pose from app (PoseStamped)
- `/bros2/volume` — Gripper commands (Int32MultiArray)
- `/phone2act/robot_feedback` — Current robot pose (PoseStamped)

**Outputs:**
- `/phone2act/target_pose` — Target end-effector pose for hardware bridge (PoseStamped)
- `/phone2act/gripper_cmd` — Gripper target (Float32)

**Processing:**
1. Receives AR pose in phone frame
2. Applies position/rotation mapping from YAML
3. Scales by sensitivity factors
4. Clamps to workspace limits
5. Checks safety threshold (limit_jump)
6. Rate-limits to cmd_rate_hz
7. Publishes target pose

### Hardware Bridge

Each robot type has a custom bridge that:
1. **Subscribes** to `/phone2act/target_pose`
2. **Solves IK** to get joint angles
3. **Publishes joint commands** to hardware
4. **Reads feedback** (joint positions)
5. **Publishes pose** back via `/phone2act/robot_feedback`

---

## 🔧 Creating a Custom Hardware Bridge

Use the template as a starting point:

```bash
cp src/phone2act_core/phone2act_core/template_hardware_bridge.py my_bridge.py
```

**Key methods to implement:**

```python
class MyRobotBridge(Node):
    def __init__(self):
        # Load URDF, initialize hardware, set up subscriptions
        
    def connect_hardware(self):
        # Connect to robot (serial, network, etc.)
        
    def cb_target(self, msg):
        # Solve IK, send joint commands
        
    def loop_feedback(self):
        # Read current joints, compute FK, publish pose
```

See `dobot_hardware_bridge.py` or `lerobot_hardware_bridge.py` for full examples.

---

## 🐛 Troubleshooting

### App Can't Connect
- Check laptop IP: `hostname -I`
- Ensure WiFi: `ip link show`
- Verify ROS bridge is running: `ros2 topic list | grep bros2`
- Check firewall: `sudo ufw status`

### Robot Doesn't Move
- Check topic publish: `ros2 topic echo /phone2act/target_pose`
- Verify hardware bridge is running
- Check hardware connection (USB, serial port)
- Look for errors: `ros2 node list`

### Jerky/Unsafe Motions
- **Increase `safe_limit_jump`** (mm) to allow bigger jumps
- **Decrease `scale_pos`** to reduce sensitivity
- **Increase `cmd_rate_hz`** for smoother updates

### IK Fails
- Check if pose is reachable: `limits.x`, `limits.y`, `limits.z`
- Verify URDF is correct
- Try different phone orientation (reset pose)

---

## 📄 Citation

If you use Phone2Act in your research, please cite:

```bibtex
@inproceedings{phone2act2026,
  title={Phone2Act: Intuitive Smartphone-Based Teleoperation for Robotic Arms},
  author={Mandhane, Om and ...},
  booktitle={Proceedings of ICRA 2026},
  year={2026}
}
```

---

## 📄 License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

---

## 🤝 Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📞 Support

- **Issues**: [GitHub Issues](link/to/issues)
- **Discussions**: [GitHub Discussions](link/to/discussions)
- **Email**: your.email@example.com

---

**Made with ❤️ for intuitive robot control**
