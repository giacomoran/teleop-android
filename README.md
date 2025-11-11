# Teleop SO101 arm from Android app

I wasn't happy with the [LeRobot phone teleoperation setup](https://huggingface.co/docs/lerobot/en/phone_teleop) for my SO101 robot arm, so I've built my own.

[DEMO VIDEO AND APP EXPLANATION]()

Highlights:

- Intuitive control:
  - phone's xyz position controls the lower arm tip position
  - phone's pitch and roll control the wrist flex and roll
  - robot moves only when holding down on the control pad
  - slide your finger up/down on the control pad to open/close the gripper
  - finer movements while holding the left side of the control pad (with dead zone between the left and right sides)
- ARCore to estimate the phone's position and orientation (works much better than WebXR on my Pixel 7a)
- WebSocket server for communication from the Android app to a Python server
- Integration with LeRobot, both in simulation and hardware

<img src="assets/demo.gif" width="480" alt="Teleop Android app screenshot" />

This project includes:

- `python/`: A Python library and examples to teleoperate a SO101 arm with a phone, either in simulation or on hardware. The Python library runs a server to receive data from the Android app over WebSocket, and implements a few LeRobot processing steps.
- `android/`: An Android app that sends the phone position and orientation (estimated with ARCore) to the Python server.

## Requirements

Before starting, ensure you have:

- **Android device** with ARCore support:
  - Android 7.0+ (API level 24+)
  - Check [ARCore compatibility](https://developers.google.com/ar/devices) for your device
  - Tested on Pixel 7a
- **Python 3.13+** (see `python/pyproject.toml`)
- **Android Studio** (latest stable version recommended)
- **SO-ARM100 repository** cloned locally (for URDF files):
  ```bash
  git clone https://github.com/TheRobotStudio/SO-ARM100.git
  ```
- **Network**: Phone and computer must be on the same Wi-Fi network
- **Optional for simulation**: `rerun` dependency for visualization (see Python setup below)

## Setup

### Step 1: Prepare the URDF file

The Python examples require the [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) repository to be cloned. Instead of using the phone's 6DoF to control the gripper tip, this project uses the phone's xyz position to control the lower arm tip and the phone's pitch/roll to control the wrist flex/roll.

**Add the following frame to your SO101 URDF file** (e.g., `SO-ARM100/Simulation/SO101/so101_new_calib.urdf`):

```xml
<!-- Lower arm frame (dummy link + fixed joint) -->
<link name="lower_arm_frame_link">
  <origin xyz="0 0 0" rpy="0 -0 0"/>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1e-9"/>
    <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
  </inertial>
</link>

<joint name="lower_arm_frame_joint" type="fixed">
  <origin xyz="-0.1349 0.0052 0.015" rpy="1.5708 0 -1.5708"/>
  <parent link="lower_arm_link"/>
  <child link="lower_arm_frame_link"/>
  <axis xyz="0 0 0"/>
</joint>
```

### Step 2: Install Python dependencies

Navigate to the `python/` directory and install the package:

**Using uv:**

```bash
cd python
uv sync --extra rerun  # Include rerun for simulation visualization
```

### Step 3: Update URDF path in examples

Edit the `PATH_URDF` variable in `python/examples/run_simulation.py` (simulation) or `run_so101.py` (hardware) to point to your modified URDF.

**For hardware only**: Also update the robot port in `run_so101.py`:

```python
robot_config = SO100FollowerConfig(
    port="/dev/tty.usbmodem5A460829821",  # Update with your device port
    id="arm_follower_0",
    use_degrees=True
)
```

### Step 4: Build and install the Android app

1. **Open Android Studio** and select "Open an Existing Project"
2. **Navigate to** the `android/` folder in this repository
3. **Connect your Android device** via USB and enable USB debugging
4. **Build and install** the app:
   - Click "Run" (green play button) or press `Shift+F10`
   - Select your connected device
   - The app will install and launch automatically

**First launch**: The app will check ARCore support and request camera permissions. Grant all permissions when prompted.

### Step 5: Run the Python server

Choose one of the following options:

**Option A: Simulation (recommended for first-time setup)**

```bash
cd python
python examples/run_simulation.py
```

This opens a rerun session which:

- Logs phone x, y, z, roll, pitch, yaw over time
- Shows the phone position and orientation in 3D space
- Simulates a SO101 arm

**Option B: Hardware control**

```bash
cd python
python examples/run_so101.py
```

This connects to the actual SO101 hardware and allows real-time control.

**Important**: The server will print connection information like:

```
Starting teleop stream for Android...
Server started at 0.0.0.0:4443
WebSocket endpoint available at wss://192.168.1.100:4443/ws
```

Note the **IP address** (e.g., `192.168.1.100`) and **port** (default: `4443`) from the "WebSocket endpoint" line—you'll need these in the next step.

### Step 6: Connect from the Android app

1. **Ensure your phone and computer are on the same Wi-Fi network**
2. **Open the teleop app** on your Android device
3. **Enter the connection details**:
   - IP address: The IP address shown in the Python server output (e.g., `192.168.1.100`)
   - Port: The port number shown (default: `4443`)
4. **Tap "START"** - the app will establish a WebSocket connection
5. **Start teleoperating**:
   - Hold down on the control pad to enable movement
   - Move your phone to control the robot
   - Slide finger up/down on the control pad to control the gripper
   - Hold the left side of the control pad for fine control mode

## Misc notes

- The `python/certs/` directory contains self-signed SSL certificates for development use only. Do not use these certificates in production.
- To make the robot move more smoothly, I also had to manually adjust the servos P and D gains in the `SO100Follower` class in LeRobot (I've set them respectively to 4 and 8).
