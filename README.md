# The Pelican

The Pelican is an autonomous retrieval system designed to scout its environment for misplaced waste. Built on a TurtleBot3 frame and equipped with a custom motorized lid, it spends its time wandering through a space "hunting" for disposal opportunities. Using a YOLOv8 vision model, the robot identifies specific targets like bottles, cups, and wine glasses, at which point it locks on, tracks the object to get within range, and uses high-torque Dynamixel-powered motors to actuate its lid and scoop the item up.

The Pelican is an autonomous retrieval system designed to scout its environment for misplaced waste. Built on a TurtleBot3 frame and equipped with a custom motorized lid, it spends its time wandering through a space "hunting" for disposal opportunities. Using a YOLOv8 vision model, the robot identifies specific targets like bottles, cups, and wine glasses, at which point it locks on, tracks the object to get within range, and uses high-torque Dynamixel-powered motors to actuate its lid and scoop the item up.
<img width="1150" height="762" alt="Screenshot 2026-03-19 at 10 47 08 PM" src="https://github.com/user-attachments/assets/5b74c77f-595d-4575-9f32-78cea165be80" />


## Demo Link
https://drive.google.com/file/d/1fELpsYOHe1enObhneh7-VH1AN5VL8o85/view?usp=sharing

## Feature Checklist

| Requirement | Status | Details |
|---|---|---|
| **URDF robot description** | ✅ | Custom URDF extends TurtleBot3 Waffle with lid motor joint (`gix`). Published via `robot_state_publisher`. See [URDF Description](#urdf-robot-description). |
| **Application code** | ✅ | `bottle.py` — autonomous detection, tracking, and lid actuation node. See [Application Code](#application-code-bottlepy). |
| **Launch file** | ✅ | `full_bringup.launch.py` — single launch file brings up hardware drivers, URDF, joint states, and LiDAR. See [Launch Files](#launch-files). |
| **LiDAR with sufficient FOV for mapping** | ✅ | LDS-01 LiDAR provides **360° FOV** at 5 Hz, publishing on `/scan`. Used for obstacle avoidance and compatible with SLAM Toolbox for full map generation. See [LiDAR & Mapping](#lidar--mapping). |
| **Single-command demo** | ✅ | Robot-side: `ros2 launch ~/full_bringup.launch.py`. Desktop-side: `ros2 launch yolo_bringup yolo.launch.py` + `ros2 run final bottle`. See [Quick Start Demo](#quick-start-demo-single-command). |
| **Teleoperation support** | ✅ | Standard `turtlebot3_teleop` keyboard teleop works alongside or instead of autonomous mode. See [Teleoperation](#teleoperation). |
| **Robot body visible in RViz** | ✅ | Full robot mesh and TF tree visualized in RViz2 via URDF + `robot_state_publisher`. Includes base, wheels, LiDAR, camera, and lid joint. See [RViz Visualization](#rviz-visualization). |

---

## URDF Robot Description

The Pelican uses a **custom URDF** that extends the standard TurtleBot3 Waffle model with an additional revolute joint for the lid motor (Dynamixel Motor ID 3). The URDF defines the following links and joints:

- `base_footprint` → `base_link` (fixed) — robot chassis
- `base_link` → `wheel_left_link` / `wheel_right_link` (continuous) — drive wheels
- `base_link` → `caster_back_link` (fixed) — rear caster
- `base_link` → `base_scan` (fixed) — LDS-01 LiDAR mount
- `base_link` → `camera_link` (fixed) — USB camera mount
- `base_link` → `gix_link` (revolute) — **lid motor joint**, controlled via `/gix_controller/joint_trajectory`

The URDF is loaded by `robot_state_publisher` at launch, which publishes the full TF tree and makes the robot body available for RViz visualization.

```bash
# View the URDF TF tree
ros2 run tf2_tools view_frames
```

## Application Code (bottle.py)

`bottle.py` is the main application node. It implements a full autonomous behavior pipeline:

**Subscriptions:**
- `/yolo/detections` (DetectionArray) — YOLOv8 bounding box detections
- `/scan` (LaserScan) — 360° LiDAR data for obstacle avoidance

**Publishers:**
- `/cmd_vel` (Twist) — velocity commands to the robot base
- `/gix_controller/joint_trajectory` (JointTrajectory) — lid motor position commands

**Behavior state machine:**

```
WANDER → (object detected) → TRACK → (aligned & close) → AIM
  ↑                            |
  ← (lost target, timeout) ←──┘

WANDER → (obstacle < 0.5m) → BACKUP → TURN → WANDER
```

The node uses a **PID controller** (proportional, integral, derivative) for both angular and linear tracking to smoothly align with and approach detected objects. Once aligned, it triggers the lid motor sequence: open to 145°, hold for 3 seconds, close to 45°.

**Detected object classes:** bottle, wine glass, cup.

## Launch Files

### `full_bringup.launch.py` (Robot-side, runs on Pi)

A single launch file that brings up all hardware and description nodes:

- **OpenCR driver** — motor control for wheels + Dynamixel Motor 3 (lid)
- **robot\_state\_publisher** — loads the custom URDF and publishes the TF tree
- **joint\_state\_publisher** — publishes joint states for RViz visualization
- **LDS-01 LiDAR driver** — starts the 360° laser scanner, publishes `/scan`
- **USB camera driver** — publishes raw images on `/image_raw`

```bash
# Single command to launch everything on the robot
export TURTLEBOT3_MODEL=Waffle
export LDS_MODEL=LDS-01
ros2 launch ~/full_bringup.launch.py
```

### Desktop launch (YOLO + tracker)

```bash
# Terminal 1: YOLOv8 detection
ros2 launch yolo_bringup yolo.launch.py device:=cpu input_image_topic:=/image_raw model:=yolov8n.pt

# Terminal 2: Autonomous bottle tracker
ros2 run final bottle
```

## LiDAR & Mapping

The LDS-01 LiDAR provides **360-degree field of view** at approximately 5 Hz with a range of 0.12–3.5 m. This full-surround FOV is sufficient for mapping and SLAM.

**In the application**, LiDAR data on `/scan` is used for real-time obstacle avoidance. The `_sector()` method in `bottle.py` reads a configurable angular sector of the scan to detect obstacles in the robot's forward path (default: 120° front arc, threshold 0.5 m).

**For mapping**, the LiDAR is compatible with SLAM Toolbox and Nav2. To generate a map:

```bash
# Launch SLAM Toolbox (on desktop, while robot is running)
ros2 launch slam_toolbox online_async_launch.py

# Drive around to build the map (teleop or autonomous wander)
# Save the map when done
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Quick Start Demo (Single Command)

### On the robot (SSH into Pi):

```bash
export ROS_DOMAIN_ID=23 && export TURTLEBOT3_MODEL=Waffle && export LDS_MODEL=LDS-01 && ros2 launch ~/full_bringup.launch.py
```

This single command launches all hardware drivers, the URDF description, LiDAR, and camera.

### On the desktop:

```bash
# Terminal 1: Source and start YOLO
export ROS_DOMAIN_ID=23 && source /opt/ros/humble/setup.bash && source ~/turtlebot4_ws/install/setup.bash && ros2 launch yolo_bringup yolo.launch.py device:=cpu input_image_topic:=/image_raw model:=yolov8n.pt

# Terminal 2: Start the autonomous tracker
export ROS_DOMAIN_ID=23 && source /opt/ros/humble/setup.bash && source ~/turtlebot4_ws/install/setup.bash && ros2 run final bottle
```

Place a bottle, cup, or wine glass in front of the robot. It will detect, track, approach, and activate the lid.

## Teleoperation

The robot supports **keyboard teleoperation** using the standard TurtleBot3 teleop package. This can be used for manual control, testing, or mapping.

```bash
# Launch teleop (on desktop)
export TURTLEBOT3_MODEL=Waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

**Controls:**
- `w` / `x` — increase / decrease linear velocity
- `a` / `d` — increase / decrease angular velocity
- `s` — stop all motion
- `Ctrl+C` — quit

> **Note:** Do not run teleop and `bottle.py` at the same time — both publish to `/cmd_vel`. Stop the autonomous node first before switching to teleop.

## RViz Visualization

The **robot body is fully visible in RViz2** via the URDF published by `robot_state_publisher`. The TF tree and robot meshes render the complete robot model including base, wheels, LiDAR, camera, and lid joint.

```bash
# Launch RViz with the robot model (on desktop)
export TURTLEBOT3_MODEL=Waffle
ros2 launch turtlebot3_bringup rviz2.launch.py
```

**What you'll see in RViz:**
- **RobotModel** — full 3D mesh of the TurtleBot3 body, wheels, and lid mechanism
- **TF tree** — all coordinate frames (`base_footprint`, `base_link`, `base_scan`, `camera_link`, `gix_link`, wheel links)
- **LaserScan** — real-time 360° LiDAR point cloud overlay on topic `/scan`
- **Camera image** — raw feed on `/image_raw` (add Image display, select topic)

To manually configure RViz2:
1. Open `rviz2`
2. Set Fixed Frame to `base_footprint` or `odom`
3. Add displays: RobotModel, TF, LaserScan (`/scan`), Image (`/image_raw`)

---

## Hardware

- **Robot:** TurtleBot3 Burger (running Waffle firmware for Motor 3 support)
- **Camera:** USB camera (published on `/image_raw`)
- **LiDAR:** LDS-01 — 360° FOV, 5 Hz, 0.12–3.5 m range
- **Lid Motor:** Dynamixel Motor ID 3 (GIX\_Waffle firmware on OpenCR)
- **Compute:** Raspberry Pi 3B+ (onboard) + Desktop PC (YOLO inference)

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_gix_bringup`)
- [YOLO ROS](https://github.com/mgonzs13/yolov8_ros) (`yolo_bringup` package)
- YOLOv8 model: `yolov8n.pt`

## Workspace Setup

```bash
# Clone into your ROS 2 workspace
cd ~/turtlebot4_ws/src
git clone https://github.com/JzZ404/The_Pelican.git final

# Build
cd ~/turtlebot4_ws
colcon build --packages-select final
source install/setup.bash
```

## Network Configuration

| Device | IP | ROS\_DOMAIN\_ID |
|---|---|---|
| TurtleBot3 (Pi) | 10.155.234.127 | 23 |
| Desktop PC | (your IP) | 23 |

Set on both machines:

```bash
export ROS_DOMAIN_ID=23
```

## Parameters (bottle.py)

| Parameter | Value | Description |
|---|---|---|
| `DEADBAND_ANG` | 20 px | Angular deadband for centering |
| `DEADBAND_LIN` | 20 px | Linear deadband for approach |
| `MAX_ANG_VEL` | 0.8 rad/s | Max turning speed |
| `MAX_LIN_VEL` | 0.25 m/s | Max forward speed |
| `KP_ANG` | 0.012 | Angular proportional gain |
| `KP_LIN` | 0.01 | Linear proportional gain |
| `OBSTACLE_DIST` | 0.5 m | LiDAR obstacle threshold |
| `WANDER_LIN_VEL` | 0.12 m/s | Wander forward speed |
| `NO_DETECT_TIMEOUT` | 7 s | Return to wander after losing target |

## Usage

### Tuning Parameters

All tunable constants are at the top of `bottle.py`. Edit them directly and rebuild:

```bash
cd ~/turtlebot4_ws
colcon build --packages-select final
source install/setup.bash
```

**Speed & responsiveness:**
- Increase `MAX_LIN_VEL` (default 0.25) for faster approach — keep below 0.35 to avoid overshooting.
- Increase `MAX_ANG_VEL` (default 0.8) for snappier turning.
- Increase `WANDER_LIN_VEL` (default 0.12) to cover more ground while searching.

**Tracking precision:**
- Lower `DEADBAND_ANG` / `DEADBAND_LIN` (default 20 px each) for tighter alignment before approach — too low may cause oscillation.
- Increase `KP_ANG` (default 0.012) or `KP_LIN` (default 0.01) for more aggressive corrections. Reduce if the robot jitters.

**Safety & timing:**
- `OBSTACLE_DIST` (default 0.5 m) — increase for more cautious obstacle avoidance, decrease for tighter spaces.
- `NO_DETECT_TIMEOUT` (default 7 s) — how long the robot continues searching after losing sight of a target before returning to wander.

**Lid motor:**
- Open/close angles (default 45°–145°) and hold duration (default 3 s) are set in the lid sequence section of `bottle.py`.

### Adding New Object Classes

The robot tracks any COCO class detected by YOLOv8. To add or remove target classes:

1. Open `bottle.py` and find the list of detected class names (e.g. `bottle`, `wine glass`, `cup`).

2. Add or remove class names. These must match [COCO class names](https://docs.ultralytics.com/datasets/detect/coco/) exactly — for example:
   - `"cell phone"` — track phones
   - `"backpack"` — track backpacks
   - `"banana"` — track bananas

3. Rebuild and relaunch:
   ```bash
   cd ~/turtlebot4_ws
   colcon build --packages-select final
   source install/setup.bash
   ros2 run final bottle
   ```

No retraining is needed — YOLOv8n already recognizes all 80 COCO classes. You're just filtering which ones the tracker responds to.

## Troubleshooting

- **`ros2` CLI not found after activating venv:** Source ROS first, then workspace, then optionally activate venv. Never activate venv before sourcing ROS.
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/turtlebot4_ws/install/setup.bash
  # Only then, if needed:
  source ~/venv/bin/activate
  ```
- **Motor 3 not responding:** Ensure OpenCR is flashed with GIX\_Waffle firmware (`~/TECHIN516/project/t516_OpenCR/`). The claw joint name is `gix`, controlled via `/gix_controller/joint_trajectory`.
- **No YOLO detections:** Verify camera is publishing on `/image_raw`. Check with `ros2 topic echo /image_raw --no-arr`.
- **Domain ID mismatch:** Confirm `ROS_DOMAIN_ID=23` is set on both Pi and desktop.
- **Robot not visible in RViz:** Make sure `robot_state_publisher` is running (it launches automatically via `full_bringup.launch.py`). Set Fixed Frame to `base_footprint`. Add a RobotModel display.

## Team

Built at the University of Washington Global Innovation Exchange (GIX) — TECHIN 516 Robotics.
