# The Pelican

TurtleBot3 autonomous bottle-tracking robot with a motorized lid. The robot wanders, detects target objects (bottles, cups, and wine glasses) using YOLOv8, tracks and approaches them, then picks them up with a Dynamixel-powered motors

## Hardware

- **Robot:** TurtleBot3 Burger
- **Camera:** USB camera (published on `/image_raw`)
- **LiDAR:** LDS-01
- **Lid Motor:** Dynamixel Motor ID 3 (GIX\_Waffle firmware on OpenCR)
- **Compute:** Raspberry Pi (onboard) + Desktop PC (YOLO inference)

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

## Launch Sequence

### 1. TurtleBot3 (SSH into Pi)

```bash
export TURTLEBOT3_MODEL=Waffle
export LDS_MODEL=LDS-01
ros2 launch ~/full_bringup.launch.py
```

> This launches the GIX hardware bringup (NOT standard `turtlebot3_bringup`) which includes Motor 3 (lid) support.

### 2. Desktop — YOLO Detection (Terminal 1)

```bash
ros2 launch yolo_bringup yolo.launch.py device:=cpu input_image_topic:=/image_raw model:=yolov8n.pt
```

### 3. Desktop — Bottle Tracker Node (Terminal 2)

```bash
ros2 run final bottle
```

### 4. Desktop — Debug View (Terminal 3, optional)

```bash
ros2 run rqt_image_view rqt_image_view
# Select topic: /yolo/dbg_image
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

## Behavior State Machine

```
WANDER → (object detected) → TRACK → (aligned & close) → AIM
  ↑                            |
  ← (lost target, timeout) ←──┘

WANDER → (obstacle < 0.5m) → BACKUP → TURN → WANDER
```

### Lid Sequence

1. Initialize to 45° (closed)
2. Open to 145°
3. Hold for 3 seconds
4. Close to 45° (grab)
5. Wait 5 seconds before resuming

## Detected Object Classes

- Bottle
- Wine glass
- Cup

## Usage

### Demo Walkthrough

1. **Power on the TurtleBot3** and wait for the Pi to boot (~30 seconds).

2. **SSH into the Pi** from your desktop:
   ```bash
   ssh ubuntu@10.155.234.127
   ```

3. **Set environment variables and launch** on the Pi:
   ```bash
   export ROS_DOMAIN_ID=23
   export TURTLEBOT3_MODEL=Waffle
   export LDS_MODEL=LDS-01
   ros2 launch ~/full_bringup.launch.py
   ```
   Wait until you see the LiDAR spinning and no errors in the terminal.

4. **Open three terminals on your desktop.** In each, source your workspace:
   ```bash
   export ROS_DOMAIN_ID=23
   source /opt/ros/humble/setup.bash
   source ~/turtlebot4_ws/install/setup.bash
   ```

5. **Terminal 1 — Start YOLO:**
   ```bash
   ros2 launch yolo_bringup yolo.launch.py device:=cpu input_image_topic:=/image_raw model:=yolov8n.pt
   ```

6. **Terminal 2 — Start the bottle tracker:**
   ```bash
   ros2 run final bottle
   ```
   The robot will begin wandering and avoiding obstacles automatically.

7. **Terminal 3 (optional) — Open the debug camera view:**
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```
   Select `/yolo/dbg_image` to see bounding boxes overlaid on the camera feed.

8. **Place a bottle, cup, or wine glass** in the robot's path. The robot will detect it, turn to face it, approach, and trigger the lid sequence.

9. **To stop**, press `Ctrl+C` in Terminal 2 first (stops motion), then shut down the other terminals.

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

## Team

Built at the University of Washington Global Innovation Exchange (GIX) — TECHIN 516 Robotics.
