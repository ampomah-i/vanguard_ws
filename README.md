# Autonomous Drone Control and Localization Project - WIP

## Project Overview

This project is designed to implement and test drone control and localization functionalities using ROS 2 and PX4. This includes path following, AR tag detection, and position estimation.

## Directory Structure

```
vanguard_ws/
├── src/
│   ├── control/            # Control package
│   │   ├── control/
│   │   │   ├── __init__.py
│   │   │   ├── path_following.py
│   │   │   ├── hover.py
│   │   │   ├── arm_disarm.py
│   │   │   ├── translation_control.py
│   │   ├── setup.py
│   ├── localization/       # Localization package
│   │   ├── localization/
│   │   │   ├── __init__.py
│   │   │   ├── ar_tag_detection.py
│   │   │   ├── position_estimation.py
│   │   │   ├── config/
│   │   │   │   ├── ar_tag_positions.yaml
│   │   ├── setup.py
│   ├── px4_msgs/           # PX4 messages package
│   │   ├── px4_msgs/
│   │   ├── setup.py
```

## Dependencies

- ROS 2 (Humble)
- PX4 Autopilot
- Python 3.8+
- Required ROS 2 packages: `px4_msgs`, `sensor_msgs`, `geometry_msgs`
- OpenCV
- YAML
- ROS2 V4L2 Camera Node
- DepthAI ROS Driver

## Installation Instructions

1. **Install ROS 2 and PX4**:
   - Follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).
   - Install PX4 following the [PX4 Autopilot installation guide](https://docs.px4.io/main/en/dev_setup/building_px4.html).

2. **Clone the Repository**:
   ```sh
   git clone <repository_url> vanguard_ws
   cd vanguard_ws
   ```

3. **Install Dependencies**:
   ```sh
   sudo apt update
   sudo apt install python3-opencv python3-yaml
   ```

4. **Build the Workspace**:
   ```sh
   colcon build
   ```

5. **Source the Workspace**:
   ```sh
   source install/setup.bash
   ```

## Usage

### Path Following

1. **Configure Path**:
   - Edit the `path.yaml` file in `src/control/control/` to define paths to follow.

2. **Run Path Following Node**:
   ```sh
   ros2 run control path_following
   ```

### AR Tag Detection
0. **Define AR Tag Locations for Localization**:
   - Edit the `ar_tag_positions.yaml` file in `src/localization/localization/config/` to define AR tag positions.

1. **Run AR Tag Detection Node**:
   ```sh
   ros2 run localization ar_tag_detection
   ```

### Position Estimation

1. **Run Position Estimation Node**:
   ```sh
   ros2 run localization position_estimation
   ```

## Scripts Description

### Control Package

- `path_following.py`: 
  - Implements the path following functionality using trajectory setpoints.
  - Reads path waypoints from a YAML file and follows the defined path.
 
- `arm_disarm.py`:
  - Implements arming, disarming, takeoff and landing of the drone.
  - Publishes appropriate commands to arm and disarm the drone.

- `trajectory_control.py`:
  - Implements trajectory control functionalities.
  - Allows setting custom trajectories for the drone to follow.

- `hover.py`:
  - Implements hover functionality.
  - Maintains the drone's position at a specified point.

### Localization Package

- `ar_tag_detection.py`: 
  - Detects AR tags in images and publishes the relative position of the drone to the detected tags.
  - Uses OpenCV for AR tag detection.

- `position_estimation.py`: 
  - Estimates the global position of the drone based on AR tag detections and odometry data.
  - Uses AR tag positions defined in a YAML file for accurate localization.

### Configuration Files

- `ar_tag_positions.yaml`:
  - Contains the positions of AR tags used for localization.
  - Define each tag's position in the global frame.

## Authors

- Immanuel
- Contributors: [List of contributors]

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
