# RoboRacer wall-following challenge

A ROS 2 Python node that autonomously follows the left wall in the AutoDRIVE IROS 2026 practice simulator.

## Results

- Fastest verified clean lap: **11.9742 seconds**, achieved in an earlier clean validation run with the same submitted controller settings.
- Submission video: **[Watch three consecutive clean laps](https://youtu.be/bJJ0KVZFIds)**.
- The submission recording shows three consecutive complete flying laps with zero collisions: **12.3093 s, 12.1873 s, 12.2903 s**. These follow a 13.8150 s standing-start lap.
- Settings: `LOOKAHEAD = 0.8`, `BASE_THROTTLE = 0.18`.
- Lap times are simulator-reported times, not wall-clock video duration. Rendering speed can make the two differ.
- Tuning and validation details: [TUNING.md](TUNING.md).

## Controller

`autodrive_devkit/autodrive_roboracer/wall_follow_node.py` subscribes to LiDAR scans and publishes normalized throttle and steering commands. It estimates the left-wall angle from median readings near +90 and +45 degrees, projects clearance ahead, and uses PD steering. Front clearance and steering demand reduce throttle. A scan timeout and invalid/too-close front checks request zero commands.

The controller is reactive: it uses no map or prerecorded path. The provided bridge is unchanged. AI assisted implementation, explanation and testing.

## Run (Windows PowerShell)

Install Docker Desktop and Git. Start Docker Desktop. Download and extract `autodrive_simulator_practice_windows.zip` from the [IROS 2026 release](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros).

From the repository root:

```powershell
docker build --platform linux/amd64 -t penn-challenge -f autodrive_devkit.Dockerfile .
docker run --platform linux/amd64 --name penn-challenge --rm -it -p 4567:4567 -v "${PWD}/autodrive_devkit:/home/autodrive_devkit/src/autodrive_devkit" --entrypoint /bin/bash penn-challenge
```

Inside the container:

```bash
cd /home/autodrive_devkit
colcon build
source install/setup.bash
ros2 launch autodrive_roboracer bringup_headless.launch.py
```

In another PowerShell terminal:

```powershell
docker exec -it penn-challenge bash
```

Then inside that terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 run autodrive_roboracer wall_follow_node
```

Open the simulator, connect to `127.0.0.1:4567`, and select Autonomous. Do not run keyboard teleoperation at the same time. For recording, use Driver's Eye view. On this Intel Iris Xe machine, the simulator needs `-force-d3d11`; its default Direct3D 12 mode crashed during startup.

After editing the node, stop that node, run `colcon build`, source `install/setup.bash`, and restart it. Switch out of Autonomous before stopping the controller.

## Topics

| Direction | Topic | Message |
|---|---|---|
| Subscribe | `/autodrive/roboracer_1/lidar` | `sensor_msgs/LaserScan` |
| Publish | `/autodrive/roboracer_1/throttle_command` | `std_msgs/Float32` |
| Publish | `/autodrive/roboracer_1/steering_command` | `std_msgs/Float32` |

## Acknowledgments

Built on the [AutoDRIVE RoboRacer Sim Racing starter](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing). Geometry and PD control were informed by RoboRacer/F1TENTH wall-following teaching materials. Existing third-party copyright and license notices are retained.

