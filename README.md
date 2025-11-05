# Gazebo Sim (Harmonic) + ROS 2 Humble — Installation & Demos

This guide walks you through installing Gazebo Sim (Harmonic) on Ubuntu 22.04, optionally integrating it with ROS 2 Humble, and running several simulation demos.

Gazebo Sim (also called `gz sim`) is the modern simulation framework replacing the older Gazebo Classic.

---

## 1️⃣ Add Gazebo Sim Repository

```bash
sudo apt update
sudo apt install -y curl lsb-release gnupg
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

---

## 2️⃣ Install Gazebo Sim (Harmonic)

```bash
sudo apt update
sudo apt install -y gz-harmonic
```

---

## 3️⃣ Verify Installation

```bash
gz sim
gz --version
```

---

## 4️⃣ Run a Sample World

```bash
gz sim -v 4 empty.sdf
# or load a custom world
gz sim ~/ros2_ws/src/mybot_gz/worlds/empty.world.sdf
```

---

## 5️⃣ (Optional) Integrate with ROS 2 Humble

```bash
sudo apt install -y ros-humble-ros-gz
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```

---

## 6️⃣ (Optional) Uninstall Gazebo Sim

```bash
sudo apt remove --purge -y gz-harmonic*
sudo apt autoremove -y
sudo rm -f /usr/share/keyrings/gazebo-archive-keyring.gpg
sudo rm -f /etc/apt/sources.list.d/gazebo-stable.list
```

---

## 7️⃣ ROS + Gazebo Sim Demos

### Run Gazebo Sim

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"
```

### Air Pressure Sensor

```bash
ros2 launch ros_gz_sim_demos air_pressure.launch.py
ros2 topic echo /air_pressure --qos-reliability best_effort
ros2 topic echo /air_pressure --qos-reliability reliable
```

### Camera

```bash
ros2 launch ros_gz_sim_demos image_bridge.launch.py
ros2 launch ros_gz_sim_demos camera.launch.py
ros2 launch ros_gz_sim_demos triggered_camera.launch.py
ros2 topic pub /camera/trigger std_msgs/msg/Bool "{data: true}" --once
```

### Differential Drive Vehicle

```bash
ros2 launch ros_gz_sim_demos diff_drive.launch.py
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.5}}"
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.0}}" --qos-reliability reliable
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.0}}" --qos-reliability best_effort
```

### Depth Camera

```bash
ros2 launch ros_gz_sim_demos image_bridge.launch.py image_topic:=/depth_camera
ros2 launch ros_gz_sim_demos depth_camera.launch.py
```

### GPU Lidar

```bash
ros2 launch ros_gz_sim_demos gpu_lidar_bridge.launch.py
ros2 launch ros_gz_sim_demos gpu_lidar.launch.py
```

### IMU

```bash
ros2 launch ros_gz_sim_demos imu.launch.py
```

### Magnetometer

```bash
ros2 launch ros_gz_sim_demos magnetometer.launch.py
```

### GNSS (Fortress only)

```bash
ros2 launch ros_gz_sim_demos navsat.launch.py
```

### RGBD Camera

```bash
ros2 launch ros_gz_sim_demos image_bridge.launch.py image_topic:=/rgbd_camera/image
ros2 launch ros_gz_sim_demos image_bridge.launch.py image_topic:=/rgbd_camera/depth_image
ros2 launch ros_gz_sim_demos rgbd_camera_bridge.launch.py
ros2 launch ros_gz_sim_demos rgbd_camera.launch.py
```

### Battery

```bash
ros2 launch ros_gz_sim_demos battery.launch.py
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.5}}"
```

### Robot Description Publisher

```bash
ros2 launch ros_gz_sim_demos robot_description_publisher.launch.py
```

### Joint States Publisher

```bash
ros2 launch ros_gz_sim_demos joint_states.launch.py
```

### TF Bridge (Joint states ↔ TF)

```bash
ros2 launch ros_gz_sim_demos tf_bridge.launch.py
```

---

## Notes

* Always use `gz`, not `gazebo`, for Gazebo Sim commands.
* Gazebo Harmonic integrates smoothly with ROS 2 Humble.
* Custom `.sdf` or `.world` files can be placed in `~/ros2_ws/src/<your_robot>/worlds/`.
* Some demos may be blocked by current `ros_gz_point_cloud` issues.

---

## References

* Gazebo Sim Docs — Harmonic: [https://gazebosim.org/docs/harmonic](https://gazebosim.org/docs/harmonic)
* ROS–Gazebo Integration: [https://gazebosim.org/docs/harmonic/ros_integration](https://gazebosim.org/docs/harmonic/ros_integration)
* OSRF Package Repository: [https://packages.osrfoundation.org/gazebo/](https://packages.osrfoundation.org/gazebo/)
