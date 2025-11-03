# Gazebo Sim (Harmonic) Installation Guide — Ubuntu 22.04

This guide provides all commands and steps to install the **modern Gazebo Sim (Harmonic)** on **Ubuntu 22.04**, along with optional **ROS 2 Humble integration**.  
Gazebo Sim (also known as `gz-sim`) is the next-generation simulation framework developed by **Open Robotics**, replacing the old **Gazebo Classic**.

---

## 🧩 Installation Commands

```bash
# 1️⃣ Add Gazebo Sim Repository
# The official OSRF repository hosts the latest stable Gazebo Sim (Harmonic) packages.
sudo apt update
sudo apt install -y curl lsb-release gnupg
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 2️⃣ Install Gazebo Sim (Harmonic)
# This installs the latest version of Gazebo Sim along with dependencies.
sudo apt update
sudo apt install -y gz-harmonic

# 3️⃣ Verify Installation
# Check if Gazebo Sim runs correctly and verify its version.
gz sim
gz --version

# 4️⃣ Run a Sample World
# You can run a simple empty world or your own world file.
gz sim -v 4 empty.sdf
# or
gz sim ~/ros2_ws/src/mybot_gz/worlds/empty.world.sdf

# 5️⃣ (Optional) Integrate with ROS 2 Humble
# This bridge connects Gazebo topics with ROS 2 nodes.
sudo apt install -y ros-humble-ros-gz
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock

# 6️⃣ (Optional) Uninstall Gazebo Sim
# Run these commands if you want to remove Gazebo Sim completely.
sudo apt remove --purge -y gz-harmonic*
sudo apt autoremove -y
sudo rm -f /usr/share/keyrings/gazebo-archive-keyring.gpg
sudo rm -f /etc/apt/sources.list.d/gazebo-stable.list
```

---

## 💡 Notes
- The command `gz sim` launches Gazebo Sim’s graphical interface.
- Gazebo Sim (Harmonic) can coexist with ROS 2 Humble using the `ros-gz` bridge.
- Always use `gz` instead of `gazebo` — the latter refers to the deprecated Gazebo Classic.

---

## 📚 Reference
- Gazebo Sim Official Docs → https://gazebosim.org/docs/harmonic  
- ROS–Gazebo Integration → https://gazebosim.org/docs/harmonic/ros_integration  
- OSRF Package Repository → https://packages.osrfoundation.org/gazebo/
