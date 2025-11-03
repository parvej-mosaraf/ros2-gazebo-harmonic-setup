# Gazebo Sim (Harmonic) Installation Guide — Ubuntu 22.04

This guide walks you through installing the **latest Gazebo Sim (Harmonic)** on **Ubuntu 22.04**, along with optional **ROS 2 Humble integration**.  
Gazebo Sim (also called `gz sim`) is the modern simulation framework replacing the older **Gazebo Classic**.

---

## 🧩 1️⃣ Add Gazebo Sim Repository
The OSRF (Open Source Robotics Foundation) repository provides official Gazebo Sim packages.

```bash
sudo apt update
sudo apt install -y curl lsb-release gnupg
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

---

## 🚀 2️⃣ Install Gazebo Sim (Harmonic)
After adding the repository, install the Gazebo Harmonic release.

```bash
sudo apt update
sudo apt install -y gz-harmonic
```

---

## ✅ 3️⃣ Verify Installation
Check that Gazebo Sim was installed correctly and runs as expected.

```bash
gz sim
gz --version
```

---

## 🌍 4️⃣ Run a Sample World
You can test Gazebo with an empty world or load your own simulation.

```bash
gz sim -v 4 empty.sdf
# or
gz sim ~/ros2_ws/src/mybot_gz/worlds/empty.world.sdf
```

---

## 🔗 5️⃣ (Optional) Integrate with ROS 2 Humble
Install the ROS–Gazebo bridge to connect Gazebo topics with ROS 2 nodes.

```bash
sudo apt install -y ros-humble-ros-gz
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```

---

## 🧹 6️⃣ (Optional) Uninstall Gazebo Sim
If you ever need to remove Gazebo Sim completely, use these commands.

```bash
sudo apt remove --purge -y gz-harmonic*
sudo apt autoremove -y
sudo rm -f /usr/share/keyrings/gazebo-archive-keyring.gpg
sudo rm -f /etc/apt/sources.list.d/gazebo-stable.list
```

---

## 💡 Notes
- Always use the `gz` command, not `gazebo`, since the latter refers to Gazebo Classic.  
- Gazebo Harmonic integrates smoothly with **ROS 2 Humble** and later versions.  
- You can create custom `.sdf` or `.world` files inside your `~/ros2_ws/src/<your_robot>/worlds/` folder.  

---

## 📚 References
- Gazebo Sim Official Docs → https://gazebosim.org/docs/harmonic  
- ROS–Gazebo Integration → https://gazebosim.org/docs/harmonic/ros_integration  
- OSRF Package Repository → https://packages.osrfoundation.org/gazebo/
