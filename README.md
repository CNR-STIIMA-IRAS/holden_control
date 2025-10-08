# ROS installation
Open a terminal and type there instrictions:

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

Then: 
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Then:
```bash
sudo apt update
sudo apt upgrade
```

Then:
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

Finally, everytime you open a terminal to use ROS:
```bash 
source /opt/ros/humble/setup.bash
```

# Holden Workspace Setup Instructions

This guide explains how to set up the ROS 2 Humble workspace for the Holden project.

---

## 1. Source ROS 2 Humble
Make sure you have ROS 2 Humble installed and source it:
```bash
source /opt/ros/humble/setup.bash
```

---

## 2. Create the Workspace
From your home directory, create the workspace structure:
```bash
cd ~
mkdir -p projects/holden_ws/src
```

---

## 3. Move into the Source Folder
```bash
cd ~/projects/holden_ws/src
```

---

## 4. Download the Dependencies File
Download the `deps.repos` file:
```bash
wget https://raw.githubusercontent.com/CNR-STIIMA-IRAS/holden_control/refs/heads/test_real_franka/deps.repos
```

---

## 5. Install `vcs` Tool and Import Dependencies
Install the `vcs` tool (if not already installed):
```bash
sudo apt install python3-vcstool -y
```

Import dependencies listed in `deps.repos`:
```bash
vcs import < deps.repos
```

---

## 6. Install ROS Dependencies via rosdep
Make sure `rosdep` is initialized:
```bash
sudo rosdep init   # run only once, skip if already done
rosdep update
```

Install required dependencies:
```bash
rosdep install --from-paths . --ignore-src -r -y
```

---

## 7. Build the Workspace
Go back to the workspace root:
```bash
cd ~/projects/holden_ws
```

Build with `colcon`:
```bash
colcon build --symlink-install --continue-on-error --parallel-workers 4
```

---

## 8. Source the Workspace
After building, source your workspace:
```bash
source install/setup.bash
```
<!-- You may want to add this line to your `~/.bashrc` to make it permanent. -->

## 9. Run the demo
Open a terminal:

```bash
source /opt/ros/humble/setup.bash
```

Then: 

```bash
ros2 launch franka_holden_control franka_holden.launch.py use_fake_hardware:=false robot_ip:=<insert robot ip> 
```

Then, do a little movement of the robot using the MotionPlanning tab and check if the real robot executes it.

If it works, try the gripper.
Firstly, open another terminal:

```bash
source /opt/ros/humble/setup.bash
```
Then, if the gripper is closed, type this command to open it:

```bash
ros2 action send_goal /fr3_gripper/gripper_cmd control_msgs/action/GripperCommand "command:
  position: 0.05
  max_effort: 0.0" 
```

Otherwise, if it is already open, close it:

```bash
ros2 action send_goal /fr3_gripper/gripper_cmd control_msgs/action/GripperCommand "command:
  position: 0.0
  max_effort: 0.0" 
```

Hopefully everything worked as expected!

Thank you!