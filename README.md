# ROS Humble Multi-Platform Docker Setup

This guide details how to run a multi-container ROS 2 setup using Docker on Linux, macOS, and WSL2. The containers use a virtual display (via Xvfb) and expose VNC/noVNC for GUI access.

> **Note:**  
> Ensure Docker (or Docker Desktop) is installed and configured for your platform. If you’re using WSL2 on Windows, ensure Docker Desktop is set up with WSL2 integration.

---

## Table of Contents

- [Create Docker Network](#create-docker-network)
- [Running the Containers](#running-the-containers)
  - [Terminal 1: ROS Bag Playback](#terminal-1-ros-bag-playback)
  - [Terminal 2: Launch RViz](#terminal-2-launch-rviz)
  - [Terminal 3: Lidar Calibration Node](#terminal-3-lidar-calibration-node)
  - [Terminal 4: Run Odometry Node (KISS-ICP)](#terminal-4-run-odometry-node-kiss-icp)
  - [Terminal 5: Run off the shelf calibration Node with NDT/ICP(Optional) ](#terminal-5-run-calibration-node-ndt-icp)
- [Accessing the VNC/noVNC Interface](#accessing-the-vncnovnc-interface)
- [WSL2 Specific Instructions](#wsl2-specific-instructions)
- [Troubleshooting](#troubleshooting)

---

## Pull the project docker image

* If you're on an AMD architecture:
```bash
docker pull thinkautonomous/ros2_humble_gto_amd:latest
```
* If you're on an ARM architecture:
```bash
docker pull thinkautonomous/ros2_humble_gto_arm:latest
```

## Create Docker Network

Before launching any containers, create the Docker network `rosnet` so that your containers can communicate:

```bash
docker network create rosnet
```

---


## Running the Containers

The following instructions apply to Linux and macOS. The same commands will work in WSL2 (see [WSL2 Specific Instructions](#wsl2-specific-instructions) below).

### Terminal 1: ROS Bag Playback

1. **Start a Container for ROS Bag Playback:**

   ## WSL and Linux users
   ```bash
   docker run -it --network rosnet -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_amd:latest
   ```
   ## MacOS users
   ```bash
   docker run -it --network rosnet -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_arm:latest
   ```

2. **Inside the Container, Play the ROS Bag:**

   ```bash
   gdown --folder "https://drive.google.com/drive/folders/12pR6wQEqxaXjcPqfb802WWwhzXz9_Ayw?usp=sharing"

   
   cd ros2_bags/dynamic_road03
    
   ros2 bag play --loop ros2bags.db3
   ```   
Note: In some cases, you may need to launch a program like our LiDAR Registration code first. If so, just relaunch this one after Terminal 3.

---
### Terminal 2: Launch RViz

1. **Start a Container for RViz:**

   ## WSL and Linux users
   ```bash
   docker run -it --network rosnet -p 5901:5900 -p 6081:6080 -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_amd:latest
   ```
   ## MacOS users
   ```bash
   docker run -it --network rosnet -p 5901:5900 -p 6081:6080 -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_arm:latest
   ```
2. **Inside the Container, Launch RViz:**

   ```bash
   rviz2
   ```

3. **Access via Browser:**

   Open a web browser and navigate to: and switch the base frame **from "map" to "velodyne"**

   ```
   http://localhost:6081/vnc_auto.html
   
   ```
   Replace `<host_IP>` with your machine’s IP address or use `localhost` if running locally.

---

### Terminal 3: Lidar Calibration Node

1. **Run the Container & Build Workspace:**

   ## WSL and Linux users
   ```bash
   docker run -it --network rosnet -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_amd:latest
   ```
   ## MacOS users
   ```bash
   docker run -it --network rosnet -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_arm:latest
   ```
2. **Inside the Container:**

   ```bash
   cd colcon_ws/
   # colcon build --packages-select lidar_calibration --symlink-install
   . install/setup.bash
   ros2 run lidar_calibration uncalibrated_node  # check /uncalibrated_points topic in rviz
   # and close this node , and run it
   ```
   ```bash
   ros2 run lidar_calibration lidar_calibration_node  # check /merged_points topic in rviz
   ```

### Terminal 4: Run Odometry Node (KISS-ICP)

1. **Start a Container for the Odometry Node:**

   ## WSL and Linux users
   ```bash
   docker run -it --network rosnet -p 5900:5900 -p 6080:6080 -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_amd:latest
   ```
   ## MacOS users
   ```bash
   docker run -it --network rosnet -p 5900:5900 -p 6080:6080 -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_arm:latest
   ```
2. **Inside the Container, Build and Launch:**

After running the kiss_icp node **for SLAM, you can see the localized position of the vehicle on /kiss/odometry topic and map on /kiss/local_map , but for that set the frame "odom_lidar"**

   ```bash
   cd kiss_icp_ws/
   # colcon build --packages-select kiss_icp --symlink-install
   . install/setup.bash

   ros2 launch kiss_icp odometry.launch.py topic:=/velodyne_points 
   # and close this node 
   ```
   ```bash
   ros2 launch kiss_icp odometry.launch.py topic:=/merged_points   # check the rviz trajectory
   ```
3. **Access the Odometry Interface:**

   Open a browser and go to:
   ```
   http://localhost:6080/vnc_auto.html
   ```

---
---
### Terminal 5: Multi Lidar Calibration Node(Optional)

If you want to test the NDT(Normal Distribution Transform) for lidar calibration then you can use the off shelf following ros2 package.

1. **Run the Container & Build Workspace:**

   ## WSL and Linux users
   ```bash
   docker run -it --network rosnet -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_amd:latest
   ```
   ## MacOS users
   ```bash
   docker run -it --network rosnet -e DISPLAY=:0 -v "$(pwd):/workspace/" thinkautonomous/ros2_humble_gto_arm:latest
   ```
2. **Inside the Container:**

   ```bash
   cd multi_lidar_ws/
   # colcon build --packages-select multi_lidar_calibration --symlink-install
   . install/setup.bash
   ros2 launch multi_lidar_calibration multi_lidar_calibration_ndt.launch.xml


---



## Accessing the VNC/noVNC Interface

- **For RViz (Terminal 2):**
  - **VNC Client:** Connect to `<host_IP>:5901`
  - **Web Browser:** Open `http://<host_IP>:6081/vnc_auto.html`

- **For Odometry (Terminal 5):**
  - **VNC Client:** Connect to `<host_IP>:5900`
  - **Web Browser:** Open `http://<host_IP>:6080/vnc_auto.html`

> **Tip:**  
> Replace `<host_IP>` with your actual host IP address. On Linux/macOS, if you're running on the same machine, you can often use `localhost`.

---


## Troubleshooting

- **Port Conflicts:**  
  Make sure that the host ports (5901, 6081, etc.) are not in use by other services.

- **Container Logs:**  
  Check logs with:
  ```bash
  docker logs <container_id>
  ```
  to troubleshoot startup issues for Xvfb, x11vnc, or websockify.

- **Networking:**  
  Ensure the Docker network `rosnet` is created and that containers are connected properly:
  ```bash
  docker network ls
  ```

---
## Output:
![Alt text](images/without_registration.jpg)

**This is the point cloud by merging the point cloud from velodyne(green) and Ouster 1 lidar, without registration or alignment.**
![Alt text](images/icp.jpg)

**This is a primary version of the merged point cloud with standard point to plane ICP method from Open3D**
![Alt text](images/multi_scale_GICP_ICP1.jpg)
![Alt text](images/multi_scale_GICP_ICP2.jpg)

**This is a final version of the merged point cloud with standard point to plane GICP followed by ICP method from Open3D with multi scaled approached to get the better alignment.**

![Alt text](images/ros_output.png)

**This is the output from rviz when we achieve the point cloud registration using the fast global registration for global registration, followed by Generalized ICP and ICP with multi scaled approach for the refined registration between OS1 and velodyne lidars in velodyne frame.**


![Alt text](images/slam_kiss_icp.gif)

**This is the output for the SLAM using kiss_icp alogorithm operating on registered point clouds fom velodyne and os1, showing the localized actor trajectory and local map.**


This README provides a complete guide for setting up and accessing your ROS 2 multi-container system on Linux, macOS, and WSL2. Adjust the IP addresses and ports as needed based on your network configuration. 

Let me know if you have any questions or need further assistance!
