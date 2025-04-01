# ROS Humble Multi-Platform Docker Setup

This guide details how to run a multi-container ROS 2 setup using Docker on Linux, macOS, and WSL2. The containers use a virtual display (via Xvfb) and expose VNC/noVNC for GUI access.

> **Note:**  
> Ensure Docker (or Docker Desktop) is installed and configured for your platform. If you’re using WSL2 on Windows, ensure Docker Desktop is set up with WSL2 integration.

---

## Table of Contents

- [Create Docker Network](#create-docker-network)
- [Running the Containers](#running-the-containers)
  - [Terminal 1: Lidar Calibration Node](#terminal-1-lidar-calibration-node)
  - [Terminal 2: Launch RViz](#terminal-2-launch-rviz)
  - [Terminal 3: Run Odometry Node (KISS-ICP)](#terminal-3-run-odometry-node-kiss-icp)
  - [Terminal 4: ROS Bag Playback](#terminal-4-ros-bag-playback)
- [Accessing the VNC/noVNC Interface](#accessing-the-vncnovnc-interface)
- [WSL2 Specific Instructions](#wsl2-specific-instructions)
- [Troubleshooting](#troubleshooting)

---

## Create Docker Network

Before launching any containers, create the Docker network `rosnet` so that your containers can communicate:

```bash
docker network create rosnet
```

---

## Running the Containers

The following instructions apply to Linux and macOS. The same commands will work in WSL2 (see [WSL2 Specific Instructions](#wsl2-specific-instructions) below).

### Terminal 1: Lidar Calibration Node

1. **Run the Container & Build Workspace:**

   ```bash
   docker run -it --network rosnet -e DISPLAY=:0 -v "$(pwd):/workspace/" mayakshanesht/ros_humble_multi_plt_final:latest
   ```

2. **Inside the Container:**

   ```bash
   cd colcon_ws/
   colcon build
   . install/setup.bash
   ros2 run lidar_calibration lidar_calibration_node
   ```

---

### Terminal 2: Launch RViz

1. **Start a Container for RViz:**

   ```bash
   docker run -it --network rosnet -p 5901:5900 -p 6081:6080 -e DISPLAY=:0 -v "$(pwd):/workspace/" mayakshanesht/ros_humble_multi_plt_final:latest
   ```

2. **Inside the Container, Launch RViz:**

   ```bash
   rviz2
   ```

3. **Access via Browser:**

   Open a web browser and navigate to:
   ```
   http://<host_IP>:6081/vnc_auto.html
   ```
   Replace `<host_IP>` with your machine’s IP address or use `localhost` if running locally.

---

### Terminal 3: Run Odometry Node (KISS-ICP)

1. **Start a Container for the Odometry Node:**

   ```bash
   docker run -it --network rosnet -p 5902:5900 -p 6082:6080 -e DISPLAY=:0 -v "$(pwd):/workspace/" mayakshanesht/ros_humble_multi_plt_final:latest
   ```

2. **Inside the Container, Build and Launch:**

   ```bash
   cd colcon_ws
   colcon build
   . install/setup.bash
   ros2 launch kiss_icp odometry.launch.py topic:=/merged_points
   ```

3. **Access the Odometry Interface:**

   Open a browser and go to:
   ```
   http://<host_IP>:6082/vnc_auto.html
   ```

---

### Terminal 4: ROS Bag Playback

1. **Start a Container for ROS Bag Playback:**

   ```bash
   docker run -it --network rosnet -e DISPLAY=:0 -v "$(pwd):/workspace/" mayakshanesht/ros_humble_multi_plt_final:latest
   ```

2. **Inside the Container, Play the ROS Bag:**

   ```bash
   cd ros2bags
   ros2 bag play --loop ros2bags.db3
   ```

---

## Accessing the VNC/noVNC Interface

- **For RViz (Terminal 2):**
  - **VNC Client:** Connect to `<host_IP>:5901`
  - **Web Browser:** Open `http://<host_IP>:6081/vnc_auto.html`

- **For Odometry (Terminal 3):**
  - **VNC Client:** Connect to `<host_IP>:5902`
  - **Web Browser:** Open `http://<host_IP>:6082/vnc_auto.html`

> **Tip:**  
> Replace `<host_IP>` with your actual host IP address. On Linux/macOS, if you're running on the same machine, you can often use `localhost`.

---

## WSL2 Specific Instructions

When using WSL2 on Windows:

1. **Determine Your WSL2 IP:**

   In your WSL2 terminal, run:
   ```bash
   ip addr show eth0
   ```
   Look for the `inet` address (e.g., `172.17.100.209`).

2. **Run Containers Using the Same Commands:**

   Use the same Docker run commands as above. The `-e DISPLAY=:0` works if you rely on the container's internal Xvfb.  
   If you need to forward the display from WSL2 to Windows, you can use an X server for Windows (e.g., VcXsrv or Xming) and adjust the `DISPLAY` variable accordingly (e.g., `DISPLAY=localhost:0.0`).  
   Alternatively, using the built-in VNC/noVNC interface as described below is often easier.

3. **Access VNC/noVNC:**

   - **From Windows:**  
     If your WSL2 instance has IP `172.17.100.209` and you've mapped ports correctly, you can access the noVNC interface via:
     ```
     http://172.17.100.209:6081/vnc_auto.html
     ```
   - **Port Forwarding Option:**  
     If accessing the WSL2 IP directly is problematic, use Windows’ `netsh` to forward the port:
     ```powershell
     netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=6081 connectaddress=172.17.100.209 connectport=6081
     ```
     Then, access via `http://localhost:6081/vnc_auto.html` in your Windows browser.

---

## Troubleshooting

- **X Server/Display Issues:**  
  If parts of the GUI aren’t visible, verify that Xvfb is running correctly inside the container. For native host display, ensure your host X server is configured to allow connections (e.g., `xhost +local:docker`).

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

This README provides a complete guide for setting up and accessing your ROS 2 multi-container system on Linux, macOS, and WSL2. Adjust the IP addresses and ports as needed based on your network configuration. 

Let me know if you have any questions or need further assistance!