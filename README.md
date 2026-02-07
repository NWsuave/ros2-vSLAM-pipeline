> **Status: Archived (hardware dependent)**  
> Development is paused due to lack of access to the required physical hardware. This repository documents an experimental ROS 2 SLAM pipeline built for a specific TurtleBot setup.


# Visual SLAM Pipeline (ORB-SLAM3 + RTAB-Map)

This repository contains scripts, launch files, and configuration used to run a **monocular visual SLAM pipeline** using **ORB-SLAM3** and **RTAB-Map** on a TurtleBot-based platform with a USB camera and LiDAR.

The system separates **visual odometry** (ORB-SLAM3) from **mapping and loop closure** (RTAB-Map) and includes helper tools to manage camera format conversion, TF publishing, and visualization.

This project was created for learning and system integration purposes and is not intended to be a fully self contained or plug and play solution.

---

## Camera Pipeline (Format Conversion + TF)

 Launch Camera Conversion + TF
```bash
ros2 launch ~/ws_tools/launch/campipe.launch.py
```
or via alias:
```bash
campipe
```
### Description

This launch file starts the video processing pipeline required for visual SLAM. It converts the raw USB camera stream from YUY2 (YUV422) format to BGR8, which is the expected input format for both ORB-SLAM3 and RTAB-Map. The launch file also publishes the required static camera transform when needed.

### Functionality

- Subscribes to /image_raw (YUY2 / YUV422)
- Converts and republishes to /image_raw/bgr8
- Publishes static camera transforms when required
- Ensures camera frames are compatible with SLAM algorithms

## ORB-SLAM3 Monocular Pipeline

Run ORB-SLAM3 Driver
```bash
~/ws_tools/scripts/run_orbslam_mono_driver.sh
```

or via alias:
```bash
orbmono_driver
```
### Description

This script initializes the ORB-SLAM3 monocular SLAM pipeline by sourcing the required ROS 2 and workspace environments and launching the ORB-SLAM3 monocular driver node.

### Node Executed
ros2_orb_slam3/mono_driver_node.py

### Inputs

Monocular BGR camera stream
/image_raw/bgr8

### Outputs

Estimated camera pose

Visual odometry (/odom)

SLAM-generated transforms and topics for visualization and mapping

## RTAB-Map (Mapping + Loop Closure)

RTAB-Map is configured to use external odometry produced by ORB-SLAM3 and combines it with camera and LiDAR data to generate a globally consistent map.

## Assumptions
- ROS 2 workspace is already configured
- ORB-SLAM3 and RTAB-Map are installed locally
- Camera publishes /image_raw in YUY2 or YUV422 format
- TF frames match those referenced in the launch files
