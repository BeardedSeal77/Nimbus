# Meeting with ntsuaviation - DJI Drone Upgrade Proposal

## Executive Summary

This document outlines the technical limitations of our current NLP drone project using the DJI Tello EDU and proposes specific DJI drone models that will address these constraints. Our project requires advanced positioning, depth sensing, and autonomous navigation capabilities that are not available with our current platform.

## Current Project Overview

**Project**: Natural Language Processing Autonomous Drone Navigation System  
**Current Platform**: DJI Tello EDU  
**System Architecture**: Ground-based AI processing with ROS2 integration  
**Primary Components**:
- YOLOv11 object detection (`object_detect_node_v11.py`)
- Structure from Motion (SfM) depth estimation (`depth_node.py`)
- Natural language processing and intent recognition
- Ground station AI processing system (`nimbus-ai/`)

<div style="page-break-after: always;"></div>

## Technical Limitations with DJI Tello EDU

### 1. **Position Tracking & SLAM Deficiencies**
- **No GPS positioning data**: Tello EDU provides no location telemetry
- **No inertial measurement unit (IMU) access**: Cannot track drone orientation/movement
- **No simultaneous localization and mapping (SLAM)**: Unable to build environmental maps
- **Position estimation errors**: Must assume drone reaches commanded positions (error propagation risk)

### 2. **Depth Detection Limitations**
- **Single camera feed**: No stereo vision capabilities
- **Unreliable Structure from Motion**: Hit-or-miss depth estimation from monocular camera
- **No LiDAR or depth sensors**: Relying entirely on visual depth estimation
- **No obstacle avoidance sensor data**: Cannot access proximity sensor information

### 3. **Limited Sensor Integration**
- **Restricted SDK access**: Limited telemetry and sensor data availability
- **No real-time positioning**: Cannot track drone location in 3D space
- **Basic obstacle avoidance**: No access to obstacle detection sensor data

### 4. **System Architecture Constraints**
- **Ground-based processing requirement**: All AI processing occurs on ground station
- **Video feed limitations**: Single low-resolution camera stream
- **No dual-camera systems**: Cannot implement stereo vision for depth

<div style="page-break-after: always;"></div>

## RTK Positioning Technology

**Real-Time Kinematic (RTK)** is a high-precision GPS enhancement technique that provides centimeter-level positioning accuracy instead of the typical 3-5 meter accuracy of standard GPS systems.

### How RTK Works:
1. **Base Station Setup**: A fixed RTK base station is positioned at a precisely surveyed location
2. **Correction Data Transmission**: The base station continuously sends correction signals to the rover (drone)
3. **Error Correction**: Real-time corrections eliminate atmospheric interference, satellite clock errors, and orbital variations
4. **Precise Positioning**: Achieves 1-2cm horizontal accuracy and 2-3cm vertical accuracy

### RTK Benefits for Autonomous Drones:
- **Absolute Position Reference**: Provides precise global coordinates for navigation
- **SLAM Initialization**: Gives accurate starting position for mapping algorithms  
- **Loop Closure Detection**: Enables recognition of previously visited locations
- **Drift Elimination**: Prevents accumulation of positioning errors over time
- **Indoor/Outdoor Transitions**: Maintains positioning continuity across environments

### RTK vs Standard GPS:
| System | Horizontal Accuracy | Vertical Accuracy | Update Rate |
|--------|-------------------|------------------|-------------|
| Standard GPS | 3-5 meters | 5-10 meters | 1-10 Hz |
| RTK GPS | 1-2 cm | 2-3 cm | 10-20 Hz |

<div style="page-break-after: always;"></div>

## SLAM Requirements Analysis

### What SLAM Needs:
1. **Multiple sensor inputs**: IMU, cameras, LiDAR, or depth sensors
2. **Positioning data**: GPS, visual-inertial odometry, or RTK positioning
3. **Environmental mapping**: Ability to build and update 3D maps
4. **Loop closure detection**: Recognition of previously visited locations
5. **Real-time processing**: Low-latency position estimation and mapping

### Current Capabilities vs. SLAM Requirements:
| Requirement | Tello EDU | Required |
|-------------|-----------|----------|
| Stereo/depth cameras | L Single camera |  Stereo cameras or LiDAR |
| IMU access | L No access |  Full IMU telemetry |
| GPS/positioning | L No GPS data |  RTK/GPS positioning |
| Obstacle sensors | L No sensor access |  Multi-directional sensors |
| SDK integration | L Limited |  Full ROS2/SDK support |

<div style="page-break-after: always;"></div>

## Recommended DJI Drone Solutions

### Primary Recommendation: **DJI Mavic 3 Enterprise Series**

#### Technical Specifications:
- **Positioning System**: RTK module with centimeter-level precision
  - Horizontal accuracy: 1 cm + 1 ppm
  - Vertical accuracy: 1.5 cm + 1 ppm
- **Obstacle Avoidance**: Omnidirectional binocular vision system
  - Forward/Backward: 0.5-20m measurement range
  - Left/Right: 0.5-16m measurement range  
  - Up/Down: 0.5-25m measurement range
- **Camera System**: 4/3 CMOS sensor with mechanical shutter
- **ROS2 Integration**: Full support via `psdk_ros2` wrapper
- **Flight Time**: Up to 43 minutes
- **Weather Resistance**: IP54 rating

#### SLAM Capabilities:
- **Visual-Inertial SLAM**: Can run NVIDIA Isaac ROS Visual SLAM
- **RTK Positioning**: Provides precise global positioning for SLAM initialization
- **Multi-sensor Fusion**: Combines visual, inertial, and GPS data
- **ROS2 Navigation Stack**: Compatible with Nav2 and slam_toolbox

#### Benefits for Our Project:
1. **Precise Positioning**: RTK provides accurate drone location tracking
2. **Sensor Data Access**: Full obstacle avoidance sensor telemetry
3. **Depth Information**: Access to omnidirectional distance sensors
4. **Ground Station Processing**: Maintains our current architecture
5. **Advanced SDK**: Complete ROS2 integration with psdk_ros2

<div style="page-break-after: always;"></div>

### Alternative Recommendation: **DJI Matrice 30T**

#### Key Features:
- **Integrated Thermal Camera**: Built-in thermal imaging (640×512 resolution)
- **Laser Rangefinder**: 1200m range measurement capability
- **Multi-sensor Payload**: Wide, zoom, and thermal cameras
- **IP55 Weather Rating**: Superior weather resistance
- **Rapid Deployment**: 1-minute setup time
- **Emergency Response Focus**: Designed for search and rescue operations

#### Benefits:
- **Distance Measurement**: Direct laser rangefinder eliminates depth estimation issues
- **Multi-modal Detection**: Thermal + RGB for enhanced object detection
- **Robust Operation**: Better weather resistance for outdoor testing
- **Professional SDK Support**: Full PSDK integration

### Enterprise Option: **DJI Matrice 300 RTK**

#### Professional Specifications:
- **Flight Time**: Up to 55 minutes
- **Payload Capacity**: Up to 3 gimbals simultaneously
- **6-Directional Obstacle Sensing**: ToF sensors on all sides
- **Detection Range**: Up to 40m obstacle detection
- **Custom Payload Support**: PSDK for custom sensor integration
- **Advanced AI**: Built-in edge computing capabilities

#### Research Benefits:
- **Custom Sensor Integration**: Can add specialized sensors
- **Multiple Camera Angles**: Simultaneous multi-gimbal operation
- **Extended Testing Time**: Longer flight duration for research
- **Professional Development Platform**: Full SDK access for custom applications

<div style="page-break-after: always;"></div>

## ROS2 Integration Architecture

### psdk_ros2 Wrapper Capabilities:
- **Real-time Telemetry**: Position, velocity, orientation, battery status
- **Camera Control**: Gimbal control and camera parameter management
- **Flight Control**: Waypoint navigation and velocity commands
- **Sensor Data**: Access to all obstacle avoidance sensors
- **Standard ROS Topics**: Follows REP 105 and REP 103 standards

### SLAM Integration Options:
1. **NVIDIA Isaac ROS Visual SLAM**: GPU-accelerated visual SLAM
2. **SLAM Toolbox**: Standard ROS2 SLAM implementation
3. **ORB-SLAM3**: Monocular, stereo, and RGB-D SLAM
4. **RTAB-Map**: Real-time appearance-based mapping

### System Architecture Enhancement:
```
Ground Station (Current):
- YOLOv11 Object Detection
- Natural Language Processing  
- Flight Command Generation
- Mission Planning

New Drone Capabilities:
- RTK Positioning Data → SLAM System
- Obstacle Sensor Data → Navigation Stack
- High-resolution Video → Enhanced Detection
- IMU/GPS Telemetry → State Estimation
```

<div style="page-break-after: always;"></div>

## Technical Implementation Plan

### Phase 1: Hardware Upgrade
1. **Acquire DJI Mavic 3 Enterprise** with RTK module
2. **Install psdk_ros2 wrapper** for ROS2 integration
3. **Configure RTK base station** for precise positioning

### Phase 2: Software Integration
1. **Integrate SLAM system** (Isaac ROS Visual SLAM or SLAM Toolbox)
2. **Enhance object detection** with improved video feed
3. **Implement sensor fusion** for depth estimation
4. **Add autonomous navigation** with Nav2 stack

### Phase 3: System Testing
1. **Indoor SLAM validation** without GPS
2. **Outdoor RTK positioning tests** 
3. **Autonomous navigation trials** with obstacle avoidance
4. **End-to-end system integration** with NLP commands

## Expected Improvements

### Positioning Accuracy:
- **Current**: Unknown position (assumption-based)
- **With RTK**: ±1cm horizontal, ±1.5cm vertical precision

### Depth Detection:
- **Current**: Unreliable SfM from single camera
- **With New System**: Direct sensor measurements + visual SLAM

### Navigation Reliability:
- **Current**: Open-loop commands (no feedback)
- **With SLAM**: Closed-loop control with real-time position feedback

### Object Detection Enhancement:
- **Current**: 720p single camera
- **With Mavic 3**: 4K camera with mechanical shutter and gimbal stabilization

<div style="page-break-after: always;"></div>

## Meeting Objectives

### Primary Goals:
1. **Secure access** to DJI Mavic 3 Enterprise or Matrice 30T
2. **Establish partnership** for ongoing technical support
3. **Negotiate educational pricing** for university research project
4. **Plan implementation timeline** for hardware integration

### Technical Discussion Points:
1. **RTK base station requirements** and setup procedures
2. **PSDK integration support** and documentation access
3. **ROS2 compatibility** and technical assistance
4. **Training requirements** for advanced drone operation
5. **Maintenance and support** considerations for research environment

### Questions for ntsuaviation:
1. What is the lead time for Mavic 3 Enterprise with RTK module?
2. Do you provide technical training for PSDK and ROS2 integration?
3. Is there educational pricing available for university research projects?
4. What ongoing support is available for technical implementation?
5. Can you provide RTK base station setup and configuration support?

## Conclusion

Upgrading from the DJI Tello EDU to a professional-grade drone with RTK positioning, obstacle avoidance sensors, and full SDK access will transform our NLP drone project from a proof-of-concept to a robust autonomous navigation system. The recommended solutions address all current technical limitations while providing a foundation for advanced research in autonomous drone navigation and human-robot interaction.

The investment in professional drone hardware will enable:
- **Reliable SLAM implementation** for indoor and outdoor navigation
- **Precise positioning tracking** eliminating error propagation
- **Enhanced sensor integration** for robust obstacle avoidance  
- **Professional development platform** for future research expansion

This upgrade is essential for the successful completion of our research objectives and will provide invaluable experience with industry-standard autonomous drone systems.