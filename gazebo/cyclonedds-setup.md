# CycloneDDS Setup for Nimbus

## Overview
CycloneDDS is an alternative DDS implementation for ROS2 that can provide better network performance and discovery control. This document covers setup for future use if the default ROS2 DDS doesn't work properly.

## Installation

### WSL2 Ubuntu
```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

### Docker Container
Add to Dockerfile or docker-compose:
```yaml
command: ["bash", "-c", "apt update && apt install -y ros-jazzy-rmw-cyclonedds-cpp && ..."]
```

## Configuration

### Environment Variables
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDX_URI=file:///path/to/cyclonedx.xml
```

### XML Configuration File
Create `cyclonedx.xml` for network discovery:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDX xmlns="https://raw.githubusercontent.com/eclipse-cyclonedx/cyclonedx/iceoryx/etc/cyclonedx.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>192.168.8.102</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer address="192.168.8.102"/>  <!-- Docker ROS2 -->
                <Peer address="192.168.8.101"/>  <!-- Server -->
            </Peers>
        </Discovery>
    </Domain>
</CycloneDX>
```

## Known Issues with ROS2 Jazzy

⚠️ **Buffer Overflow Warning**: There are reported issues with ROS2 Jazzy + CycloneDDS causing buffer overflow errors. This combination works fine with Humble/Iron but has problems with Jazzy.

**Symptoms:**
```
*** buffer overflow detected ***: terminated
```

**Workaround:** Use default ROS2 DDS with static peers instead.

## Network Configuration

For cross-machine communication:
- Set same `ROS_DOMAIN_ID` on all machines
- Configure `CYCLONEDX_URI` to point to XML config
- Ensure firewall allows DDS ports (7400, 7401, etc.)
- Use static peer discovery for reliable connection

## Usage Example

```bash
# Terminal 1 (Gazebo machine)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDX_URI=file:///path/to/gazebo/cyclonedx.xml
export ROS_DOMAIN_ID=0
ros2 run ros_gz_bridge parameter_bridge [topics...]

# Terminal 2 (Docker machine)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDX_URI=file:///path/to/docker/cyclonedx.xml
export ROS_DOMAIN_ID=0
ros2 run rosbridge_server rosbridge_websocket
```

## References
- [ROS2 CycloneDDS Documentation](https://docs.ros.org/en/jazzy/p/rmw_cyclonedds_cpp/)
- [CycloneDDS GitHub](https://github.com/eclipse-cyclonedx/cyclonedx)
- [Jazzy Buffer Overflow Issue](https://github.com/eclipse-cyclonedx/cyclonedx/issues/2043)