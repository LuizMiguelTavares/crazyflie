# Crazyflie ROS Packages

This repository provides a modular and flexible collection of ROS packages designed to interface with and control the **Crazyflie drone**. It enables seamless integration with existing ROS systems, offering support for advanced control algorithms, custom message types, and direct communication with the Crazyflie hardware.

---

## Overview

The Crazyflie ROS packages are structured into two primary components:

- **`crazyflie_msgs`**:  
   Defines custom ROS message types tailored for Crazyflie operations, ensuring efficient communication within the ROS environment.

- **`crazyflie_server`**:  
   Acts as a bridge between the ROS ecosystem and the Crazyflie drone. It handles command dispatching and sensor data collection.

---

## Getting Started

### Prerequisites

- **ROS Distribution**: Ensure ROS (e.g., **ROS Noetic**) is installed on your system.  
- **Operating System**: Tested on **Ubuntu 20.04 LTS** but compatible with other Linux distributions that support ROS.

---

### Installation

1. Clone the repository into your catkin workspace's `src` directory:
   ```bash
   git clone <repository_url> ~/catkin_ws/src
   ```

2. Build the packages:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. Source your workspace:
   ```bash
   source devel/setup.bash
   ```

---

### Running the Packages

To launch the **Crazyflie server** and controller, run the following command, replacing `<crazyflie ID>` with the appropriate ID:

```bash
roslaunch crazyflie_server server.launch ID:=<crazyflie ID>
```

You can optionally set a namespace for the topics by specifying:

```bash
namespace:=<desired_namespace>
```

---

## Generated Topics

The **`server.launch`** file generates the following topics:

1. **`/<namespace>/cmd_vel`** (`geometry_msgs/Twist`):  
   Used to send velocity commands:  
   - **`linear.x`**: Linear velocity in the X direction  
   - **`linear.y`**: Linear velocity in the Y direction  
   - **`linear.z`**: Thrust (vertical velocity)  
   - **`angular.z`**: Angular velocity for yaw  

2. **`/<namespace>/crazyflieLog`** (`custom message`):  
   Provides detailed drone telemetry data:  
   ```plaintext
   float64 timestamp   # Timestamp
   float64 vx          # Velocity in X
   float64 vy          # Velocity in Y
   float64 vz          # Velocity in Z
   float64 z           # Altitude
   float64 pitch       # Pitch angle
   float64 roll        # Roll angle
   float64 yaw         # Yaw angle
   float64 thrust      # Thrust value
   ```

3. **`/<namespace>/crazyflieVel`** (`geometry_msgs/Vector3`):  
   Publishes velocity data as a 3D vector.

4. **`/<namespace>/crazyflieAng`** (`geometry_msgs/Twist`):  
   Publishes angular data as a `Twist` message.