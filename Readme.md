# Crazyflie ROS Packages

This collection of ROS packages provides a comprehensive framework for controlling and interfacing with the Crazyflie drone. Designed with modularity and flexibility in mind, it allows for easy integration into existing ROS ecosystems, enabling advanced control algorithms, custom communication messages, and direct interaction with the Crazyflie hardware.

## Overview

The Crazyflie ROS packages are divided into three main components, each serving a distinct role within the system:

- **crazyflie_controller**: Implements control algorithms for the Crazyflie drone, including but not limited to PID control, to ensure stable flight and maneuvering capabilities.

- **crazyflie_msgs**: Defines custom ROS message types specifically designed for Crazyflie operations, facilitating effective communication within the ROS environment.

- **crazyflie_server**: Acts as the bridge between the ROS system and the Crazyflie drone, handling command dispatch and sensor data collection.

## Getting Started

### Prerequisites

Ensure you have a ROS distribution installed on your system (e.g., ROS Noetic). The packages have been tested on Ubuntu 20.04 LTS, but they should be compatible with other Linux distributions supporting ROS.

### Installation

1. Clone this repository into your catkin workspace's `src` directory.
2. Navigate back to your catkin workspace root and run `catkin_make` to build the packages.
3. Source your workspace's setup file with `source devel/setup.bash`.

### Running the Packages

To launch the Crazyflie server and controller, use the following command:

```bash
roslaunch crazyflie_server server.launch