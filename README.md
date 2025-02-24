# ws24-multi-robot-task-distribution

# Multi-Robot Task Distributions

## Description

The project aims to migrate a multi-robot task distribution system, originally developed using ROS1 to ROS2. The system simulates robots working collaboratively in a warehouse to fetch items based on client requests. The migration ensures compatibility with ROS2’s improved communication features and long-term support. ROS2 provides enhanced multi-robot communication, security, and real-time control features. Thus, migration is required to ensure system sustainability and to benefit from ROS2’s advanced tools.

## Features

1) The system will be able to run and simulate in ROS2, provinding a stable environment for multi-robot communication.

## Scope



## Tools/Software Used

1) Ubuntu 20.04
2) ROS Humble
3) Python
4) Git
5) DDS


# Installation and Project setup.
You need ROS Humble version to use these packages.

- Please install ros humble using this link and instructions [Installation Guide](https://docs.ros.org/en/humble/Installation.html)

## Prerequisites

Make sure you have the following installed on your system:
- ROS (Robot Operating System) 
- Git

## 1. Create a ROS Workspace
First, create a new ROS workspace and initialize it:

```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/
colcon build

source install/setup.bash
```
### Clone the Package 
Navigate to the src directory of your workspace and clone the repository:

```
cd ~/ros_ws/src
git clone git@github.com:HBRS-SDP/ss24-multi-robot-task-distribution.git
```

Install the required dependencies.
```
cd ~/ros_ws 
rosdep install --from-paths src --ignore-src -r -y
```

Build the Package.

```
colcon build --symlink-install
```

## Run the Nodes
You can now run the nodes provided by your package. Open a new terminal, source the workspace, and use rosrun or roslaunch to start your nodes:

```
# Source the workspace
source ~/ros_ws/install/setup.bash
```

# Simulation
For the testing purpose, we made a simulation environment that somewhat resembles the warehouse with 8 shelves.
It starts the gazebo environment, map server, and rviz window and can be launched using
```
ros2 launch simulation_env demo_world.launch.py
```
![Simulation Environment](docs/simulation_environment.png?raw=true)

# SM_Module
Shared Memory Module logs he robot activities and manages the Inventory. It integrates with ROS to handle real-time updates and communicate with robots.

# Task Distribution Overview

## Multi-Robot Task Distribution in ROS2

In our ROS2 migration, we replace the ZeroMQ-based broker with a ROS2-native solution DDS for distributing tasks among robots. The architecture consists of a **Broker Node** that assigns tasks to **Worker Nodes** (robots) based on availability and load balancing.

### Architecture Overview

1. **Broker Node**:
   - Listens for task requests from **Client Nodes** via ROS2 services or actions.
   - Maintains a registry of **Worker Nodes** and their statuses (idle, busy, etc.).
   - Distributes tasks to available workers based on a custom load-balancing strategy.

2. **Worker Nodes**:
   - Publish their availability on a status topic (e.g., `/worker_status`).
   - Receive and execute tasks assigned by the Broker.
   - Update the Broker on task progress and completion using actions or services.

3. **Client Nodes**:
   - Send task requests to the Broker, which then routes them to appropriate Worker Nodes.

### Implementation Notes

- **ROS2 Services** and **Actions** enable reliable task assignment and feedback.
- **Quality of Service (QoS)** settings ensure robust communication, even with varying network conditions.
- **DDS Discovery** allows dynamic scaling, enabling new Worker Nodes to register with the Broker automatically.

This ROS2 setup leverages DDS for fault tolerance, scalability, and efficient intra-process communication, making it ideal for multi-robot task distribution.
