# ws24-multi-robot-task-distribution

## Multi-Robot Task Distribution System

### Description

This project implements a multi-robot task distribution system designed for warehouse automation. The system simulates robots collaborating in a warehouse environment to efficiently fetch and deliver items based on client requests. It leverages ROS2 for robust communication and coordination between multiple robots, ensuring seamless task allocation and execution.

### Key Features

1. **ROS2-Based Architecture**: Built on ROS2 for enhanced communication and scalability.
2. **User-Friendly GUI**: Provides an intuitive interface for managing orders and monitoring system performance.
3. **Task Distribution**: Efficient task allocation using a centralized architecture.
4. **Centralized Logging**: Logs are published to a central logging topic for monitoring and debugging.

---

## Scope

The project is designed to demonstrate the following:
- Multi-robot collaboration in a warehouse environment.
- Real-time task distribution and load balancing.
- Integration of ROS2 features like DDS, QoS, and Actions for robust communication.
- Scalability for any number of robots.
---

## Tools/Software Used

1. **Ubuntu 20.04**
2. **ROS2 Humble**
3. **Python**
4. **Git**
5. **DDS (Data Distribution Service)**
6. **Flask**

---

## Installation and Project Setup

### Prerequisites

Ensure the following are installed on your system:
- ROS2 Humble (Refer to the [Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- Git

### 1. Create a ROS Workspace

Create a new ROS workspace and initialize it (or use this repository as a workspace):

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/
colcon build
source install/setup.bash
```

### 2. Clone the Package

Navigate to the `src` directory of your workspace and clone the repository:

```bash
cd ~/ros_ws/src
git clone https://github.com/HBRS-SDP/ws24-multi-robot-task-distribution.git
```

### 3. Install Dependencies

Install the required dependencies:

```bash
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y
pip install flask
```

### 4. Build the Package

Build the package using `colcon`:

```bash
colcon build --symlink-install
```

---

## Launching the System

To start the system, follow these steps:

### 1. Source the Workspace

Before launching any nodes, ensure the workspace is sourced:

```bash
source ~/ros_ws/install/setup.bash
```
### 2. Set the Number of Robots (Optional)
The system allows you to configure the number of robots by setting the NUM_OF_ROBOTS environment variable. This variable, **if not set, takes default value of 2**.

To set the number of robots, use the following command:
```bash
export NUM_OF_ROBOTS=4  # Replace 4 with the desired number of robots
```
This value will be used by the Simulation Environment and Warehouse Manager both to initialize the robot fleet, so ensure this variable is exported in all terminals before launching the system.

### 3. Launch the Simulation Environment

The simulation environment replicates a warehouse with 8 shelves and includes:
- **Gazebo**: For 3D simulation.
- **Map Server**: For navigation.
- **RViz**: For visualization (Optional).

Launch the simulation environment using:

```bash
ros2 launch simulation_env sim_with_nav.launch.py
```
Alternatively, to launch the simulation with RViz enabled, use this command:
```bash
ros2 launch simulation_env sim_with_nav.launch.py enable_rviz:=true
# The `enable_rviz` parameter, when set to `true`, launches RViz for visualizing the robots' navigation and tasks in real-time.
```
This launches usual simulation and additionally opens up RViz window for each robot for monitoring or testing individually.

![Simulation Environment](docs/simulation_rviz.png?raw=true)

### 4. Launch the Warehouse Manager

The Warehouse Manager initializes the core nodes required for the system. It includes:
- **Logger Node**: Logs simulation events and data.
- **Fleet Manager Node**: Manages the robot fleet, with the number of robots specified by the `num_of_robots` parameter.
- **Shared Memory Node**: Handles shared memory for inter-process communication.
- **Task Manager Node**: Distributes tasks among robots.

Run the following command to launch the Warehouse Manager:

```bash
ros2 launch simulation_env warehouse_manager.launch.py
```

### 5. Start the Web Client

The Web Client provides a graphical interface for managing orders and monitoring the system. It interacts with the Shared Memory to access data.

Navigate to the Web Server directory and start the server:

```bash
cd ~/ros_ws/src/ws24-multi-robot-task-distribution/src/web_server
./Server.py
```

Once started, access the Web Client through your browser to place orders and monitor the system in real-time.

![Web GUI](docs/sim_web_gui.png)

---

## Architecture Overview
This architecture represents a **multi-robot task distribution system** with the following key components:

1. **Web Server**: Handles client orders via a GUI and interacts with Shared Memory for data access.
2. **Task Manager**: Processes order requests, queries the database, and uses a task allocation algorithm to assign tasks based on robot proximity, battery, and availability.
3. **Fleet Manager**: Manages robot statuses, aggregates fleet status, and assigns tasks to robots via a task assignment service.
4. **Robot Fleet**: Consists of multiple robots that navigate, pick up, and drop off parcels while reporting their status to the Fleet Manager.
5. **Shared Memory**: Stores and updates inventory, robot locations, and other critical data.
6. **Database**: Maintains shelf locations, capacities, inventory, robot locations, and order history.
7. **Central Logger**: Logs activities from the Task Manager, Fleet Manager, and Shared Memory for monitoring and debugging.

The system ensures efficient task distribution, real-time fleet monitoring, and seamless coordination between robots, tasks, and inventory management.

![Software Architecture](docs/MRTD_arch4.png)

---

## Node Documentation

### 1. **Task Manager**

The **Task Manager** is responsible for receiving order details from the Web Server and assigning tasks to robots based on a Task Allocation Algorithm. It ensures efficient task distribution by considering the following parameters:
- **Availability**: Only idle robots are considered for task allocation.
- **Proximity**: Distance of the robot from the target shelf.
- **Battery Level**: Ensures robots with sufficient battery are prioritized.

#### Workflow:
1. Receives order details from the Web Server via a ROS2 service.
2. Evaluates all available robots using the Task Allocation Algorithm.
3. Assigns the task to the robot with the best score.
4. Sends the task details to the **Fleet Manager** for execution.

#### Communication:
- **Input**: Order details from the Web Server (via ROS2 service).
- **Output**: Task assignment to the Fleet Manager (via ROS2 service).

---

### 2. **Fleet Manager**

The **Fleet Manager** oversees the status of all robots and ensures smooth execution of assigned tasks. It acts as the intermediary between the Task Manager and the robots' navigation system.

#### Responsibilities:
1. Maintains the status of each robot (idle, busy, etc.).
2. Processes task assignments received from the Task Manager.
3. Sends navigation goals to the robots using the **Navigation2 Action Server**.
4. Publishes the combined fleet status for monitoring purposes.

#### Workflow:
1. Receives task details from the Task Manager.
2. Sends navigation goals to the robot assigned to the task.
3. Monitors task progress and updates the Task Manager upon completion.

#### Communication:
- **Input**: Task details from the Task Manager (via ROS2 service).
- **Output**: Navigation goals to robots (via Navigation2 Action Server).
- **Output**: Fleet status updates (via a ROS2 topic, `/fleet_status`).

---

### 3. **Central Logger**

The **Central Logger** is responsible for collecting, storing critical and forwarding logs from various nodes. These logs are essential for debugging, monitoring, and visualization on the Web Server.

#### Responsibilities:
1. Subscribes to log messages from different modules, including:
   - **Shared Memory Module**: Logs inventory updates and robot activities.
   - **Task Manager**: Logs task allocation and scheduling details.
   - **Fleet Manager**: Logs fleet status updates and task execution progress.
2. Stores logs in a centralized database.
3. Sends logs to the Web Server for real-time visualization and monitoring.

#### Communication:
- **Input**: Logs from Shared Memory Module, Task Manager, and Fleet Manager (via ROS2 topics).
- **Output**: Logs to a csv file and forwarded to the Web Server via an HTTP request for remote access.

---

### 4. **Shared Memory Module**

The **Shared Memory Module** acts as a centralized database for managing inventory and robot activity logs. It ensures real-time updates and synchronization across the system.

#### Responsibilities:
1. Maintains the inventory database, including item locations and quantities.
2. Logs robot activities, such as task assignments and completions.
3. Provides real-time updates to other nodes, such as the Task Manager and Fleet Manager.

#### Communication:
- **Input**: Reads the updates the database from files, and makes it available for other nodes.
- **Input**: Updates from robots and other nodes (via ROS2 topics).
- **Input**: Order details from Web server (via ROS2 topics).
- **Output**: Inventory and activity logs to the Central Logger and other nodes (via ROS2 topics).

---

### 5. **Web Server**

The **Web Server** provides a graphical user interface (GUI) for monitoring and managing the system. It allows users to:
- Place orders and view their status.
- Monitor the inventory and robot fleet status in real-time.
- Visualize logs and system performance for debugging.

#### Responsibilities:
1. Processes user-submitted orders and sends them to the Task Manager for execution
2. Fetches and displays inventory data to track product availability.
3. Retrieves and updates fleet status to monitor robot activity.
4. Receives logs from the Central Logger and presents them for analysis.

#### Communication:
- **Input**: Inventory and Fleet status (via API and ROS2 topics).
- **Input**: Logs from the log file updated by Central logger.
- **Output**: Order requests sent to the Task Manager via a ROS2 service.

---

### Node Interaction Overview

The following diagram summarizes the interaction between the nodes:

1. **Web Server** sends order details to the **Task Manager**.
2. **Task Manager** evaluates robots and assigns tasks to the **Fleet Manager**.
3. **Fleet Manager** sends navigation goals to robots and updates the **Task Manager** on task progress.
4. **Shared Memory Module** logs robot activities and inventory updates.
5. **Central Logger** collects logs from all nodes and provides them to the **Web Server** for visualization.

---

### Example Workflow

1. A user places an order via the Web Server.
2. The Web Server sends the order details to the Task Manager.
3. The Task Manager evaluates available robots and assigns the task to the most suitable one.
4. The Fleet Manager receives the task and sends navigation goals to the assigned robot.
5. The robot executes the task and updates the Fleet Manager on its progress.
6. The Fleet Manager informs the Task Manager upon task completion.
7. Logs from all nodes are collected by the Central Logger and displayed on the Web Server.

---

## Future Enhancements

1. **Advanced Load Balancing**: Implement AI-based algorithms for task distribution.
2. **Real-World Deployment**: Adapt the system for real-world warehouse robots.
3. **Enhanced Simulation**: Add more complex warehouse layouts and scenarios.
4. **Fault Tolerance**: Improve recovery mechanisms for node failures.
5. **Dynamic Robot allocation**: Dynamically add or remove the robots from fleet without any downtime.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---
