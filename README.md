![CICD Workflow status](https://github.com/Prathinav-kV/Martian-Gatherer/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/Prathinav-kV/Martian-Gatherer/graph/badge.svg?token=5IIV6EXEEV)](https://codecov.io/gh/Prathinav-kV/Martian-Gatherer)[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

# Martian Gatherer

## **Project Overview**

The Martian Gatherer is a **ROS2-based simulation** that allows a robot to autonomously navigate a world to collect and manipulate objects. The project integrates **Gazebo simulation**, **TurtleBot3 navigation**, and **ROS2 action and service communication**. The system is designed to demonstrate **autonomous navigation**, **goal setting**, and **object manipulation**. The key nodes responsible for the operation of this system are **nav2goal**, **spawn_box**, and **animated_box**.

---

## AIP Product Backlog and Sprint Planning 
- https://docs.google.com/spreadsheets/d/1JDAhqKNzfQw_SQy7KT3hBV-zDd4FMuCs95LgB8wYdUY/edit?gid=0#gid=0 -- Sheet Link for Product Backlog

- https://docs.google.com/document/d/1xY_Vxz4sa3DS9jx3DZNXnuZeMRPlwxqR0DCihDlczrA/edit?tab=t.0 -- Sprint Planning Doc



## **Project Requirements**

### **Operating System**

- **Ubuntu 22.04** (required for ROS2 Humble compatibility)

### **Libraries and Tools**

- **ROS2 Humble**: Core framework for robot control and communication.
- **Gazebo**: 3D simulator for robot environments.
- **TurtleBot3 Navigation2**: Package to manage the robot's navigation.
- **LCOV**: Used for code coverage tracking and reports.
- **Colcon**: Build tool for ROS2.
- **GTest**: Used for unit testing.
- **Pytest**: Used for integration and launch testing.
- **Rviz2**: Visualization tool for robot state visualization.

---

## **Project Structure**

```
/home/pratkv/Martian-Gatherer/src
├── mars_interfaces
│   ├── CMakeLists.txt
│   ├── include
│   │   └── mars_interfaces
│   ├── package.xml
│   ├── src
│   └── srv
│       └── SendCoordinates.srv
└── martian_gatherer
    ├── CMakeLists.txt
    ├── include
    ├── launch
    │   ├── robot_state_publisher.launch.py
    │   ├── spawn_turtlebot3.launch.py
    │   └── world.launch.py
    ├── map
    │   ├── my_map.pgm
    │   └── my_map.yaml
    ├── models
    │   ├── simple_box
    │   │   ├── model.config
    │   │   └── model.sdf
    │   ├── turtlebot3_common
    │   │   ├── meshes
    │   │   │   ├── burger_base.dae
    │   │   │   ├── lds.dae
    │   │   │   ├── r200.dae
    │   │   │   ├── tire.dae
    │   │   │   ├── waffle_base.dae
    │   │   │   └── waffle_pi_base.dae
    │   │   └── model.config
    │   ├── turtlebot3_waffle
    │   │   ├── model-1_4.sdf
    │   │   ├── model.config
    │   │   └── model.sdf
    │   └── turtlebot3_world
    │       ├── meshes
    │       │   ├── hexagon.dae
    │       │   └── wall.dae
    │       ├── model-1_4.sdf
    │       ├── model.config
    │       └── model.sdf
    ├── package.xml
    ├── src
    │   ├── animated_box.cpp
    │   ├── nav2goal.cpp
    │   └── spawn_box.cpp
    ├── srv
    │   └── SendCoordinates.srv
    ├── test
    ├── UML
    │   ├── Initial
    │   │   ├── activity_diagram.jpeg
    │   │   └── class_diagram.jpeg
    │   └── Revised
    │       ├── activity diagram
    │       ├── Nav2goal.pdf
    │       └── Spawn_box_diagram.pdf
    ├── urdf
    │   ├── common_properties.urdf
    │   └── turtlebot3_waffle.urdf
    └── worlds
        └── my_world_2.world
```

---

## **How the Project Runs**

To run the system, follow the following sequence in **four terminal windows** (each command must be run in a separate terminal window):

**1. Launch the World**

```bash
ros2 launch martian_gatherer world.launch.py
```

This launches the Gazebo world and spawns the environment.

**2. Launch Navigation2 for TurtleBot3**

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/pratkv/Martian-Gatherer/src/martian_gatherer/map/my_map.yaml
```

This command launches the **TurtleBot3 navigation stack** with the required map and sets `use_sim_time` to True. **Note:** The path to the map (`/home/pratkv/Martian-Gatherer/src/martian_gatherer/map/my_map.yaml`) contains the username `pratkv`. If you are using a different system, you will need to replace `pratkv` with your own username to correctly point to the map file.

**3. Run nav2goal Node**

```bash
ros2 run martian_gatherer nav2goal
```

This launches the **nav2goal** node, which allows the robot to navigate to the goal position set by the `spawn_box` node.

**4. Run spawn_box Node**

```bash
ros2 run martian_gatherer spawn_box
```

This launches the **spawn_box** node, which spawns a box at a random, valid position within the Gazebo world and sends its coordinates to the **nav2goal** node.

---

## **Node Descriptions**

### **1. nav2goal**

- **File:** `src/nav2goal.cpp`
- **Role:** Handles navigation of the robot to reach a specific goal point. It receives (x, y, z) coordinates from the **spawn_box** node and sends navigation goals to the TurtleBot3 using the Navigation2 action server.
- **Key Functions:**
  - **set_initial_pose()**: Sets the robot’s initial position to `(0, 0, 0)` with zero orientation.
  - **set_goal_pose(x, y)**: Calculates the robot's goal position and orientation to face the target point using `atan2`.
  - **send_goal()**: Sends the goal position to the Navigation2 action server.
  - **result_callback()**: Checks if the goal has been reached successfully.
  - **execute_next_task()**: Requests Gazebo to delete the box after the goal is reached.
- **Dependencies:** ROS2 action client (nav2_msgs) and ROS2 service client to communicate with the Navigation2 action server and delete Gazebo models.

---

### **2. spawn_box**

- **File:** `src/spawn_box.cpp`
- **Role:** Spawns a box at a random, valid position in the Gazebo world and sends its coordinates to the **nav2goal** node.
- **Key Functions:**
  - **spawn_box()**: Randomly generates (x, y) coordinates and spawns the box in Gazebo using the `/spawn_entity` service.
  - **send_coordinates()**: Sends the (x, y, z) coordinates of the box to the **nav2goal** node via the `/send_coordinates` service.
  - **is_valid_spawn(x, y)**: Ensures that the (x, y) position is valid and not inside any obstacle or other restricted areas.
- **Dependencies:** Uses ROS2 services to spawn entities in Gazebo.

---

