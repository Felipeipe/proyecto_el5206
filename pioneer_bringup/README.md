# Bender Bringup

## Table of Contents

- [Bender Bringup](#bender-bringup)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Overview](#overview)
  - [Requirements](#requirements)
  - [Package Structure](#package-structure)
  - [Launch Instructions](#launch-instructions)
    - [Start the robot](#start-the-robot)
      - [Basic setup](#basic-setup)
      - [Full system](#full-system)
      - [Slam Mode](#slam-mode)
      - [Localization only](#localization-only)
      - [Navigation only](#navigation-only)
    - [Maps and Parameters](#maps-and-parameters)
    - [Notes](#notes)

## Introduction

This package provides the launch configurations, parameter files, and map resources required to bring up the **Bender robot** on **ROS 2 Jazzy**.  
It integrates the robot’s localization, navigation, and visualization systems into a unified launch structure.

---

## Overview

`bender_bringup` is responsible for starting all core subsystems of the robot, including:

- **Localization** (AMCL, SLAM Toolbox, or Cartographer)
- **Navigation** (Nav2 stack)
- **Visualization** (RViz configuration)
- **Robot bringup** (base, sensors, and full setup)

The package is structured to support different operation modes such as:

- Mapping and localization
- Full navigation with existing maps
- SLAM operation for exploration and map creation

---

## Requirements

- **ROS 2 Jazzy Jalisco**
- **Nav2** (`nav2_bringup`, `nav2_msgs`, etc.)
- **SLAM Toolbox** or **Cartographer**
- **Robot-specific packages** (`bender_description`, `bender_bringup`, `bender_bringup_interfaces`, etc.)

Remember that even though you can build this package and all its necessary dependencies directly in your pc it is **highly** recommended that you use the docker containers that are located in [uchile_system](https://github.com/uchile-robotics/uchile_system). In that package there are some instructions on how to install it properly (In that docker container you can plug-and-play)

Make sure all dependencies are properly built in your workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Package Structure

```bash
bender_bringup/
├── launch/
│   ├── localization/
│   │   ├── bender_cartographer.launch.py
│   │   ├── localization_launch.py
│   │   └── slam_toolbox.launch.py
│   ├── navigation/
│   │   └── navigation_launch.py
│   ├── robot/
│   │   ├── bender_basic.launch.py
│   │   ├── bender_full.launch.py
│   │   └── bender_slam.launch.py
│   └── tools/rviz/bender.rviz
├── maps/
│   ├── env_30_may_2025/
│   ├── quimica/
│   └── stage_5_may_2025/
├── params/
│   ├── localization/
│   ├── maps/
│   └── navigation/
├── CMakeLists.txt
├── package.xml
└── README.md
```

| Folder                | Description                                                       |
| --------------------- | ----------------------------------------------------------------- |
| `launch/localization` | Launch files for localization and SLAM modes                      |
| `launch/navigation`   | Launch for the Nav2 navigation stack                              |
| `launch/robot`        | Core bringup launch files for base, sensors, and full robot setup |
| `launch/tools/rviz`   | RViz configuration for visualization                              |
| `maps`                | Predefined maps with `.pgm` and `.yaml` pairs                     |
| `params`              | Parameter files for localization, navigation, and maps            |

## Launch Instructions

### Start the robot

#### Basic setup

Launch file that launches the base driver, LiDaR sensor and joystick.

```bash
ros2 launch bender_bringup bender_basic.launch.py
```

#### Full system

Launches all core nodes: localization, navigation and the `bender_basic.launch.py` launch file

```bash
ros2 launch bender_bringup bender_full.launch.py
```

#### Slam Mode

Launches the robot in SLAM (Simultaneous Localization and Mapping) mode to build a map of the environment

```bash
ros2 launch bender_bringup bender_slam.launch.py
```

#### Localization only

Useful for testing AMCL or SLAM Toolbox independently

```bash
ros2 launch bender_bringup localization_launch.py
```

#### Navigation only

Starts the Nav2 stack using the selected map and parameter configuration.

```bash
ros2 launch bender_bringup navigation_launch.py
```

### Maps and Parameters

All parameters of interest are located in the [`params`](./params) folder inside this repository. Navigation parameters can be tuned on [`params/navigation/nav2_params.yaml`](params/navigation/nav2_params.yaml). Maps are stored in [`params/maps/maps.yaml`](params/maps/maps.yaml) and selected via the configuration file.

### Notes

- This package assumes the robot’s base and sensors are already available via their respective drivers.

- The launch files are modular, allowing you to include them in higher-level launch setups if needed (e.g., navigation-only mode, mapping mode, etc.).

- To integrate with your own robot description, ensure TF tree and topic names match those expected by Nav2 and AMCL.
