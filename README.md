# XJU ROBOT

![teaser](figures/teaser.gif)

XJU-ROBOT is an open-source project based on ROS (Robot Operating System), providing a **simulation** environment for various heterogeneous robots. It integrates various algorithms such as **pnc**, **slam**, **perception**, etc., and can be used for relevant practitioners to study and develop.

**Table of Contents**
- [Installation](#installation)
- [Getting Started](#getting-started)
- [Video](#video)
- [License](#license)
- [Citation](#citation)

## Installation

[Install ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

---

```bash
# Download code
git clone https://github.com/Mr-Tony921/xju-robot.git
git submodule init && git submodule update
# Install dependencies
bash install_dependencies.sh
```

## Getting Started

```bash
# Basic example of how to build and run
# Open one terminal
cd xju-robot && source devel/setup.bash
roslaunch xju_simu simple_world.launch
# Open another terminal
roslaunch xju_pnc move_base_flex.launch
```

## Video

A series of instructional videos detailing the usage of XJU-ROBOT is available. The provided link directs to the initial episode, from which viewers can access subsequent episodes.

[XJU移动机器人仿真-第1期 环境搭建（gazebo模型 插件 urdf rviz）](https://www.bilibili.com/video/BV1be4y1z7cr/?share_source=copy_web)

## License

All resources in XJU-ROBOT are licensed under fully permissive licenses (e.g., Apache-2.0).

## Citation

If you use XJU-ROBOT, consider citing the following publication:

```
@inproceedings{tony2024xju-robot,
  title={xju-robot: A Robot Simulation Framework},
  author={Tony},
  url={https://github.com/Mr-Tony921/xju-robot.git},
  year={2024}
}
```
