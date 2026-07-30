# FineNav-Engine-Robot-Test

本仓库用于放置 FineNav 机器人实机测试的相关代码与配置，基于 [FineNav-Engine](https://github.com/FineNav/FineNav-Engine) 导航框架。

## 目录结构

```
finenav_robot_bringup/
├── behavior_trees/   # 行为树 XML 配置
├── config/           # 机器人参数配置 (robot_params.yaml)
├── include/          # 头文件 (MapView 适配器等)
├── launch/           # ROS 2 launch 文件
├── src/              # 源代码 (main.cpp, MapView 实现等)
├── CMakeLists.txt
└── package.xml
```

## 分支说明

| 分支 | 用途 |
|---|---|
| `main` | 仓库说明与文档 |
| `test/FineNav-202607` | 2026年7月 FineNav 实机测试代码 |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- FineNav-Engine (需提前编译并 source)

## 快速开始

```bash
cd ~/FineNav_Engine_Test_ws
colcon build --symlink-install --packages-select finenav_robot_bringup
source install/setup.bash
ros2 launch finenav_robot_bringup robot_bringup.launch.py
```

## 相关仓库

- [FineNav-Engine](https://github.com/FineNav/FineNav-Engine) — 导航引擎主仓库
