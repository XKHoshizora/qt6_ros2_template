# Qt6 ROS2 Template

[中文](README.md) | [English](README_en.md)

## 目录

- [项目简介](#项目简介)
- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [安装指南](#安装指南)
- [项目设置](#项目设置)
- [运行项目](#运行项目)
- [项目结构](#项目结构)
- [自定义和扩展](#自定义和扩展)
- [常见问题](#常见问题)
- [贡献指南](#贡献指南)
- [版本历史](#版本历史)
- [许可证](#许可证)
- [联系方式](#联系方式)

## 项目简介

**Qt6 ROS2 Template** 是一个基于 Qt6 构建的 ROS2 人机交互界面模板。本项目旨在为开发者提供一个快速入门的框架，方便用户在此基础上进行 ROS2 人机交互界面的开发。它融合了 Qt6 的现代化 UI 设计能力和 ROS2 的强大机器人开发生态，为机器人应用的图形界面开发提供了一个理想的起点。

本项目的开发灵感来自于早期的 ROS1 和 Qt5 项目。然而，本项目进行了重大改进：

1. **双重编译支持**：本模板不仅支持在 ROS2 环境下使用 `colcon build` 进行编译，还可以直接在 Qt Creator 中打开和编译，无需复杂的配置步骤。

2. **简化开发流程**：本项目实现了"开箱即用"的体验，大大简化了开发流程。

3. **增强的灵活性**：开发者可以根据自己的偏好选择使用 ROS2 工具链或 Qt Creator 进行开发，提供了更大的灵活性。

## 功能特性

- 集成 Qt6 和 ROS2，提供现代化的 UI 设计和强大的机器人开发能力
- 双重编译支持：兼容 ROS2 的 `colcon build` 和 Qt Creator 的构建系统
- 无需复杂配置，可直接在 Qt Creator 中打开项目进行开发
- 预配置的 CMakeLists.txt，简化了 Qt 和 ROS2 的集成过程
- 包含基本的 ROS2 节点和 Qt 主窗口实现，可作为开发起点
- 提供了 ROS2 launch 文件，方便启动和管理节点
- 支持在 ROS2 和非 ROS 环境下编译运行，增强了项目的灵活性

## 系统要求

- 操作系统：Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS 版本：ROS2 Humble
- Qt 版本：6.x
- CMake 版本：3.16 或更高

## 安装指南

### 安装 ROS2 Humble

请按照 ROS2 Humble 的官方安装指南进行安装：
[ROS2 Humble 安装指南](https://docs.ros.org/en/humble/Installation.html)

### 安装 Qt6

推荐使用 Qt 官方安装器安装 Qt6：

1. 下载 Qt Installer：
   访问 [Qt 官方下载页面](https://www.qt.io/download-qt-installer)，下载适用于 Linux 的在线安装程序。

2. 授予执行权限并运行安装程序：

   ```bash
   chmod +x qt-unified-linux-x64-4.x.x-online.run
   ./qt-unified-linux-x64-4.x.x-online.run
   ```

3. 在安装过程中，确保选择 Qt 6.x 和 Qt Creator。

### 安装其他依赖

如果遇到问题，可能需要安装以下依赖：

```bash
sudo apt install libxcb-xinerama0 libxcb-xinerama0-dev libxcb-cursor0
```

## 项目设置

### 创建工作空间

```bash
mkdir -p ~/qt6_ros2_template_ws/src
cd ~/qt6_ros2_template_ws/src
```

### 克隆项目

```bash
git clone https://github.com/XKHoshizora/qt6_ros2_template.git
cd qt6_ros2_template
```

### 构建项目

1. 使用 colcon（ROS2 环境）：

   ```bash
   cd ~/qt6_ros2_template_ws
   colcon build --symlink-install
   ```

2. 使用 Qt Creator：
   - 打开 Qt Creator
   - 选择 "File" > "Open File or Project"
   - 导航到 `~/qt6_ros2_template_ws/src/qt6_ros2_template` 并选择 `CMakeLists.txt` 文件
   - 配置项目并点击 "Build"

## 运行项目

1. 在 ROS2 环境中运行：

   ```bash
   source ~/qt6_ros2_template_ws/install/setup.bash
   ros2 launch qt6_ros2_template demo.launch.py
   ```

2. 在 Qt Creator 中运行：
   - 在 Qt Creator 中打开项目
   - 点击 "Run" 按钮或按 Ctrl+R

## 项目结构

```
qt6_ros2_template/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── qt6_ros2_template/
│   └── mainwindow.h
├── msg/
├── resources/
│   ├── images/
│   │   └── icon.png
│   └── resource.qrc
├── scripts/
├── src/
│   ├── main.cpp
│   ├── mainwindow.cpp
│   └── qt6_ros2_template_node.cpp
├── srv/
├── ui/
│   └── mainwindow.ui
└── launch/
    └── demo.launch.py
```

## 自定义和扩展

1. 修改 `src/mainwindow.cpp` 和 `ui/mainwindow.ui` 以自定义 UI。
2. 在 `src/qt6_ros2_template_node.cpp` 中添加 ROS2 功能。
3. 更新 `CMakeLists.txt` 以包含新的源文件或依赖项。

## 常见问题

1. **问题**：编译时出现 "Qt6 not found" 错误。
   **解决方案**：确保 Qt6 正确安装，并且 CMAKE_PREFIX_PATH 包含 Qt6 安装路径。

2. **问题**：ROS2 相关的函数未定义。
   **解决方案**：确保已 source ROS2 的 setup 文件，并且 package.xml 中包含所有必要的依赖。

3. **问题**：在 Qt Creator 中无法找到 ROS2 头文件。
   **解决方案**：在 Qt Creator 的项目设置中，添加 ROS2 的 include 路径（通常在 /opt/ros/humble/include）。

## 贡献指南

我们欢迎并感谢任何形式的贡献！如果您想为项目做出贡献，请遵循以下步骤：

1. Fork 本仓库
2. 创建您的特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交您的更改 (`git commit -m 'Add some AmazingFeature'`)
4. 将您的更改推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建一个新的 Pull Request

## 版本历史

- 0.1.0
  - 初始版本
  - 基本的 Qt6 和 ROS2 Humble 集成
  - 实现了双重编译支持（ROS2 环境和 Qt Creator）

## 许可证

本项目基于 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

## 联系方式

项目维护者：Hoshizora - hoshizoranihon@gmail.com.com

项目链接：[https://github.com/XKHoshizora/qt6_ros2_template](https://github.com/XKHoshizora/qt6_ros2_template)
