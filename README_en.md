# Qt6 ROS2 Template

[中文](README.md) | [English](README_en.md)

## Table of Contents

- [Project Introduction](#project-introduction)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation Guide](#installation-guide)
- [Project Setup](#project-setup)
- [Running the Project](#running-the-project)
- [Project Structure](#project-structure)
- [Customization and Extension](#customization-and-extension)
- [Frequently Asked Questions](#frequently-asked-questions)
- [Contribution Guidelines](#contribution-guidelines)
- [Version History](#version-history)
- [License](#license)
- [Contact Information](#contact-information)

## Project Introduction

**Qt6 ROS2 Template** is a ROS2 human-machine interface template built on Qt6. This project aims to provide developers with a quick-start framework for developing ROS2 human-machine interfaces. It combines Qt6's modern UI design capabilities with ROS2's powerful robot development ecosystem, offering an ideal starting point for graphical interface development in robotic applications.

This project draws inspiration from earlier ROS1 and Qt5 projects. However, it has made significant improvements:

1. **Dual Compilation Support**: This template not only supports compilation in the ROS2 environment using `colcon build`, but can also be opened and compiled directly in Qt Creator without complex configuration steps.

2. **Simplified Development Process**: This project achieves an "out-of-the-box" experience, greatly simplifying the development process.

3. **Enhanced Flexibility**: Developers can choose to use either the ROS2 toolchain or Qt Creator for development according to their preferences, providing greater flexibility.

## Features

- Integration of Qt6 and ROS2, providing modern UI design capabilities and powerful robot development capabilities
- Dual compilation support: compatible with ROS2's `colcon build` and Qt Creator's build system
- No complex configuration required, projects can be opened directly in Qt Creator for development
- Pre-configured CMakeLists.txt, simplifying the integration process of Qt and ROS2
- Includes basic ROS2 node and Qt main window implementation as a development starting point
- Provides ROS2 launch files for easy node startup and management
- Supports compilation and running in both ROS2 and non-ROS environments, enhancing project flexibility

## System Requirements

- Operating System: Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS Version: ROS2 Humble
- Qt Version: 6.x
- CMake Version: 3.16 or higher

## Installation Guide

### Installing ROS2 Humble

Follow the official ROS2 Humble installation instructions:
[ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### Installing Qt6

It is recommended to install Qt6 using the official Qt installer:

1. Download Qt Installer:
   Visit the [Qt official download page](https://www.qt.io/download-qt-installer) and download the online installer for Linux.

2. Grant execution permissions and run the installer:

   ```bash
   chmod +x qt-unified-linux-x64-4.x.x-online.run
   ./qt-unified-linux-x64-4.x.x-online.run
   ```

3. During installation, make sure to select Qt 6.x and Qt Creator.

### Installing Other Dependencies

If you encounter issues, you may need to install additional dependencies:

```bash
sudo apt install libxcb-xinerama0 libxcb-xinerama0-dev libxcb-cursor0
```

## Project Setup

### Creating a Workspace

```bash
mkdir -p ~/qt6_ros2_template_ws/src
cd ~/qt6_ros2_template_ws/src
```

### Cloning the Project

```bash
git clone https://github.com/XKHoshizora/qt6_ros2_template.git
cd qt6_ros2_template
```

### Building the Project

1. Using colcon (ROS2 environment):

   ```bash
   cd ~/qt6_ros2_template_ws
   colcon build --symlink-install
   ```

2. Using Qt Creator:
   - Open Qt Creator
   - Select "File" > "Open File or Project"
   - Navigate to `~/qt6_ros2_template_ws/src/qt6_ros2_template` and select the `CMakeLists.txt` file
   - Configure the project and click "Build"

## Running the Project

1. Running in the ROS2 environment:

   ```bash
   source ~/qt6_ros2_template_ws/install/setup.bash
   ros2 launch qt6_ros2_template demo.launch.py
   ```

2. Running in Qt Creator:
   - Open the project in Qt Creator
   - Click the "Run" button or press Ctrl+R

## Project Structure

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

## Customization and Extension

1. Modify `src/mainwindow.cpp` and `ui/mainwindow.ui` to customize the UI.
2. Add ROS2 functionality in `src/qt6_ros2_template_node.cpp`.
3. Update `CMakeLists.txt` to include new source files or dependencies.

## Frequently Asked Questions

1. **Issue**: "Qt6 not found" error during compilation.
   **Solution**: Ensure Qt6 is correctly installed and CMAKE_PREFIX_PATH includes the Qt6 installation path.

2. **Issue**: ROS2-related functions are undefined.
   **Solution**: Make sure you have sourced the ROS2 setup file and all necessary dependencies are included in package.xml.

3. **Issue**: Unable to find ROS2 header files in Qt Creator.
   **Solution**: Add the ROS2 include path (usually /opt/ros/humble/include) in the project settings of Qt Creator.

## Contribution Guidelines

We welcome and appreciate any form of contribution! If you want to contribute to the project, please follow these steps:

1. Fork this repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Create a new Pull Request

## Version History

- 0.1.0
  - Initial version
  - Basic integration of Qt6 and ROS2 Humble
  - Implemented dual compilation support (ROS2 environment and Qt Creator)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Contact Information

Project Maintainer: Hoshizora - hoshizoranihon@gmail.com

Project Link: [https://github.com/XKHoshizora/qt6_ros2_template](https://github.com/XKHoshizora/qt6_ros2_template)
