# Qt6 ROS2 Template

[中文](README.md) | [English](README_en.md)

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation Guide](#installation-guide)
- [Project Setup](#project-setup)
- [Running the Project](#running-the-project)
- [Project Structure](#project-structure)
- [Customization and Extension](#customization-and-extension)
- [FAQ](#faq)
- [Contributing](#contributing)
- [Version History](#version-history)
- [License](#license)
- [Contact](#contact)

## Project Overview

**Qt6 ROS2 Template** is a human-machine interface template for ROS2 built on Qt6. This project aims to provide developers with a quick-start framework for developing ROS2 human-machine interaction interfaces. It combines Qt6's modern UI design capabilities with ROS2's powerful robot development ecosystem, offering an ideal starting point for developing graphical interfaces for robotic applications.

This project was inspired by earlier ROS1 and Qt5 projects. However, this project has made significant improvements:

1. **Dual Compilation Support**: This template supports compilation using `colcon build` in a ROS2 environment and can also be opened and compiled directly in Qt Creator without complex configuration steps.

2. **Simplified Development Process**: This project achieves an "out-of-the-box" experience, greatly simplifying the development process.

3. **Enhanced Flexibility**: Developers can choose to use either the ROS2 toolchain or Qt Creator for development according to their preferences, providing greater flexibility.

## Features

- Integration of Qt6 and ROS2, providing modern UI design and powerful robot development capabilities
- Dual compilation support: compatible with ROS2's `colcon build` and Qt Creator's build system
  - Can be compiled using `colcon build` in an environment with only ROS2
  - Can be compiled and opened in Qt Creator in an environment with only Qt6
  - Can be compiled using either `colcon build` or Qt Creator in an environment with both ROS2 and Qt6
- Can be opened directly in Qt Creator for development without complex configuration
- Pre-configured CMakeLists.txt, simplifying the integration process of Qt and ROS2
- Includes basic ROS2 node and Qt main window implementation as a starting point for development
- Provides ROS2 launch file for easy node startup and management
- Supports compilation and running in both ROS2 and non-ROS environments, enhancing project flexibility

## System Requirements

- Operating System: Ubuntu 22.04 LTS (Jammy Jellyfish) or higher
- ROS Version: ROS2 Humble or higher
- Qt Version: 6.x
- CMake Version: 3.16 or higher

## Installation Guide

### Installing ROS2

1. Using Auto ROS Installer (recommended):

   ```bash
   git clone https://github.com/XKHoshizora/auto-ros-installer.git
   cd auto-ros-installer
   ```

   For detailed installation instructions, please refer to the [project homepage](https://github.com/XKHoshizora/auto-ros-installer).

2. Follow the official ROS2 installation guide (for Humble):
   [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### Installing Qt6

It is recommended to use the official Qt installer to install Qt6:

1. Download Qt Installer:
   Visit the [Qt official download page](https://www.qt.io/download-qt-installer) and download the online installer for Linux.

2. Grant execute permission:

   ```bash
   chmod +x qt-online-installer-linux-<x64/arm64>-<version>.run
   ```

3. Run the installer:

   ```bash
   ./qt-online-installer-linux-<x64/arm64>-<version>.run
   ```

4. Configure Qt Creator shortcut:
   Create and open the `qtcreator` file using the following command:

   ```bash
   sudo nano /usr/bin/qtcreator
   ```

   Add the following content to the `qtcreator` file:

   ```shell
   #!/bin/sh

   export QT_HOME=/home/<user>/Qt/Tools/QtCreator/bin
   $QT_HOME/qtcreator $*
   ```

5. Add executable permission to the `qtcreator` file:

   ```bash
   sudo chmod a+x /usr/bin/qtcreator
   ```

6. Launch Qt Creator:

   ```bash
   qtcreator
   ```

7. Install other dependencies:

   If you encounter issues when running Qt Creator, please install the following dependencies:

   ```bash
   sudo apt install libxcb-xinerama0 libxcb-xinerama0-dev libxcb-cursor0
   ```

## Project Setup

### Creating a Workspace

```bash
mkdir -p ~/qt6_ws/src
cd ~/qt6_ws/src
```

### Cloning the Project

```bash
git clone https://github.com/XKHoshizora/qt6_ros2_template.git
```

### Building the Project

#### In a ROS2-only Environment

Execute the following commands in your workspace to compile the package:

```bash
cd ~/qt6_ws
colcon build --symlink-install
```

#### In a Qt6-only Environment

1. Open Qt Creator
2. Select "File" > "Open File or Project"
3. Navigate to the `~/qt6_ws/src/qt6_ros2_template` directory, select the `CMakeLists.txt` file and open it
4. Configure and open the project. If the project files are displayed normally on the left side, it indicates successful compilation. If compilation fails, only a `CMakeLists.txt` file will be displayed.

#### In an Environment with both ROS2 and Qt6

There are two compilation methods:

- Execute the following commands in your workspace to compile the package:

  ```bash
  cd ~/qt6_ws
  colcon build --symlink-install
  ```

- Compile by opening the project in Qt Creator, following these steps:
  1. Open Qt Creator
  2. Select "File" > "Open File or Project"
  3. Navigate to the `~/qt6_ws/src/qt6_ros2_template` directory, select the `CMakeLists.txt` file and open it
  4. Configure and open the project. If the project files are displayed normally on the left side, it indicates successful compilation. If compilation fails, only a `CMakeLists.txt` file will be displayed.

## Running the Project

### In a ROS2-only Environment

First, ensure that you have completed the compilation according to the steps above. Then execute the following commands in the workspace:

```bash
source ~/qt6_ws/install/setup.bash
ros2 launch qt6_ros2_template demo.launch.py
```

### In a Qt6-only Environment

After compiling and opening the project through Qt Creator, click the green `Run` button on the left side or use the `Ctrl + R` shortcut to run.

### In an Environment with both ROS2 and Qt6

First, ensure that you have completed the compilation according to the steps above. Then execute the following commands in the workspace:

```bash
source ~/qt6_ws/install/setup.bash
ros2 launch qt6_ros2_template demo.launch.py
```

Or, after compiling and opening the project through Qt Creator, click the green `Run` button on the left side or use the `Ctrl + R` shortcut to run.

## Project Structure

This project uses the standard CMake build system and follows the directory structure of Qt projects.

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

## FAQ

1. **Issue**: "Qt6 not found" error during compilation.
   **Solution**: Ensure Qt6 is correctly installed and CMAKE_PREFIX_PATH includes the Qt6 installation path.

2. **Issue**: ROS2-related functions are undefined.
   **Solution**: Make sure you have sourced the ROS2 setup file and all necessary dependencies are included in package.xml.

3. **Issue**: Unable to find ROS2 headers in Qt Creator.
   **Solution**: Add ROS2 include paths (usually in /opt/ros/humble/include) to the project settings in Qt Creator.

## Contributing

We welcome and appreciate any form of contribution! If you want to contribute to the project, please follow these steps:

1. Fork this repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Create a new Pull Request

## Version History

- 0.1.0
  - Initial release
  - Basic Qt6 and ROS2 integration
  - Implemented dual compilation support (ROS2 environment and Qt Creator)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Contact

Project Maintainer: Hoshizora - hoshizoranihon@gmail.com

Project Link: [https://github.com/XKHoshizora/qt6_ros2_template](https://github.com/XKHoshizora/qt6_ros2_template)

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=XKHoshizora/qt6_ros2_template&type=Date)](https://star-history.com/#XKHoshizora/qt6_ros2_template&Date)
