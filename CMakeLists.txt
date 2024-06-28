# 指定 CMake 的最低版本要求
# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.8)

# 设置项目名称
# Set the project name
project(qt6_ros2_template)

# 设置 C++ 标准
# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
# Set compilation options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 检查是否在 ROS 环境中
# Check if we are in a ROS environment
if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "ROS environment detected: $ENV{ROS_DISTRO}")
    set(USE_ROS TRUE)
else()
    message(STATUS "ROS environment not detected. Building without ROS support.")
    set(USE_ROS FALSE)
endif()

# 尝试查找 Qt6
# Try to find Qt6
find_package(Qt6 COMPONENTS Widgets QUIET)

if(Qt6_FOUND)
    message(STATUS "Qt6 found. Building with Qt support.")
    set(USE_QT TRUE)
else()
    message(WARNING "Qt6 not found. Building without Qt support.")
    set(USE_QT FALSE)
endif()

# 如果在 ROS 环境中，查找 ROS 2 依赖包
# If in ROS environment, find ROS 2 dependencies
if(USE_ROS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    # 【依赖项】如果需要添加新的 ROS 2 依赖，在这里添加 find_package 命令
    # [Dependencies] Add new find_package commands here for additional ROS 2 dependencies
    # 例如 / For example: find_package(sensor_msgs REQUIRED)
endif()

# 【依赖项】如果需要添加新的非 ROS 依赖，在这里添加 find_package 命令
# [Dependencies] Add new find_package commands here for additional non-ROS dependencies
# 例如 / For example: find_package(OpenCV REQUIRED)

# 设置包含目录
# Set include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}

    # 【依赖项】如果添加了新的依赖，可能需要在这里包含它们的头文件目录
    # [Dependencies] Include header directories of new dependencies here if needed
)

# 如果找到 Qt6，设置 Qt 相关选项和文件
# If Qt6 is found, set Qt-related options and files
if(USE_QT)
    # 设置 Qt 选项
    # Set Qt options
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

    # 使用通配符查找所有头文件、源文件、UI 文件和资源文件
    # Use wildcards to find all header files, source files, UI files, and resource files
    file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
    file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
    file(GLOB UI_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui)
    file(GLOB RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.qrc)

    # 【新文件】如果添加了新的源文件、头文件、UI文件或资源文件，请确保它们被包含在上面的 GLOB 命令中
    # [New Files] Ensure that any new source, header, UI, or resource files are included in the GLOB commands above

    # 声明 Qt 应用程序可执行文件
    # Declare Qt application executable
    add_executable(${PROJECT_NAME}
        ${HEADERS}
        ${SOURCES}
        ${UI_FILES}
        ${RESOURCES}
    )

    # 链接 Qt 库
    # Link Qt libraries
    target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Widgets)

    # 【依赖项】如果需要链接其他 Qt 模块或外部库，在这里添加
    # [Dependencies] Add links to other Qt modules or external libraries here if needed
    # 例如 / For example: target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Network)

    # 如果在 ROS 环境中，添加 ROS 依赖
    # If in ROS environment, add ROS dependencies
    if(USE_ROS)
        ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)

        # 【依赖项】如果添加了新的 ROS 2 依赖，在这里添加它们
        # [Dependencies] Add new ROS 2 dependencies here if added
        # 例如 / For example: ament_target_dependencies(${PROJECT_NAME} sensor_msgs)
    endif()
endif()

# 如果在 ROS 环境中，设置 ROS 相关目标和安装
# If in ROS environment, set ROS-related targets and installation
if(USE_ROS)
    # 声明 ROS 2 节点可执行文件
    # Declare ROS 2 node executable
    add_executable(${PROJECT_NAME}_node src/qt6_ros2_template_node.cpp)
    ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs)

    # 【依赖项】如果添加了新的 ROS 2 依赖，在这里也要添加它们
    # [Dependencies] Add new ROS 2 dependencies here as well if added
    # 例如 / For example: ament_target_dependencies(${PROJECT_NAME}_node sensor_msgs)

    # 安装可执行文件
    # Install executables
    install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
    )

    if(USE_QT)
        install(TARGETS
            ${PROJECT_NAME}
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()

    # 安装启动文件
    # Install launch files
    install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
    )

    # 【新文件】如果添加了新的配置文件、数据文件等，在这里添加相应的安装命令
    # [New Files] Add installation commands for new configuration files, data files, etc. here
    # 例如 / For example:
    # install(DIRECTORY
    # config
    # DESTINATION share/${PROJECT_NAME}
    # )

    # 测试设置
    # Testing setup
    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()

    # 声明 ament 包
    # Declare ament package
    ament_package()
else()
    # 非 ROS 环境下的设置
    # Settings for non-ROS environment
    if(USE_QT)
        # 为 Windows 设置 GUI 应用属性
        # Set GUI application properties for Windows
        if(WIN32)
            set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
        endif()

        # 为 macOS 设置 bundle 属性
        # Set bundle properties for macOS
        if(APPLE)
            set_target_properties(${PROJECT_NAME} PROPERTIES
                MACOSX_BUNDLE TRUE
                MACOSX_BUNDLE_GUI_IDENTIFIER com.example.${PROJECT_NAME}
                MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
                MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
            )
        endif()

        # 安装目标
        # Install targets
        install(TARGETS ${PROJECT_NAME}
            BUNDLE DESTINATION .
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    else()
        # 如果既没有 ROS 也没有 Qt，显示警告
        # If neither ROS nor Qt is available, display a warning
        message(WARNING "Neither ROS nor Qt support available. No targets will be built.")
    endif()
endif()