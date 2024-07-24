cmake_minimum_required(VERSION 3.8)
project(qt6_ros2_template VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 控制是否使用 ROS 的选项
option(USE_ROS "Build with ROS2 support" ON)

# 检查是否在 ROS 环境中
if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "ROS environment detected: $ENV{ROS_DISTRO}")
    set(USE_ROS ON)
else()
    message(STATUS "ROS environment not detected. Building without ROS support.")
    set(USE_ROS OFF)
endif()

# 尝试查找 Qt6
find_package(Qt6 COMPONENTS Widgets QUIET)

if(Qt6_FOUND)
    message(STATUS "Qt6 found. Building with Qt support.")
    set(USE_QT ON)
else()
    message(WARNING "Qt6 not found. Building without Qt support.")
    set(USE_QT OFF)
endif()

# ROS 2 依赖
if(USE_ROS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
endif()

# Qt 相关设置
if(USE_QT)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

    file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h *.hpp)
    file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp *.c)
    file(GLOB UI_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui)
    file(GLOB RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.qrc)

    add_executable(${PROJECT_NAME}
        ${HEADERS}
        ${SOURCES}
        ${UI_FILES}
        ${RESOURCES}
    )

    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    if(USE_ROS)
        ament_target_dependencies(${PROJECT_NAME}
            rclcpp
            std_msgs
        )
    endif()

    target_link_libraries(${PROJECT_NAME} 
        Qt6::Widgets
    )

    # 为 Windows 设置 GUI 应用属性
    if(WIN32)
        set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
    endif()

    # 为 macOS 设置 bundle 属性
    if(APPLE)
        set_target_properties(${PROJECT_NAME} PROPERTIES
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_GUI_IDENTIFIER com.example.${PROJECT_NAME}
            MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
            MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )
    endif()
endif()

# 安装目标
if(USE_ROS)
    install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
    )

    # 安装启动文件
    install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
    )

    ament_package()
else()
    if(USE_QT)
        install(TARGETS ${PROJECT_NAME}
            BUNDLE DESTINATION .
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    else()
        message(WARNING "Neither ROS nor Qt support available. No targets will be built.")
    endif()
endif()

# 添加编译定义
if(USE_ROS)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_ROS)
endif()
