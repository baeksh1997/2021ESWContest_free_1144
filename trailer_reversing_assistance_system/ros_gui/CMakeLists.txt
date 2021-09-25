cmake_minimum_required(VERSION 2.8.3)
project(ros_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pose_msgs
  image_transport
  cv_bridge
  geometry_msgs
  wrp_sdk
)

# Find Qt5 libraries
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Qml
  Quick
  Widgets
)
# Find OpenCV libraries
find_package(OpenCV 3 REQUIRED)

# Configuration to make Qt compiles go smoother
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
include_directories( ${CMAKE_BINARY_DIR} )

# Configure Catkin dependencies
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


######################################
# Talker and Listener using QML gui

# Run MOC on headers of Q_OBJECT classes
set(MOC_HDRS include/ros_connect.h include/ros_image.h)
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

# Run RCC on QRC file
set(RCC_FILES src/qml/qml_.qrc)
qt5_add_resources(RCC_SOURCES ${RCC_FILES})

# Build executables with source, MOC result files, RCC result files
add_executable(gui_wheel src/gui_wheel.cpp ${MOC_SRCS} ${RCC_SOURCES})
add_executable(gui_streaming src/gui_streaming.cpp)

# Configure Qt5 Modules
# Equivalent to doing
# Qt += Core Qml Quick
# In Qt .pro file
qt5_use_modules(gui_wheel Core Qml Quick)

target_link_libraries(gui_wheel
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(gui_streaming
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
