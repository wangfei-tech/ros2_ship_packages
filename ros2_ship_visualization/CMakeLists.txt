cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(ros2_ship_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)
find_path(OGRE_INCLUDE_DIR OgrePrerequisites.h PATHS /usr/include/OGRE)

find_library(OGRE_LIBRARY OgreMain)

if(NOT OGRE_INCLUDE_DIR OR NOT OGRE_LIBRARY)
  message(FATAL_ERROR "Ogre3D not found")
endif()

include_directories(${Python3_INCLUDE_DIRS})


# Qt 设置
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

# 生成消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlotCharacteristicCurve.srv"
  DEPENDENCIES std_msgs
)

# 添加库，避免与包名冲突
add_library(${PROJECT_NAME}_lib
  # src/overlay_gps_display.cpp
  # src/overlay_utils.cpp
  # include/overlay_gps_display.h
  # include/overlay_utils.h
)

# 包依赖
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  rviz_common
)

# 头文件路径
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${OGRE_INCLUDE_DIR})
# 链接库
target_link_libraries(${PROJECT_NAME}_lib
  Qt5::Widgets
  ${OpenCV_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_lib ${OGRE_LIBRARY})

# 安装
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/
)

# 导出接口
ament_export_dependencies(rclcpp sensor_msgs std_msgs geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)

ament_package()
