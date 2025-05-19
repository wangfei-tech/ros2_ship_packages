cmake_minimum_required(VERSION 3.5)
project(ros2_ship_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
# find_package(xacro REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# 如果有包含文件
# include_directories(include)

# 如果你有 src/.cpp 文件需要编译，请取消注释以下并修改对应路径
# add_executable(${PROJECT_NAME}_node src/ros2_ship_description_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs ...)
# install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

# 安装 meshes、urdf、launch、world 等模型相关资源
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)
install(DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots
)
install(DIRECTORY world/
  DESTINATION share/${PROJECT_NAME}/world
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
