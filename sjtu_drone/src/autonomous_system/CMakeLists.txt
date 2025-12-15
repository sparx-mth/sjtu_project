cmake_minimum_required(VERSION 3.5)
project(autonomous_system)

# ------------------------------------------------------------------------------
# Build settings
# ------------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ------------------------------------------------------------------------------
# Find packages
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# C++ dependencies
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ompl REQUIRED)

# ------------------------------------------------------------------------------
# Generate interfaces (services)
# ------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateToPose.srv"
  "srv/PlanPath.srv"
)

# Needed so C++ targets can link against generated typesupport
ament_export_dependencies(rosidl_default_runtime)

# ------------------------------------------------------------------------------
# Build C++ RRT planner service (planning only)
# ------------------------------------------------------------------------------
add_executable(rrt_planner_service src/rrt_planner_service.cpp)

target_include_directories(rrt_planner_service PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

# ROS2 deps (rclcpp)
ament_target_dependencies(rrt_planner_service
  rclcpp
)

# Link non-ROS libs
target_link_libraries(rrt_planner_service
  ompl::ompl
  yaml-cpp
  ${OpenCV_LIBS}
)

# Link generated ROS2 service typesupport into the C++ node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rrt_planner_service "${cpp_typesupport_target}")

install(TARGETS rrt_planner_service
  DESTINATION lib/${PROJECT_NAME}
)


# ------------------------------------------------------------------------------
# Package
# ------------------------------------------------------------------------------
ament_package()
