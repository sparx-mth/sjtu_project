cmake_minimum_required(VERSION 3.5)
project(autonomous_system)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === C++ dependencies for RRT ===
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# === OMPL (CMake package, not pkg-config) ===
find_package(ompl REQUIRED)

# Generate interfaces (services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateToPose.srv"
)

# === Build C++ RRT navigation service ===
add_executable(rrt_navigation_service_cpp src/rrt_navigation_service.cpp)

target_include_directories(rrt_navigation_service_cpp
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

# IMPORTANT: use "plain" signature everywhere to avoid keyword/plain mixing
target_link_libraries(rrt_navigation_service_cpp
  ompl::ompl
  yaml-cpp
  ${OpenCV_LIBS}
)

ament_target_dependencies(rrt_navigation_service_cpp
  rclcpp
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(rrt_navigation_service_cpp "${cpp_typesupport_target}")

install(TARGETS rrt_navigation_service_cpp
  DESTINATION lib/${PROJECT_NAME}
)
# === END NEW ===

# Install Python package subdirectories to dist-packages
install(
  DIRECTORY autonomous_system/agents
  DESTINATION local/lib/python3.10/dist-packages/autonomous_system
)

install(
  DIRECTORY autonomous_system/control
  DESTINATION local/lib/python3.10/dist-packages/autonomous_system
)

install(
  DIRECTORY autonomous_system/planning
  DESTINATION local/lib/python3.10/dist-packages/autonomous_system
)

install(
  DIRECTORY autonomous_system/mapping
  DESTINATION local/lib/python3.10/dist-packages/autonomous_system
)

# Install the executable scripts with proper names
install(PROGRAMS
  autonomous_system/agents/navigation_agent_service_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_agent_service
)

install(PROGRAMS
  autonomous_system/agents/meta_agent_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME meta_agent
)

install(PROGRAMS
  autonomous_system/agents/doorway_traversal_agent.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME doorway_traversal_agent
)

install(PROGRAMS
  autonomous_system/agents/frontier_exploration_service_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME frontier_exploration_service
)

install(PROGRAMS
  autonomous_system/agents/room_exploration_agent.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME room_exploration_agent
)

install(PROGRAMS
  autonomous_system/agents/turn_right_agent_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME turn_right_agent
)

install(PROGRAMS
  autonomous_system/agents/turn_left_agent_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME turn_left_agent
)

install(PROGRAMS
  autonomous_system/agents/move_forward_agent_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_forward_agent
)

install(PROGRAMS
  autonomous_system/mapping/show_drone_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME show_drone_map
)

install(PROGRAMS
  autonomous_system/mapping/show_drone_map_with_exploration.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME show_drone_map_exploration
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()