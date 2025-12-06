cmake_minimum_required(VERSION 3.5)
project(autonomous_system)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces (services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateToPose.srv"
)

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