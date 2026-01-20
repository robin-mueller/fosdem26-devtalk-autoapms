cmake_minimum_required(VERSION 3.9)
project(fosdem26_autoapms_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)
find_package(fosdem26_autoapms_interfaces REQUIRED)

# Create shared library with node implementations
add_library(behavior_tree_nodes SHARED
  src/velocity_pub.cpp
  src/position_in_range.cpp
)
target_link_libraries(behavior_tree_nodes PUBLIC
  auto_apms_behavior_tree::auto_apms_behavior_tree
  ${fosdem26_autoapms_interfaces_TARGETS}
)

# Register behavior tree nodes
auto_apms_behavior_tree_register_nodes(behavior_tree_nodes
  "fosdem26_autoapms_behavior::VelocityPub"
  "fosdem26_autoapms_behavior::PositionInRange"
  NODE_MANIFEST config/nodes.yaml
)

# Register behavior trees
auto_apms_behavior_tree_register_trees(
  behavior/tree/wave_hand.xml
  NODE_MANIFEST "${PROJECT_NAME}::behavior_tree_nodes"
)

# Install library
install(
  TARGETS behavior_tree_nodes
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
