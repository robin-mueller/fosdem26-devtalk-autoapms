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

# Register common behavior tree nodes
auto_apms_behavior_tree_register_nodes(behavior_tree_nodes
  "fosdem26_autoapms_behavior::VelocityPub"
  "fosdem26_autoapms_behavior::PositionInRange"
)

# Register nodes for specific use cases
auto_apms_behavior_tree_register_nodes(wave_left
  NODE_MANIFEST
  config/common_nodes.yaml
  config/wave_left_nodes.yaml
)
auto_apms_behavior_tree_register_nodes(wave_right
  NODE_MANIFEST
  config/common_nodes.yaml
  config/wave_right_nodes.yaml
)

# Register behavior trees for specific use cases
auto_apms_behavior_tree_register_trees(
  behavior/tree/generic_wave.xml
  ALIAS_NAMESPACE "wave_left"
  NODE_MANIFEST "${PROJECT_NAME}::wave_left"
)
auto_apms_behavior_tree_register_trees(
  behavior/tree/generic_wave.xml
  ALIAS_NAMESPACE "wave_right"
  NODE_MANIFEST "${PROJECT_NAME}::wave_right"
)

# Install library
install(
  TARGETS behavior_tree_nodes
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
