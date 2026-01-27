// Copyright 2026 Robin Müller
//
// Licensed under the MIT License

/**
 * @file custom_build_handler.cpp
 * @brief Custom behavior tree build handler for the FOSDEM '26 devtalk.
 *
 * This build handler creates a behavior tree that moves a robot hand in a custom
 * sequence of left/right movements based on a build request string (e.g., "l;l;r;r;l;r").
 */

#include <sstream>
#include <vector>

#include "auto_apms_behavior_tree/build_handler.hpp"

namespace fosdem26_autoapms_behavior
{

/**
 * @brief Custom build handler that creates movement sequences from a direction string.
 *
 * The build request should be a semicolon-separated string of 'l' (left) and 'r' (right)
 * directions, e.g., "l;l;r;r;l;r" for the pattern: Left → Left → Right → Right → Left → Right.
 *
 * This handler reuses the existing MoveToEndPosition and MoveToCenterPosition subtrees,
 * loading the appropriate node manifest (wave_left or wave_right) to configure the movement
 * direction.
 */
class CustomBuilder : public auto_apms_behavior_tree::TreeBuildHandler
{
  enum class Direction
  {
    Left,
    Right
  };
  std::vector<Direction> directions_;

public:
  using TreeBuildHandler::TreeBuildHandler;

  bool setBuildRequest(
    const std::string & build_request, const std::string & /*entrypoint*/,
    const NodeManifest & /*node_manifest*/) override final
  {
    // Parse the build request string into a vector of directions
    directions_.clear();
    std::stringstream ss(build_request);
    std::string token;

    while (std::getline(ss, token, ';')) {
      // Trim whitespace
      token.erase(0, token.find_first_not_of(" \t"));
      token.erase(token.find_last_not_of(" \t") + 1);

      if (token.empty()) {
        continue;
      }

      if (token == "l" || token == "L" || token == "left") {
        directions_.push_back(Direction::Left);
      } else if (token == "r" || token == "R" || token == "right") {
        directions_.push_back(Direction::Right);
      } else {
        RCLCPP_WARN(logger_, "Unknown direction '%s' in build request, skipping.", token.c_str());
      }
    }

    if (directions_.empty()) {
      RCLCPP_ERROR(
        logger_, "Build request '%s' did not contain any valid directions (use 'l' or 'r' separated by ';').",
        build_request.c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Parsed %zu movement directions from build request.", directions_.size());
    return true;
  }

  TreeDocument::TreeElement buildTree(TreeDocument & doc, TreeBlackboard & /*bb*/) override final
  {
    // Alias for standard node models
    namespace model = auto_apms_behavior_tree::model;

    // Create separate documents for left and right configurations
    TreeDocument doc_left, doc_right;

    // Load only MoveToEndPosition from the left-configured resource and rename it
    doc_left.newTreeFromResource("fosdem26_autoapms_behavior::wave_left::MoveToEndPosition")
      .setName("MoveLeft")
      .makeRoot();

    // Since we expect the "wave_left" and "wave_right" node manifest share common node names, we must apply a
    // namespace to avoid name clashes when both are loaded into the same document.
    doc_left.applyNodeNamespace("wave_left");

    // Load only MoveToEndPosition from the right-configured resource and rename it
    doc_right.newTreeFromResource("fosdem26_autoapms_behavior::wave_right::MoveToEndPosition")
      .setName("MoveRight")
      .makeRoot();

    // Since we expect the "wave_left" and "wave_right" node manifest share common node names, we must apply a
    // namespace to avoid name clashes when both are loaded into the same document.
    doc_right.applyNodeNamespace("wave_right");

    // Add the subtree to move back to the center
    TreeDocument::TreeElement move_to_center =
      doc.newTreeFromResource("fosdem26_autoapms_behavior::wave_left::MoveToCenterPosition");

    // Create the root tree with a sequence
    TreeDocument::TreeElement tree = doc.newTree("CustomBehavior").makeRoot();
    model::Sequence sequence = tree.insertNode<model::Sequence>();

    // Insert subtree calls for each direction in the sequence
    for (const auto & dir : directions_) {
      const TreeDocument::TreeElement move_to_side =
        (dir == Direction::Left) ? doc_left.getTree("MoveLeft") : doc_right.getTree("MoveRight");
      sequence.insertSubTreeNode(move_to_side);
      sequence.insertSubTreeNode(move_to_center);
    }

    // Print tree
    // RCLCPP_INFO(logger_, "Constructed behavior tree:\n%s", doc.writeToString().c_str());

    return tree;
  }
};

}  // namespace fosdem26_autoapms_behavior

// Make the build handler discoverable for the class loader
AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(fosdem26_autoapms_behavior::CustomBuilder)
