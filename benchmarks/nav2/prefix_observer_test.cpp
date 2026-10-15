// SPDX-License-Identifier: Apache-2.0
#include <stdexcept>
#include "nav2_smac_planner/prefix_observer.hpp"

int main()
{
  using nav2_smac_planner::NodeLattice;
  namespace observer = rs_benchmark_observer;
  const auto require = [](bool condition) {
      if (!condition) {throw std::runtime_error("observer guard failed");}
    };
  NodeLattice parent(1), other_parent(2), node(3);
  nav2_smac_planner::MotionPrimitive primitive{};
  node.parent = &parent;
  node.pose = {1.25f, 2.5f, 3.0f};
  node.backwards(true);
  node.setMotionPrimitive(&primitive);
  observer::reset();
  observer::capture(&node);
  require(!observer::surviving_prefix(&node));  // Never override live metadata.
  require(node.getMotionPrimitive() == &primitive && node.parent == &parent &&
    node.pose.x == 1.25f && node.isBackward());
  node.setMotionPrimitive(nullptr);
  require(observer::surviving_prefix(&node)->primitive == &primitive);
  observer::capture(&node);  // A repeated null capture preserves the original.
  require(observer::captured_count() == 1);
  node.parent = &other_parent;
  require(!observer::surviving_prefix(&node));
  node.parent = &parent;
  for (float * coordinate : {&node.pose.x, &node.pose.y, &node.pose.theta}) {
    const float previous = *coordinate;
    *coordinate += 0.125f;
    require(!observer::surviving_prefix(&node));
    *coordinate = previous;
  }
  node.backwards(false);
  require(!observer::surviving_prefix(&node));
  node.backwards(true);
  require(observer::surviving_prefix(&node));
  observer::reset();
  require(observer::captured_count() == 0 && !observer::surviving_prefix(&node));
  observer::capture(&node);
  require(observer::captured_count() == 0);  // Null metadata is never evidence.
}
