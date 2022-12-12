/*
author:jiangchao
date:2022.12.09
description:navigate to pose use c++
*/

#include <memory>

#include "cpp_navitopose/NavigateToPose.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navi2_points_control>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
