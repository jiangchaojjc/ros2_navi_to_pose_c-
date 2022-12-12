/*
author:jiangchao
date:2022.12.09
description:navigate to pose use c++
*/

#include "cpp_navitopose/NavigateToPose.h"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

navi2_points_control::navi2_points_control()
    : nav2_util::LifecycleNode("navi2_points_control", "", false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);
  auto state = rclcpp_lifecycle::State();
  on_configure(state);
}

navi2_points_control::~navi2_points_control()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
navi2_points_control::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();

  std::vector<std::string> new_args = rclcpp::NodeOptions().arguments();
  new_args.push_back("--ros-args");
  new_args.push_back("-r");
  new_args.push_back(std::string("__node:=") + this->get_name() +
                     "_rclcpp_node");
  new_args.push_back("--");
  client_node_ = std::make_shared<rclcpp::Node>(
      "_", "", rclcpp::NodeOptions().arguments(new_args));

  nav_to_pose_client_ =
      rclcpp_action::create_client<ClientT>( // jc:创建NavigateToPose的client
          client_node_, "navigate_to_pose");

  // action_server_ = std::make_unique<ActionServer>(
  // //jc:调用了simple_action_server.hpp 51行的构造函数
  //   get_node_base_interface(),
  //   get_node_clock_interface(),
  //   get_node_logging_interface(),
  //   get_node_waitables_interface(),
  //   "FollowWaypoints", std::bind(&follow_points_control::followWaypoints,
  //   this), false);
  navigateToPose();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
navi2_points_control::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
navi2_points_control::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // action_server_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
navi2_points_control::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // action_server_.reset();
  nav_to_pose_client_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
navi2_points_control::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void navi2_points_control::navigateToPose()
{
  // auto goal = action_server_->get_current_goal();

  // //std::vector<geometry_msgs::msg::PoseStamped> poses;
  // auto feedback = std::make_shared<ActionT::Feedback>();
  // auto result = std::make_shared<ActionT::Result>();

  // // Check if request is valid
  // if (!action_server_ || !action_server_->is_server_active()) {
  //   RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
  //   return;
  // }

  // RCLCPP_INFO(
  //   get_logger(), "Received follow waypoint request with %i waypoints.",
  //   static_cast<int>(goal->poses.size()));

  // if (goal->poses.size() == 0) {
  //   action_server_->succeeded_current(result);
  //   return;
  // }

  rclcpp::WallRate r(loop_rate_);

  // Check if asked to stop processing action
  // if (action_server_->is_cancel_requested()) {  //jc:如果取消了当前的任务
  //   auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
  //   rclcpp::spin_until_future_complete(client_node_, cancel_future);
  //   // for result callback processing
  //   spin_some(client_node_);
  //   action_server_->terminate_all();
  //   return;
  // }

  // Check if asked to process another action
  // //jc:是否有其他的任务需要做，比如更换导航点 if
  // (action_server_->is_preempt_requested()) {
  //   RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
  //   goal = action_server_->accept_pending_goal();
  //   goal_index = 0;
  //   new_goal = true;
  // }

  // Check if we need to send a new goal

  ClientT::Goal client_goal; // jc:这个主要是发一个NavigateToPose_Goal_，包含posestamp
  // client_goal.pose = goal->poses[goal_index];

  client_goal.pose.header.frame_id = "map";
  client_goal.pose.header.stamp = rclcpp::Clock().now();
  client_goal.pose.pose.position.x = 0.48;
  client_goal.pose.pose.position.y = 1.74;
  client_goal.pose.pose.orientation.z = -0.707;
  client_goal.pose.pose.orientation.w = 0.707;
  client_goal.pose.pose.orientation.x = 0.0;
  client_goal.pose.pose.orientation.y = 0.0;

  RCLCPP_INFO(get_logger(), "jcjcjcjc get the goal pose. x %f y %f ",
              client_goal.pose.pose.position.x,
              client_goal.pose.pose.position.y);
  auto send_goal_options = rclcpp_action::Client<ClientT>::
      SendGoalOptions(); // jc:返回结构体SendGoalOptions，包含3个函数指针，主要是注册回调函数
  send_goal_options.result_callback = std::bind(
      &navi2_points_control::resultCallback, this, std::placeholders::_1);
  send_goal_options
      .goal_response_callback = // jc:主要是注册回调函数
      std::bind(&navi2_points_control::goalResponseCallback, this,
                std::placeholders::_1);
  future_goal_handle_ = nav_to_pose_client_->async_send_goal(
      client_goal, send_goal_options); // jc:发送请求，执行三个回调函数
  current_goal_status_ = ActionStatus::PROCESSING;

  // feedback->current_waypoint = goal_index;
  // action_server_->publish_feedback(feedback);

  // if (current_goal_status_ == ActionStatus::FAILED) {
  //   failed_ids_.push_back(goal_index);

  //   if (stop_on_failure_) {
  //     RCLCPP_WARN(
  //       get_logger(), "Failed to process waypoint %i in waypoint "
  //       "list and stop on failure is enabled."
  //       " Terminating action.", goal_index);
  //     result->missed_waypoints = failed_ids_;
  //     action_server_->terminate_current(result);
  //     failed_ids_.clear();
  //     return;
  //   } else {
  //     RCLCPP_INFO(
  //       get_logger(), "Failed to process waypoint %i,"
  //       " moving to next.", goal_index);
  //   }
  // } else if (current_goal_status_ == ActionStatus::SUCCEEDED) {
  //   RCLCPP_INFO(
  //     get_logger(), "Succeeded processing waypoint %i, "
  //     "moving to next.", goal_index);
  // }

  // if (current_goal_status_ != ActionStatus::PROCESSING &&
  //   current_goal_status_ != ActionStatus::UNKNOWN)
  // {
  //   // Update server state
  //   goal_index++;
  //   new_goal = true;
  //   if (goal_index >= goal->poses.size()) {
  //     RCLCPP_INFO(
  //       get_logger(), "Completed all %i waypoints requested.",
  //       goal->poses.size());
  //     result->missed_waypoints = failed_ids_;
  //     action_server_->succeeded_current(result);
  //     failed_ids_.clear();
  //     return;
  //   }
  // } else {
  //   RCLCPP_INFO_EXPRESSION(
  //     get_logger(),
  //     (static_cast<int>(now().seconds()) % 30 == 0),
  //     "Processing waypoint %i...", goal_index);
  // }

  rclcpp::spin_some(client_node_);
  r.sleep();
}

void navi2_points_control::resultCallback(
    const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result)
{
  RCLCPP_INFO(get_logger(), "resultCallback");
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    current_goal_status_ = ActionStatus::SUCCEEDED;
    return;
  case rclcpp_action::ResultCode::ABORTED:
    current_goal_status_ = ActionStatus::FAILED;
    return;
  case rclcpp_action::ResultCode::CANCELED:
    current_goal_status_ = ActionStatus::FAILED;
    return;
  default:
    current_goal_status_ = ActionStatus::UNKNOWN;
    return;
  }
}

void navi2_points_control::goalResponseCallback(
    std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>
        future)
{
  RCLCPP_INFO(get_logger(), "goalResponseCallback");
  auto goal_handle = future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(
        get_logger(),
        "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}
