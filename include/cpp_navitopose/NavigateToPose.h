/*
author:jiangchao
date:2022.12.09
description:navigate to pose use c++
*/

#ifndef NAVIGATETOPOSE_H
#define NAVIGATETOPOSE_H

#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


enum class ActionStatus {
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

/**
 * @class nav2_points_control::follow_points_control
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class navi2_points_control : public nav2_util::LifecycleNode {
public:
  using ClientT = nav2_msgs::action::NavigateToPose;

  using ActionClient = rclcpp_action::Client<ClientT>;

  /**
   * @brief A constructor for nav2_points_control::nav2_points_control class
   */
  navi2_points_control();
  /**
   * @brief A destructor for nav2_points_control::nav2_points_control class
   */
  ~navi2_points_control();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "FollowWaypoints"
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Action server callbacks
   */
  void navigateToPose();

  /**
   * @brief Action client result callback
   * @param result Result of action server updated asynchronously
   */
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result);

  /**
   * @brief Action client goal response callback
   * @param future Shared future to goalhandle
   */
  void goalResponseCallback(
      std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>
          future);

  // Our action server
  ActionClient::SharedPtr nav_to_pose_client_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>
      future_goal_handle_;
  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  int loop_rate_;
  std::vector<int> failed_ids_;

  nav2_msgs::action::NavigateToPose goal;
};

#endif // NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
