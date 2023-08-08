/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>

using namespace std::placeholders;

namespace moveit_simple_controller_manager
{
bool FollowJointTrajectoryControllerHandle::sendTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "new trajectory to " << name_);

  if (!controller_action_client_)
    return false;

  if (!isConnected())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Action client not connected to action server: " << getActionName());
    return false;
  }

  if (done_)
    RCLCPP_INFO_STREAM(LOGGER, "sending trajectory to " << name_);
  else
    RCLCPP_INFO_STREAM(LOGGER, "sending continuation for the currently executed trajectory to " << name_);

  control_msgs::action::FollowJointTrajectory::Goal goal = goal_template_;
  goal.trajectory = trajectory.joint_trajectory;
  goal.multi_dof_trajectory = trajectory.multi_dof_joint_trajectory;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions send_goal_options;
  // Active callback
  send_goal_options.goal_response_callback =
      [this](
          const rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::GoalHandle::SharedPtr& goal_handle) {
        RCLCPP_INFO_STREAM(LOGGER, name_ << " started execution");
        if (!goal_handle)
          RCLCPP_WARN(LOGGER, "Goal request rejected");
        else
          RCLCPP_INFO(LOGGER, "Goal request accepted!");
      };

  done_ = false;
  last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;

  // Send goal
  auto current_goal_future = controller_action_client_->async_send_goal(goal, send_goal_options);
  current_goal_ = current_goal_future.get();
  if (!current_goal_)
  {
    RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
    return false;
  }
  return true;
}

namespace
{
const char* errorCodeToMessage(int error_code)
{
  switch (error_code)
  {
    case control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL:
      return "SUCCESSFUL";
    case control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL:
      return "INVALID_GOAL";
    case control_msgs::action::FollowJointTrajectory::Result::INVALID_JOINTS:
      return "INVALID_JOINTS";
    case control_msgs::action::FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP:
      return "OLD_HEADER_TIMESTAMP";
    case control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED:
      return "PATH_TOLERANCE_VIOLATED";
    case control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED:
      return "GOAL_TOLERANCE_VIOLATED";
    default:
      return "unknown error";
  }
}
}  // namespace


std::vector<control_msgs::msg::JointTolerance>
FollowJointTrajectoryControllerHandle::configure_tolerance_from_params(
  const std::string & parameter_prefix)
{
  std::vector<control_msgs::msg::JointTolerance> tolerances;
  auto positions = node_->declare_parameter<std::vector<double>>(
    parameter_prefix + ".position",
    std::vector<double>{});
  if (positions.size() != 0 && positions.size() != joints_.size()) {
    RCLCPP_WARN_STREAM(
      LOGGER,
      "Parameter '" << parameter_prefix << ".position' should have one value for each joint."
      " Number of joints " << joints_.size());
  }
  auto velocities = node_->declare_parameter<std::vector<double>>(
    parameter_prefix + ".velocity",
    std::vector<double>{});
  if (velocities.size() != 0 && velocities.size() != joints_.size()) {
    RCLCPP_WARN_STREAM(
      LOGGER,
      "Parameter '" << parameter_prefix << ".velocity' should have one value for each joint."
      " Number of joints " << joints_.size());
  }
  auto accelerations = node_->declare_parameter<std::vector<double>>(
    parameter_prefix + ".acceleration",
    std::vector<double>{});
  if (accelerations.size() != 0 && accelerations.size() != joints_.size()) {
    RCLCPP_WARN_STREAM(
      LOGGER,
      "Parameter '" << parameter_prefix << ".acceleration' should have one value for each joint."
      " Number of joints " << joints_.size());
  }
  if (positions.size() == 0 && velocities.size() == 0 && accelerations.size() == 0) {
    // nothing specified, default tolerances
    return tolerances;
  }
  size_t i = 0;
  bool use_position_tol = positions.size() == joints_.size();
  bool use_velocity_tol = positions.size() == joints_.size();
  bool use_acc_tol = positions.size() == joints_.size();
  for (const auto & joint : joints_) {
    control_msgs::msg::JointTolerance tol;
    tol.name = joint;
    tolerances.emplace_back(tol);
    if (use_position_tol) {
      tol.position = positions[i];
    }
    if (use_velocity_tol) {
      tol.velocity = velocities[i];
    }
    if (use_acc_tol) {
      tol.acceleration = accelerations[i];
    }
    ++i;
  }
  return tolerances;
}

FollowJointTrajectoryControllerHandle::FollowJointTrajectoryControllerHandle(
  const rclcpp::Node::SharedPtr& node,
  const std::string& name,
  const std::string& action_ns)
: ActionBasedControllerHandle<control_msgs::action::FollowJointTrajectory>(
    node, name, action_ns, "moveit.simple_controller_manager.follow_joint_trajectory_controller_handle")
{
  goal_template_.path_tolerance = configure_tolerance_from_params(name_ + ".path_tolerance");
  goal_template_.path_tolerance = configure_tolerance_from_params(name_ + ".goal_tolerance");
  goal_template_.goal_time_tolerance = rclcpp::Duration::from_seconds(
    node->declare_parameter("moveit_simple_controller_manager." + name_ + ".goal_time_tolerance", 0.));
}

void FollowJointTrajectoryControllerHandle::controllerDoneCallback(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& wrapped_result)
{
  // Output custom error message for FollowJointTrajectoryResult if necessary
  if (!wrapped_result.result)
    RCLCPP_WARN_STREAM(LOGGER, "Controller '" << name_ << "' done, no result returned");
  else if (wrapped_result.result->error_code == control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
    RCLCPP_INFO_STREAM(LOGGER, "Controller '" << name_ << "' successfully finished");
  else
    RCLCPP_WARN_STREAM(LOGGER, "Controller '" << name_ << "' failed with error "
                                              << errorCodeToMessage(wrapped_result.result->error_code) << ": "
                                              << wrapped_result.result->error_string);
  finishControllerExecution(wrapped_result.code);
}

}  // end namespace moveit_simple_controller_manager
