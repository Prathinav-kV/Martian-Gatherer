/**
 * @file nav2goal.hpp
 * @brief Header file for the ClickedPointToPose class, which handles sending navigation goals.
 * 
 * Copyright 2024 Prathinav Karnala Venkata, Abhey Sharma, Sarang Nair.
 */

#ifndef NAV2GOAL_HPP
#define NAV2GOAL_HPP

#include <gazebo_msgs/srv/delete_entity.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mars_interfaces/srv/send_coordinates.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <string>
#include <chrono>
#include <array>

class ClickedPointToPose : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Constructor for ClickedPointToPose node.
   * @param name Name of the node
   */
  explicit ClickedPointToPose(const std::string &name);

  /**
   * @brief Sends the navigation goal to the action server.
   */
  void send_goal();

  /**
   * @brief Sets the goal pose for the robot.
   * @param box_x The x-coordinate of the goal position.
   * @param box_y The y-coordinate of the goal position.
   */
  void set_goal_pose(double box_x, double box_y);

  /**
   * @brief Sets the initial pose for the robot.
   */
  void set_initial_pose();

  /**
   * @brief Deletes the simple box model from Gazebo.
   */
  void execute_next_task();

 private:
  /**
   * @brief Callback function for the SendCoordinates service.
   * @param request The incoming service request containing x, y, z coordinates.
   * @param response The service response indicating success or failure.
   */
  void handle_send_coordinates(
    const std::shared_ptr<mars_interfaces::srv::SendCoordinates::Request> request,
    std::shared_ptr<mars_interfaces::srv::SendCoordinates::Response> response
  );

  /**
   * @brief Result callback for the navigation goal.
   * @param result The result of the navigation goal.
   */
  void result_callback(const GoalHandleNavigateToPose::WrappedResult &result);

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub2_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  geometry_msgs::msg::PoseStamped goal_pose;
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  rclcpp::Service<mars_interfaces::srv::SendCoordinates>::SharedPtr service_;
  std::shared_ptr<mars_interfaces::srv::SendCoordinates::Response> service_response_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
};

#endif // NAV2GOAL_HPP
