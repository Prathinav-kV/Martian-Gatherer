/**
 * @file nav2goal.cpp
 * @brief ROS2 node to convert clicked point coordinates to initial and goal
 * poses and send navigation goals.
 *
 * Copyright 2024 Prathinav Karnala Venkata, Abhey Sharma, Sarang Nair.
 */

#include "gazebo_msgs/srv/delete_entity.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mars_interfaces/srv/send_coordinates.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::chrono::seconds;
using std::shared_ptr;

/**
 * @class ClickedPointToPose
 * @brief A ROS2 node that sets an initial pose, sends navigation goals, and
 * deletes a box in Gazebo.
 */
class ClickedPointToPose : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Constructor for ClickedPointToPose node.
   * @param name Name of the node
   */
  explicit ClickedPointToPose(const std::string &name) : Node(name) {
    // Publisher for initial pose
    pub2_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10);

    // Create service to receive coordinates
    service_ = this->create_service<mars_interfaces::srv::SendCoordinates>(
        "send_coordinates",
        std::bind(&ClickedPointToPose::handle_send_coordinates, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Action client to send goals to the "navigate_to_pose" action server
    action_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Create client to delete the entity
    delete_client_ =
        this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
  }

 private:
  /**
   * @brief Callback function for the SendCoordinates service.
   * @param request The incoming service request containing x, y, z coordinates.
   * @param response The service response indicating success or failure.
   */
  void handle_send_coordinates(
      const std::shared_ptr<mars_interfaces::srv::SendCoordinates::Request>
          request,
      std::shared_ptr<mars_interfaces::srv::SendCoordinates::Response>
          response) {
    double box_x = request->x;
    double box_y = request->y;
    double box_z = request->z;

    RCLCPP_INFO(get_logger(), "Received coordinates: x=%.2f, y=%.2f, z=%.2f",
                box_x, box_y, box_z);

    service_response_ = response;

    set_initial_pose();
    pub2_->publish(initial_pose);

    RCLCPP_INFO_STREAM(get_logger(), "Waiting for 3 seconds...");
    rclcpp::sleep_for(std::chrono::seconds(3));

    set_goal_pose(box_x, box_y);
    send_goal();
  }

  /**
   * @brief Sends the navigation goal to the action server.
   */
  void send_goal() {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
      if (service_response_) {
        service_response_->success = false;
      }
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    RCLCPP_INFO(get_logger(), "Sending goal...");

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
        &ClickedPointToPose::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Sets the goal pose for the robot.
   * @param box_x The x-coordinate of the goal position.
   * @param box_y The y-coordinate of the goal position.
   */
  void set_goal_pose(double box_x, double box_y) {
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();

    goal_pose.pose.position.x = box_x;
    goal_pose.pose.position.y = box_y;
    goal_pose.pose.position.z = 0.0;

    double robot_x = 0.0;
    double robot_y = 0.0;
    double theta = atan2(box_y - robot_y, box_x - robot_x);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    goal_pose.pose.orientation.x = q.x();
    goal_pose.pose.orientation.y = q.y();
    goal_pose.pose.orientation.z = q.z();
    goal_pose.pose.orientation.w = q.w();

    RCLCPP_INFO(get_logger(),
                "Set goal to (x=%.2f, y=%.2f) with yaw=%.2f radians", box_x,
                box_y, theta);
  }

  /**
   * @brief Sets the initial pose for the robot.
   */
  void set_initial_pose() {
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->now();

    initial_pose.pose.pose.position.x = 0.0;
    initial_pose.pose.pose.position.y = 0.0;
    initial_pose.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    initial_pose.pose.pose.orientation.x = q.x();
    initial_pose.pose.pose.orientation.y = q.y();
    initial_pose.pose.pose.orientation.z = q.z();
    initial_pose.pose.pose.orientation.w = q.w();

    std::array<double, 36> cov = {
        0.25, 0, 0, 0,    0, 0, 0, 0.25, 0, 0, 0,    0, 0, 0, 0.25, 0, 0, 0,
        0,    0, 0, 0.25, 0, 0, 0, 0,    0, 0, 0.25, 0, 0, 0, 0,    0, 0, 0.25};
    initial_pose.pose.set__covariance(cov);
  }

  /**
   * @brief Result callback for the navigation goal.
   * @param result The result of the navigation goal.
   */
  void result_callback(const GoalHandleNavigateToPose::WrappedResult &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Goal succeeded!");
      if (service_response_) {
        service_response_->success = true;
      }
      execute_next_task();
    } else {
      RCLCPP_ERROR(get_logger(), "Goal failed.");
      if (service_response_) {
        service_response_->success = false;
      }
    }
    service_response_.reset();
  }

  /**
   * @brief Deletes the simple box model from Gazebo.
   */
  void execute_next_task() {
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = "simple_box";

    auto future = delete_client_->async_send_request(
        request,
        [this](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture
                   response) {
          if (response.get()->success) {
            RCLCPP_INFO(this->get_logger(),
                        "Successfully deleted model 'simple_box'");
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to delete model 'simple_box': %s",
                         response.get()->status_message.c_str());
          }
        });
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pub2_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  geometry_msgs::msg::PoseStamped goal_pose;
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  rclcpp::Service<mars_interfaces::srv::SendCoordinates>::SharedPtr service_;
  std::shared_ptr<mars_interfaces::srv::SendCoordinates::Response>
      service_response_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClickedPointToPose>("clicked_point_to_pose"));
  rclcpp::shutdown();
  return 0;
}
