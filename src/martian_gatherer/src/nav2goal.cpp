#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class ClickedPointToPose : public rclcpp::Node
{
public:

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ClickedPointToPose(const std::string & name)
  : Node(name)
  {
    pub2_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

    one_shot_timer_ = this->create_wall_timer(500ms, std::bind(&ClickedPointToPose::callback_timer, this));

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

private:
  void callback_timer()
  {
    RCLCPP_INFO_STREAM(get_logger(), "Setting initial pose");
    set_initial_pose();
    pub2_->publish(initial_pose);

    RCLCPP_INFO_STREAM(get_logger(), "Waiting for 3 seconds...");
    rclcpp::sleep_for(3s);

    RCLCPP_INFO_STREAM(get_logger(), "Sending goal pose");
    set_goal_pose();
    send_goal();

    one_shot_timer_->cancel();
    RCLCPP_INFO_STREAM(get_logger(), "Timer canceled");
  }

  void set_goal_pose()
  {
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();

    goal_pose.pose.position.x = 3.0;
    goal_pose.pose.position.y = 3.0;
    goal_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 1.57);

    goal_pose.pose.orientation.x = q.x();
    goal_pose.pose.orientation.y = q.y();
    goal_pose.pose.orientation.z = q.z();
    goal_pose.pose.orientation.w = q.w();
  }

  void set_initial_pose()
  {
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
      0.25, 0, 0, 0, 0, 0,
      0, 0.25, 0, 0, 0, 0,
      0, 0, 0.25, 0, 0, 0,
      0, 0, 0, 0.25, 0, 0,
      0, 0, 0, 0, 0.25, 0,
      0, 0, 0, 0, 0, 0.25,
    };
    initial_pose.pose.set__covariance(cov);
  }

  void send_goal()
  {
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    RCLCPP_INFO(get_logger(), "Sending goal...");

    // goal options
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&ClickedPointToPose::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Goal succeeded!");
      execute_next_task();
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_ERROR(get_logger(), "Goal aborted");
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_ERROR(get_logger(), "Goal canceled");
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown result code");
    }
  }

  void execute_next_task()
  {
    RCLCPP_INFO(get_logger(), "Executing the next task...");
    // Add your next task logic here.
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub2_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_;
  geometry_msgs::msg::PoseStamped goal_pose;
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ClickedPointToPose>("clicked_point_to_pose"));

  rclcpp::shutdown();
  
  return 0;
}
