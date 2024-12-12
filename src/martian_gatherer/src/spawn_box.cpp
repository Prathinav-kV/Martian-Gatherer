/**
 * @file spawn_box_node.cpp
 * @brief ROS2 node to spawn a box in Gazebo at a valid random position and send
 * its coordinates.
 *
 * Copyright 2024 Prathinav Karnala Venkata, Abhey Sharma, Sarang Nair.
 */

#include <cmath>  // Required for sqrt and pow
#include <fstream>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "mars_interfaces/srv/send_coordinates.hpp"

/**
 * @class SpawnBoxNode
 * @brief A ROS2 node that spawns a box in Gazebo at a random valid position and
 * sends its coordinates.
 */
class SpawnBoxNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for SpawnBoxNode.
   */
  SpawnBoxNode() : Node("spawn_box_node") {
    x_range_ = std::make_pair(-3.5, 5.5);
    y_range_ = std::make_pair(-6.5, 2.5);

    spawn_client_ =
        this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    send_coordinates_client_ =
        this->create_client<mars_interfaces::srv::SendCoordinates>(
            "/send_coordinates");

    while (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /spawn_entity service...");
    }
    RCLCPP_INFO(this->get_logger(), "/spawn_entity service is now available");

    while (
        !send_coordinates_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for /send_coordinates service...");
    }
    RCLCPP_INFO(this->get_logger(),
                "/send_coordinates service is now available");

    spawn_box();
    print_spawn_coordinates();
    send_coordinates();
  }

 private:
  /**
   * @brief Spawns a box in Gazebo at a valid random position.
   */
  void spawn_box() {
    bool valid_position = false;

    while (!valid_position) {
      spawn_x_ = random_value(x_range_.first, x_range_.second);
      spawn_y_ = random_value(y_range_.first, y_range_.second);
      valid_position = is_valid_spawn(spawn_x_, spawn_y_);

      if (!valid_position) {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid position (x=%.2f, y=%.2f). Retrying...", spawn_x_,
                    spawn_y_);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    }

    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();

    // std::string model_path =
    //     "/home/pratkv/mars_ws/src/martian_gatherer/models/simple_box/model.sdf";
    std::string package_path = ament_index_cpp::get_package_share_directory(
      "martian_gatherer");
    std::string model_path = package_path + "/models/simple_box/model.sdf";
    std::ifstream model_file(model_path);
    if (!model_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Model file not found at %s",
                   model_path.c_str());
      return;
    }

    std::string model_xml((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());
    request->xml = model_xml;
    request->name = "simple_box";
    request->robot_namespace = "simple_box_namespace";
    request->initial_pose.position.x = spawn_x_;
    request->initial_pose.position.y = spawn_y_;
    request->initial_pose.position.z = 0.0;

    RCLCPP_INFO(
        this->get_logger(),
        "Calling /spawn_entity service to spawn the box at (x=%.2f, y=%.2f)",
        spawn_x_, spawn_y_);

    spawn_client_->async_send_request(request);

    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(this->get_logger(),
                "Spawn request sent to /spawn_entity service.");
  }

  /**
   * @brief Sends the coordinates of the spawned box to another service.
   */
  void send_coordinates() {
    auto request =
        std::make_shared<mars_interfaces::srv::SendCoordinates::Request>();
    request->x = spawn_x_;
    request->y = spawn_y_;
    request->z = 0.0;

    RCLCPP_INFO(this->get_logger(),
                "Sending coordinates to /send_coordinates service: x=%.2f, "
                "y=%.2f, z=0.0",
                request->x, request->y);

    auto future = send_coordinates_client_->async_send_request(request);

    future.wait();

    RCLCPP_INFO(this->get_logger(),
                "Coordinates request sent to /send_coordinates service.");
  }

  /**
   * @brief Checks if the given x, y position is valid for spawning the box.
   * @param x The x-coordinate of the position.
   * @param y The y-coordinate of the position.
   * @return True if the position is valid, false otherwise.
   */
  bool is_valid_spawn(double x, double y) {
    if (x < -3.5 || x > 5.5 || y < -6.5 || y > 2.5) return false;

    std::vector<std::pair<double, double>> cylinders = {{3.335, 0.01},
                                                        {-2.534, 0.07},
                                                        {-2.8, -5.11},
                                                        {0.177, -4.83},
                                                        {4.12, -4.72}};
    double cylinder_radius = 0.5;

    for (const auto& cylinder : cylinders) {
      double distance = std::sqrt(std::pow(x - cylinder.first, 2) +
                                  std::pow(y - cylinder.second, 2));
      if (distance < cylinder_radius + 0.5) return false;
    }

    if (x >= 2.0 && x <= 4.0 && y >= -5.0 && y <= 0.0) return false;

    return true;
  }

  /**
   * @brief Prints the coordinates of the spawned box.
   */
  void print_spawn_coordinates() {
    RCLCPP_INFO(this->get_logger(),
                "Final spawn coordinates used: x=%.2f, y=%.2f, z=0.0", spawn_x_,
                spawn_y_);
  }

  /**
   * @brief Generates a random value within a given range.
   * @param min The minimum value of the range.
   * @param max The maximum value of the range.
   * @return A random double value within the given range.
   */
  double random_value(double min, double max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
  }

  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<mars_interfaces::srv::SendCoordinates>::SharedPtr
      send_coordinates_client_;
  std::pair<double, double> x_range_;
  std::pair<double, double> y_range_;
  double spawn_x_, spawn_y_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpawnBoxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
