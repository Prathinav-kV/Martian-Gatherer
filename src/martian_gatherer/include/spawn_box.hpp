/**
 * @file spawn_box.hpp
 * @brief Header file for the SpawnBoxNode class, which spawns a box in Gazebo and sends its coordinates.
 * 
 * Copyright 2024 Prathinav Karnala Venkata, Abhey Sharma, Sarang Nair.
 */

#ifndef SPAWN_BOX_HPP
#define SPAWN_BOX_HPP

#include <cmath> 
#include <fstream>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include "mars_interfaces/srv/send_coordinates.hpp"
#include <utility>
#include <vector>
#include <string>
#include <memory>

/**
 * @class SpawnBoxNode
 * @brief A ROS2 node that spawns a box in Gazebo at a random valid position and sends its coordinates.
 */
class SpawnBoxNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for SpawnBoxNode.
   */
  SpawnBoxNode();

  /**
   * @brief Spawns a box in Gazebo at a valid random position.
   */
  void spawn_box();

  /**
   * @brief Sends the coordinates of the spawned box to another service.
   */
  void send_coordinates();

  /**
   * @brief Checks if the given x, y position is valid for spawning the box.
   * @param x The x-coordinate of the position.
   * @param y The y-coordinate of the position.
   * @return True if the position is valid, false otherwise.
   */
  bool is_valid_spawn(double x, double y);

  /**
   * @brief Prints the coordinates of the spawned box.
   */
  void print_spawn_coordinates();

  /**
   * @brief Generates a random value within a given range.
   * @param min The minimum value of the range.
   * @param max The maximum value of the range.
   * @return A random double value within the given range.
   */
  double random_value(double min, double max);

 private:
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<mars_interfaces::srv::SendCoordinates>::SharedPtr send_coordinates_client_;
  std::pair<double, double> x_range_;
  std::pair<double, double> y_range_;
  double spawn_x_, spawn_y_;
};

#endif // SPAWN_BOX_HPP
