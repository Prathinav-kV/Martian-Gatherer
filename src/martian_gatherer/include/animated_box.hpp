/**
 * @file animated_box.hpp
 * @brief Header file for the AnimatedBox class, a Gazebo plugin that animates a box along a trajectory.
 * 
 * Copyright 2024 Prathinav Karnala Venkata, Abhey Sharma, Sarang Nair.
 */

#ifndef ANIMATED_BOX_HPP
#define ANIMATED_BOX_HPP

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <map>
#include <memory>

namespace gazebo {

/**
 * @class AnimatedBox
 * @brief A Gazebo plugin that animates a box model along a predefined trajectory using waypoints.
 */
class AnimatedBox : public ModelPlugin {
 public:
  /**
   * @brief Loads the plugin and sets the animation for the box.
   * @param _parent Pointer to the model that this plugin is attached to.
   * @param _sdf Pointer to the SDF element of the plugin.
   */
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

 private:
  /**
   * @brief Loads the trajectory containing waypoints from the SDF element.
   * @param trajSdf Pointer to the SDF element containing the trajectory data.
   * @return Pointer to the created PoseAnimation object.
   */
  common::PoseAnimation* LoadTrajectory(sdf::ElementPtr trajSdf);

  physics::ModelPtr model;  ///< Pointer to the model
  event::ConnectionPtr updateConnection;  ///< Pointer to the update event connection
};

}  // namespace gazebo

#endif // ANIMATED_BOX_HPP
