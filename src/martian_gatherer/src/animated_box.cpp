/**
 * @file animated_box.cpp
 * @brief Gazebo plugin to animate a box along a predefined trajectory of
 * waypoints.
 *
 * Copyright 2024 Prathinav Karnala Venkata, Abhey Sharma, Sarang Nair.
 */

#include <stdio.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

namespace gazebo {
/**
 * @class AnimatedBox
 * @brief A Gazebo plugin that animates a box model along a predefined
 * trajectory using waypoints.
 */
class AnimatedBox : public ModelPlugin {
 public:
  /**
   * @brief Loads the plugin and sets the animation for the box.
   * @param _parent Pointer to the model that this plugin is attached to.
   * @param _sdf Pointer to the SDF element of the plugin.
   */
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;

    // Check if the trajectory element exists in the SDF file
    if (!_sdf->HasElement("trajectory")) {
      std::cerr << "AnimatedBox plugin: No trajectory defined." << std::endl;
      return;
    }

    // Load the trajectory containing waypoints
    common::PoseAnimation *animPtr =
        this->LoadTrajectory(_sdf->GetElement("trajectory"));

    // Check if trajectory loading was successful
    if (animPtr == NULL) {
      std::cerr << "AnimatedBox plugin: No waypoints defined." << std::endl;
      return;
    }

    // Set the animation to the model
    gazebo::common::PoseAnimationPtr anim(animPtr);
    _parent->SetAnimation(anim);

    std::cerr << "AnimatedBox plugin is loaded" << std::endl;
  }

 private:
  physics::ModelPtr model;  ///< Pointer to the model
  event::ConnectionPtr
      updateConnection;  ///< Pointer to the update event connection

  /**
   * @brief Loads the trajectory containing waypoints from the SDF element.
   * @param trajSdf Pointer to the SDF element containing the trajectory data.
   * @return Pointer to the created PoseAnimation object.
   */
  common::PoseAnimation *LoadTrajectory(sdf::ElementPtr trajSdf) {
    // Check if waypoints are defined in the SDF file
    if (!trajSdf->HasElement("waypoint")) {
      return NULL;
    }

    // Create a map to store the time and pose of each waypoint
    std::map<double, ignition::math::Pose3d> points;
    sdf::ElementPtr wayptSdf = trajSdf->GetElement("waypoint");

    while (wayptSdf) {
      points[wayptSdf->Get<double>("time")] =
          wayptSdf->Get<ignition::math::Pose3d>("pose");
      wayptSdf = wayptSdf->GetNextElement("waypoint");
    }

    // Get the total trajectory duration (last waypoint's time)
    auto last = points.rbegin();

    // Create the PoseAnimation object with the total duration and looping set
    // to true
    common::PoseAnimation *anim =
        new common::PoseAnimation("my_name", last->first, true);

    // Create keyframes for each waypoint
    for (auto pIter = points.begin(); pIter != points.end(); ++pIter) {
      common::PoseKeyFrame *key;

      // Ensure the first keyframe starts at 0s
      if (pIter == points.begin() &&
          !ignition::math::equal(pIter->first, 0.0)) {
        key = anim->CreateKeyFrame(0.0);
      } else {
        key = anim->CreateKeyFrame(pIter->first);
      }

      key->Translation(pIter->second.Pos());
      key->Rotation(pIter->second.Rot());
    }

    return anim;
  }
};

// Register this plugin with the Gazebo simulator
GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}  // namespace gazebo
