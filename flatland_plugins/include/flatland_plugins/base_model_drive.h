// BaseModelDrive: a flatland model plugin that drives like the real base. It
// replaces DiffDrive in a model file. The model (delay, lags, rate limits, yaw
// scale, caps) is in base_model_shaper.h; its parameters come from a
// parameter file (base_model_params.h).
#ifndef FLATLAND_PLUGINS_BASE_MODEL_DRIVE_H
#define FLATLAND_PLUGINS_BASE_MODEL_DRIVE_H

#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_body.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <array>
#include <memory>
#include <random>

#include "flatland_plugins/base_model_params.h"
#include "flatland_plugins/base_model_shaper.h"

namespace flatland_plugins {

class BaseModelDrive : public flatland_server::ModelPlugin {
 public:
  void OnInitialize(const YAML::Node& config) override;
  // Shapes the command and sets the same rigid-body motion on every body of the
  // model.
  void BeforePhysicsStep(
      const flatland_server::Timekeeper& timekeeper) override;
  // Publishes odometry and ground truth from the state after the step: the
  // twist is the integrated velocity.
  void AfterPhysicsStep(const flatland_server::Timekeeper& timekeeper) override;

  void TwistCallback(const geometry_msgs::TwistConstPtr& msg);

 private:
  void Publish(const flatland_server::Timekeeper& timekeeper);

  flatland_server::ModelBody* body_ = nullptr;
  std::unique_ptr<flatland_plugins::base_model::Shaper> shaper_;
  double calibrated_step_s_ = 0.0;
  bool warned_step_ = false;
  bool started_ = false;
  double last_sim_time_ = 0.0;
  bool publish_pending_ = false;

  ros::CallbackQueue twist_queue_;  // drained at the start of every step
  ros::Subscriber twist_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher ground_truth_pub_;
  ros::Publisher twist_pub_;
  ros::Publisher provenance_pub_;
  geometry_msgs::Twist twist_msg_;
  nav_msgs::Odometry odom_msg_;
  nav_msgs::Odometry ground_truth_msg_;
  flatland_plugins::UpdateTimer update_timer_;
  tf::TransformBroadcaster tf_broadcaster_;
  bool enable_odom_pub_ = true;
  bool enable_ground_truth_pub_ = true;
  bool enable_twist_pub_ = true;
  bool enable_tf_pub_ = true;

  std::default_random_engine rng_;
  std::array<std::normal_distribution<double>, 6> noise_gen_;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_BASE_MODEL_DRIVE_H
