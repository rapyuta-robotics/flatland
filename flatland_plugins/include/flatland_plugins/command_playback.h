// CommandPlayback: a test-only model plugin that replays recorded drives inside
// flatland's own step loop and records the ground-truth pose of the model's
// body. It removes ROS delivery from the fidelity gates: commands are published
// from `BeforePhysicsStep` of the same node, so they reach the drive plugin's
// subscriber through intra-process delivery and are applied at the next step,
// whatever the simulation speed. The same plugin drives BaseModelDrive and
// stock DiffDrive.
//
// Keys: `playback_file` (schedule, see playback.h; empty disables the plugin),
// `playback_out` (directory for the track files), `body` (default base_link),
// `cmd_topic` (default cmd_vel, in the model namespace).
#ifndef FLATLAND_PLUGINS_COMMAND_PLAYBACK_H
#define FLATLAND_PLUGINS_COMMAND_PLAYBACK_H

#include <flatland_server/model_body.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <string>
#include <vector>

#include "flatland_plugins/command_playback_schedule.h"

namespace flatland_plugins {

class CommandPlayback : public flatland_server::ModelPlugin {
 public:
  void OnInitialize(const YAML::Node& config) override;
  void BeforePhysicsStep(
      const flatland_server::Timekeeper& timekeeper) override;
  void AfterPhysicsStep(const flatland_server::Timekeeper& timekeeper) override;

 private:
  void Finish();
  void Publish(double v, double w);

  // Schedule time 0 is the clock at the first step: the timekeeper starts at the wall time, not at 0.
  // Differences are taken on ros::Time (integer nanoseconds), so they stay exact.
  double Elapsed(const flatland_server::Timekeeper& timekeeper) const;

  bool active_ = false;
  bool have_origin_ = false;
  ros::Time origin_;
  flatland_server::ModelBody* body_ = nullptr;
  ros::Publisher pub_;
  std::string out_dir_;
  std::vector<flatland_plugins::base_model::PlaybackDrive> drives_;
  int current_ = -1;
  size_t finished_ = 0;
  bool shutdown_requested_ = false;
  std::vector<flatland_plugins::base_model::PoseSample> samples_;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_COMMAND_PLAYBACK_H
