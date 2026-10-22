#include "flatland_plugins/command_playback.h"

#include <flatland_server/exceptions.h>
#include <flatland_server/geometry.h>
#include <flatland_server/model.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>

namespace flatland_plugins {

namespace {
constexpr const char* kLog = "CommandPlayback";
}

void CommandPlayback::OnInitialize(const YAML::Node& config) {
  flatland_server::YamlReader reader(config);
  const std::string file = reader.Get<std::string>("playback_file", "");
  out_dir_ = reader.Get<std::string>("playback_out", "");
  const std::string body_name = reader.Get<std::string>("body", "base_link");
  const std::string topic = reader.Get<std::string>("cmd_topic", "cmd_vel");
  reader.EnsureAccessedAllKeys();
  if (file.empty()) {
    ROS_INFO_NAMED(kLog, "no playback_file: playback is off");
    return;
  }
  if (out_dir_.empty())
    throw flatland_server::YAMLException(
        "CommandPlayback: playback_file needs playback_out");
  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(body_name) + " does not exist");
  }
  try {
    drives_ = flatland_plugins::base_model::ParseSchedule(YAML::LoadFile(file));
  } catch (const std::exception& e) {
    throw flatland_server::YAMLException(
        std::string("CommandPlayback: cannot read ") + file + ": " + e.what());
  }
  pub_ = nh_.advertise<geometry_msgs::Twist>(topic, 1);
  active_ = true;
  ROS_INFO_NAMED(kLog, "playing %zu drive(s) from %s into %s", drives_.size(),
                 file.c_str(), out_dir_.c_str());
}

void CommandPlayback::Publish(double v, double w) {
  geometry_msgs::Twist m;
  m.linear.x = v;
  m.angular.z = w;
  pub_.publish(m);
}

double CommandPlayback::Elapsed(
    const flatland_server::Timekeeper& timekeeper) const {
  return (timekeeper.GetSimTime() - origin_).toSec();
}

void CommandPlayback::BeforePhysicsStep(
    const flatland_server::Timekeeper& timekeeper) {
  if (!active_) return;
  if (!have_origin_) {
    origin_ = timekeeper.GetSimTime();
    have_origin_ = true;
  }
  const double now = Elapsed(timekeeper);
  const int idx = flatland_plugins::base_model::ActiveDrive(drives_, now);
  if (idx < 0) return;
  const flatland_plugins::base_model::PlaybackDrive& d = drives_[idx];
  if (idx != current_) {
    current_ = idx;
    samples_.clear();
    // every drive starts at the origin at rest, so the precision of Box2D's
    // floats does not depend on the order
    GetModel()->SetPose(flatland_server::Pose(0.0, 0.0, 0.0));
    const b2Body* b = body_->physics_body_;
    samples_.push_back(
        {0.0, b->GetPosition().x, b->GetPosition().y, b->GetAngle()});
  }
  const flatland_plugins::base_model::PlaybackEvent c =
      flatland_plugins::base_model::CommandAt(d, now - d.start_s);
  Publish(c.v, c.w);
}

void CommandPlayback::AfterPhysicsStep(
    const flatland_server::Timekeeper& timekeeper) {
  if (!active_ || current_ < 0) return;
  const flatland_plugins::base_model::PlaybackDrive& d = drives_[current_];
  const double now =
      Elapsed(timekeeper);  // flatland steps the clock before this call
  const b2Body* b = body_->physics_body_;
  samples_.push_back(
      {now - d.start_s, b->GetPosition().x, b->GetPosition().y, b->GetAngle()});
  if (now >= d.start_s + d.length_s - 1e-9) Finish();
}

void CommandPlayback::Finish() {
  const flatland_plugins::base_model::PlaybackDrive& d = drives_[current_];
  Publish(0.0, 0.0);
  flatland_plugins::base_model::WriteTrack(out_dir_ + "/" + d.name + ".json",
                                           d.name, samples_);
  ROS_INFO_NAMED(kLog, "wrote track %s (%zu samples)", d.name.c_str(),
                 samples_.size());
  current_ = -1;
  if (++finished_ == drives_.size() && !shutdown_requested_) {
    shutdown_requested_ = true;
    ros::requestShutdown();
  }
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::CommandPlayback,
                       flatland_server::ModelPlugin)
