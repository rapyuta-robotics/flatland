#include "flatland_plugins/base_model_drive.h"

#include <flatland_server/exceptions.h>
#include <flatland_server/model.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <cmath>
#include <regex>

namespace flatland_plugins {

namespace {

constexpr const char* kLog = "BaseModelDrive";
constexpr double kStepWarnFraction = 0.01;

std::string ResolveParamsPath(const std::string& params_file,
                              const std::string& params_name) {
  if (params_file.empty() == params_name.empty()) {
    throw flatland_server::YAMLException(
        "BaseModelDrive needs exactly one of 'params_file' and 'params_name' "
        "in its plugin entry");
  }
  if (!params_file.empty()) return params_file;
  if (!std::regex_match(params_name, std::regex("[A-Za-z0-9._-]+")) ||
      params_name.find("..") != std::string::npos) {
    throw flatland_server::YAMLException("params_name " + params_name +
                                         " is not a plain parameter-set name");
  }
  const std::string pkg = ros::package::getPath("flatland_plugins");
  if (pkg.empty()) {
    throw flatland_server::YAMLException(
        "package flatland_plugins not found by rospack; use params_file");
  }
  return pkg + "/base_model_params/" + params_name + ".yaml";
}

}  // namespace

void BaseModelDrive::TwistCallback(const geometry_msgs::TwistConstPtr& msg_ptr) {
  const geometry_msgs::Twist& msg = *msg_ptr;
  if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.angular.z)) {
    ROS_WARN_THROTTLE_NAMED(5.0, kLog, "ignoring a command that is not finite");
    return;
  }
  twist_msg_ = msg;
}

void BaseModelDrive::OnInitialize(const YAML::Node& config) {
  for (const char* forbidden : {"linear_dynamics", "angular_dynamics"}) {
    if (config[forbidden].IsDefined()) {
      throw flatland_server::YAMLException(
          std::string("BaseModelDrive refuses '") + forbidden +
          "': the parameter file carries the dynamics, and limiting the "
          "command twice is a configuration error");
    }
  }
  flatland_server::YamlReader reader(config);
  enable_odom_pub_ = reader.Get<bool>("enable_odom_pub", true);
  enable_ground_truth_pub_ = reader.Get<bool>("enable_ground_truth_pub", true);
  enable_twist_pub_ = reader.Get<bool>("enable_twist_pub", true);
  enable_tf_pub_ = reader.Get<bool>("enable_tf_pub", true);
  const std::string body_name = reader.Get<std::string>("body");
  const std::string odom_frame_id =
      reader.Get<std::string>("odom_frame_id", "odom");
  const std::string twist_topic =
      reader.Get<std::string>("twist_sub", "cmd_vel");
  const std::string odom_topic =
      reader.Get<std::string>("odom_pub", "odometry/filtered");
  const std::string ground_truth_topic =
      reader.Get<std::string>("ground_truth_pub", "odometry/ground_truth");
  const std::string twist_pub_topic =
      reader.Get<std::string>("twist_pub", "twist");
  const std::string provenance_topic =
      reader.Get<std::string>("provenance_pub", "base_model/provenance");
  const std::string params_file = reader.Get<std::string>("params_file", "");
  const std::string params_name = reader.Get<std::string>("params_name", "");

  const std::vector<double> odom_twist_noise =
      reader.GetList<double>("odom_twist_noise", {0, 0, 0}, 3, 3);
  const std::vector<double> odom_pose_noise =
      reader.GetList<double>("odom_pose_noise", {0, 0, 0}, 3, 3);
  const double pub_rate =
      reader.Get<double>("pub_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(pub_rate);

  std::array<double, 36> twist_covar_default = {0};
  twist_covar_default[0] = odom_twist_noise[0];
  twist_covar_default[7] = odom_twist_noise[1];
  twist_covar_default[35] = odom_twist_noise[2];
  std::array<double, 36> pose_covar_default = {0};
  pose_covar_default[0] = odom_pose_noise[0];
  pose_covar_default[7] = odom_pose_noise[1];
  pose_covar_default[35] = odom_pose_noise[2];
  const auto odom_twist_covar =
      reader.GetArray<double, 36>("odom_twist_covariance", twist_covar_default);
  const auto odom_pose_covar =
      reader.GetArray<double, 36>("odom_pose_covariance", pose_covar_default);

  reader.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(body_name) + " does not exist");
  }

  const std::string path = ResolveParamsPath(params_file, params_name);
  flatland_plugins::base_model::ParamFile file;
  try {
    file = flatland_plugins::base_model::LoadParamFile(path);
  } catch (const flatland_plugins::base_model::ParamFileError& e) {
    throw flatland_server::YAMLException(std::string("BaseModelDrive: ") +
                                         e.what());
  }
  shaper_.reset(new flatland_plugins::base_model::Shaper(file.params));
  calibrated_step_s_ = file.calibrated_step_s;
  const std::string description = flatland_plugins::base_model::Describe(file);
  ROS_INFO_NAMED(kLog, "parameter set %s from %s\n%s", file.name.c_str(),
                 path.c_str(), description.c_str());

  // Commands are taken from a queue that only the step loop drains: a command
  // published before a step is used from that step on, whatever the host load,
  // and the callback never runs beside the step.
  ros::SubscribeOptions twist_options = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      twist_topic, 1, boost::bind(&BaseModelDrive::TwistCallback, this, _1),
      ros::VoidConstPtr(), &twist_queue_);
  twist_sub_ = nh_.subscribe(twist_options);
  if (enable_odom_pub_)
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 1);
  if (enable_ground_truth_pub_)
    ground_truth_pub_ =
        nh_.advertise<nav_msgs::Odometry>(ground_truth_topic, 1);
  if (enable_twist_pub_)
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_pub_topic, 1);
  provenance_pub_ = nh_.advertise<std_msgs::String>(provenance_topic, 1, true);
  std_msgs::String provenance;
  provenance.data = description;
  provenance_pub_.publish(provenance);

  ground_truth_msg_.header.frame_id =
      tf::resolve("", GetModel()->NameSpaceTF(odom_frame_id));
  ground_truth_msg_.child_frame_id =
      tf::resolve("", GetModel()->NameSpaceTF(body_->name_));
  ground_truth_msg_.twist.covariance.fill(0);
  ground_truth_msg_.pose.covariance.fill(0);
  odom_msg_ = ground_truth_msg_;
  for (unsigned int i = 0; i < 36; i++) {
    odom_msg_.twist.covariance[i] = odom_twist_covar[i];
    odom_msg_.pose.covariance[i] = odom_pose_covar[i];
  }

  std::random_device rd;
  rng_ = std::default_random_engine(rd());
  for (unsigned int i = 0; i < 3; i++) {
    noise_gen_[i] =
        std::normal_distribution<double>(0.0, std::sqrt(odom_pose_noise[i]));
    noise_gen_[i + 3] =
        std::normal_distribution<double>(0.0, std::sqrt(odom_twist_noise[i]));
  }
}

void BaseModelDrive::BeforePhysicsStep(
    const flatland_server::Timekeeper& timekeeper) {
  twist_queue_.callAvailable();
  const double now = timekeeper.GetSimTime().toSec();
  const double dt = timekeeper.GetStepSize();
  if (started_ && now < last_sim_time_ - 1e-9) {
    // simulated time went backwards: the run was restarted, so nothing of the
    // old motion carries over
    shaper_->Reset();
    twist_msg_ = geometry_msgs::Twist();
    ROS_WARN_NAMED(kLog,
                   "simulated time went back (%.3f s to %.3f s): state reset",
                   last_sim_time_, now);
  }
  started_ = true;
  last_sim_time_ = now;
  if (!warned_step_ && calibrated_step_s_ > 0.0 &&
      std::fabs(dt - calibrated_step_s_) >
          kStepWarnFraction * calibrated_step_s_) {
    warned_step_ = true;
    ROS_WARN_NAMED(kLog,
                   "step size %.4f s differs from the calibrated %.4f s by "
                   "more than 1%%: the delay is rounded "
                   "to whole steps and the fit is not valid here",
                   dt, calibrated_step_s_);
  }
  publish_pending_ = update_timer_.CheckUpdate(timekeeper);

  double v = 0.0, w = 0.0;
  shaper_->Step(twist_msg_.linear.x, twist_msg_.angular.z, dt, v, w);

  // One rigid-body motion for the whole model. Box2D advances every centre by
  // c += dt * v (explicit Euler) and every angle by dt * w, so a velocity of
  // V_o + w x r would move the bodies off the rigid motion by w^2 r dt^2 / 2
  // per step, and the weld joints would correct that by dragging the base. The
  // velocity of each centre is therefore the one that lands it exactly on the
  // rigid pose the base reaches in this step.
  b2Body* base = body_->physics_body_;
  const double theta = base->GetAngle();
  const double dtheta = dt * w;
  const double cos_d = std::cos(dtheta), sin_d = std::sin(dtheta);
  const b2Vec2 origin = base->GetPosition();
  const b2Vec2 base_centre = base->GetWorldCenter();
  const b2Vec2 local_centre = base->GetLocalCenter();
  // velocity of the base centre: the command at the origin plus w x r
  const double ox = std::cos(theta) * v, oy = std::sin(theta) * v;
  const double bcx = ox - w * (base_centre.y - origin.y);
  const double bcy = oy + w * (base_centre.x - origin.x);
  // displacement of the base origin over the step: the centre moves by
  // dt * (bcx, bcy) and the origin sits at R(theta) * local_centre from it
  const double c_old = std::cos(theta), s_old = std::sin(theta);
  const double c_new = std::cos(theta + dtheta), s_new = std::sin(theta + dtheta);
  const double lx = local_centre.x, ly = local_centre.y;
  const double dox = dt * bcx - ((c_new - c_old) * lx - (s_new - s_old) * ly);
  const double doy = dt * bcy - ((s_new - s_old) * lx + (c_new - c_old) * ly);
  for (flatland_server::ModelBody* body : GetModel()->bodies_) {
    b2Body* b = body->physics_body_;
    const double rx = static_cast<double>(b->GetWorldCenter().x) - origin.x;
    const double ry = static_cast<double>(b->GetWorldCenter().y) - origin.y;
    // c' - c = (O' - O) + (R(dtheta) - I) r
    const double dx = dox + (cos_d - 1.0) * rx - sin_d * ry;
    const double dy = doy + sin_d * rx + (cos_d - 1.0) * ry;
    b->SetLinearVelocity(
        b2Vec2(static_cast<float>(dx / dt), static_cast<float>(dy / dt)));
    b->SetAngularVelocity(static_cast<float>(w));
  }
}

void BaseModelDrive::AfterPhysicsStep(
    const flatland_server::Timekeeper& timekeeper) {
  if (publish_pending_) {
    publish_pending_ = false;
    Publish(timekeeper);
  }
}

void BaseModelDrive::Publish(const flatland_server::Timekeeper& timekeeper) {
  // flatland steps the clock before AfterPhysicsStep, so GetSimTime() is the
  // time of the state published here.
  b2Body* b2body = body_->physics_body_;
  const b2Vec2 position = b2body->GetPosition();
  const float angle = b2body->GetAngle();
  const b2Vec2 linear_vel_local = b2body->GetLocalVector(
      b2body->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0)));
  const float angular_vel = b2body->GetAngularVelocity();

  ground_truth_msg_.header.stamp = timekeeper.GetSimTime();
  ground_truth_msg_.pose.pose.position.x = position.x;
  ground_truth_msg_.pose.pose.position.y = position.y;
  ground_truth_msg_.pose.pose.position.z = 0;
  ground_truth_msg_.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(angle);
  ground_truth_msg_.twist.twist.linear.x = linear_vel_local.x;
  ground_truth_msg_.twist.twist.linear.y = linear_vel_local.y;
  ground_truth_msg_.twist.twist.linear.z = 0;
  ground_truth_msg_.twist.twist.angular.x = 0;
  ground_truth_msg_.twist.twist.angular.y = 0;
  ground_truth_msg_.twist.twist.angular.z = angular_vel;

  odom_msg_.header.stamp = timekeeper.GetSimTime();
  odom_msg_.pose.pose = ground_truth_msg_.pose.pose;
  odom_msg_.twist.twist = ground_truth_msg_.twist.twist;
  odom_msg_.pose.pose.position.x += noise_gen_[0](rng_);
  odom_msg_.pose.pose.position.y += noise_gen_[1](rng_);
  odom_msg_.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(angle + noise_gen_[2](rng_));
  odom_msg_.twist.twist.linear.x += noise_gen_[3](rng_);
  odom_msg_.twist.twist.linear.y = 0;  // differential drive
  odom_msg_.twist.twist.angular.z += noise_gen_[5](rng_);

  if (enable_odom_pub_) odom_pub_.publish(odom_msg_);
  if (enable_ground_truth_pub_) ground_truth_pub_.publish(ground_truth_msg_);

  if (enable_twist_pub_) {
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = timekeeper.GetSimTime();
    msg.header.frame_id = odom_msg_.child_frame_id;
    // as stock DiffDrive computes it
    msg.twist.linear.x = std::cos(angle) * linear_vel_local.x +
                         std::sin(angle) * linear_vel_local.y +
                         noise_gen_[3](rng_);
    msg.twist.angular.z = angular_vel + noise_gen_[5](rng_);
    twist_pub_.publish(msg);
  }

  if (enable_tf_pub_) {
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header = odom_msg_.header;
    odom_tf.child_frame_id = odom_msg_.child_frame_id;
    odom_tf.transform.translation.x = odom_msg_.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg_.pose.pose.position.y;
    odom_tf.transform.translation.z = 0;
    odom_tf.transform.rotation = odom_msg_.pose.pose.orientation;
    tf_broadcaster_.sendTransform(odom_tf);
  }
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::BaseModelDrive,
                       flatland_server::ModelPlugin)
