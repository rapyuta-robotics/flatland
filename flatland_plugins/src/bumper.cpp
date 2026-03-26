/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  bumper.h
 * @brief   Bumper plugin
 * @author  Chunshang Li
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <flatland_msgs/Collision.h>
#include <flatland_msgs/Collisions.h>
#include <flatland_plugins/bumper.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include <boost/algorithm/string/join.hpp>

using namespace flatland_server;

namespace flatland_plugins {

Bumper::ContactState::ContactState() { Reset(); }

void Bumper::ContactState::Reset() {
  num_count = 0;
  sum_speed = 0.0;
}

void Bumper::OnInitialize(const YAML::Node &config) {
  YamlReader reader(config);

  // defaults
  world_frame_id_ = reader.Get<std::string>("world_frame_id", "map");
  topic_name_ = reader.Get<std::string>("topic", "collisions");
  publish_all_collisions_ = reader.Get<bool>("publish_all_collisions", true);
  update_rate_ = reader.Get<double>("update_rate",
                                    std::numeric_limits<double>::infinity());

  std::vector<std::string> excluded_body_names =
      reader.GetList<std::string>("exclude", {}, -1, -1);

  reader.EnsureAccessedAllKeys();

  for (unsigned int i = 0; i < excluded_body_names.size(); i++) {
    Body *body = GetModel()->GetBody(excluded_body_names[i]);

    if (body == nullptr) {
      throw YAMLException("Body with name \"" + excluded_body_names[i] +
                          "\" does not exist");
    } else {
      excluded_bodies_.push_back(body);
    }
  }

  update_timer_.SetRate(update_rate_);
  collisions_publisher_ =
      nh_.advertise<flatland_msgs::Collisions>(topic_name_, 1);

  ROS_DEBUG_NAMED("Bumper",
                  "Initialized with params: topic(%s) world_frame_id(%s) "
                  "publish_all_collisions(%d) update_rate(%f) exclude({%s})",
                  topic_name_.c_str(), world_frame_id_.c_str(),
                  publish_all_collisions_, update_rate_,
                  boost::algorithm::join(excluded_body_names, ",").c_str());
}

void Bumper::BeforePhysicsStep(const Timekeeper &timekeeper) {
  // Clear the forces at the beginning of every physics step
  for (auto &kv : contact_states_) {
    kv.second.Reset();
  }
}

void Bumper::AfterPhysicsStep(const Timekeeper &timekeeper) {
  // The expected behaviour is to always publish non-empty collisions unless
  // publish_all_collisions set to false. The publishing of empty collision
  // manages the publishing rate of empty collisions when
  // publish_all_collisions is true, or it manages the publishing rate all
  // empty and non-empty collisions when publish_all_collisions_ is false
  if (!publish_all_collisions_ || contact_states_.size() <= 0) {
    if (!update_timer_.CheckUpdate(timekeeper)) {
      return;
    }
  }

  flatland_msgs::Collisions collisions;
  collisions.header.frame_id = world_frame_id_;
  collisions.header.stamp = timekeeper.GetSimTime();

  // loop through all collisions in our record and publish
  for (auto &kv : contact_states_) {
    const ContactState &s = kv.second;
    flatland_msgs::Collision collision;
    collision.entity_A = GetModel()->GetName();
    collision.entity_B = s.entity_B->name_;

    collision.body_A = s.body_A->name_;
    collision.body_B = s.body_B->name_;

    // If there was a hit event, publish the contact force estimate
    if (s.num_count > 0) {
      double ave_speed = s.sum_speed / s.num_count;
      // approachSpeed in m/s; record as a magnitude force proxy
      collision.magnitude_forces.push_back(ave_speed);
      flatland_msgs::Vector2 point;
      flatland_msgs::Vector2 normal;
      point.x = s.point.x;
      point.y = s.point.y;
      normal.x = s.normal.x;
      normal.y = s.normal.y;
      collision.contact_positions.push_back(point);
      collision.contact_normals.push_back(normal);
    }

    collisions.collisions.push_back(collision);
  }

  collisions_publisher_.publish(collisions);
}

void Bumper::BeginContact(b2ShapeId shapeIdA, b2ShapeId shapeIdB) {
  Entity *other_entity;
  b2BodyId this_body, other_body;
  if (!FilterContact(shapeIdA, shapeIdB, other_entity, this_body,
                     other_body)) {
    return;
  }

  ContactKey key = {shapeIdA, shapeIdB};

  // If this is a new contact, add it to the records of alive contacts
  if (!contact_states_.count(key)) {
    Body *collision_body =
        static_cast<Body *>(b2Body_GetUserData(this_body));

    bool ignore = false;

    // check that the body is not in the ignore list
    for (unsigned int j = 0; j < excluded_bodies_.size(); j++) {
      if (excluded_bodies_[j] == collision_body) {
        ignore = true;
        break;
      }
    }

    // add the body to the record of active contacts
    if (!ignore) {
      contact_states_[key] = ContactState();
      ContactState *c = &contact_states_[key];
      c->entity_B = other_entity;
      c->body_B = static_cast<Body *>(b2Body_GetUserData(other_body));
      c->body_A = collision_body;
    }
  }
}

void Bumper::EndContact(b2ShapeId shapeIdA, b2ShapeId shapeIdB) {
  if (!FilterContact(shapeIdA, shapeIdB)) return;

  ContactKey key = {shapeIdA, shapeIdB};
  if (contact_states_.count(key)) {
    contact_states_.erase(key);
  }
}

void Bumper::OnContactHit(b2ShapeId shapeIdA, b2ShapeId shapeIdB,
                           b2Vec2 point, b2Vec2 normal, float approachSpeed) {
  if (!FilterContact(shapeIdA, shapeIdB)) return;

  ContactKey key = {shapeIdA, shapeIdB};
  if (!contact_states_.count(key)) return;  // ignored contact

  ContactState *state = &contact_states_[key];
  state->num_count++;
  state->sum_speed += approachSpeed;
  state->point = point;
  state->normal = normal;
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Bumper, flatland_server::ModelPlugin)
