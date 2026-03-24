/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 body.cpp
 * @brief	 implements flatland body
 * @author Chunshang Li
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

#include <flatland_server/body.h>
#include <ros/ros.h>

namespace flatland_server {

Body::Body(b2WorldId physics_world, Entity *entity, const std::string &name,
           const Color &color, const Pose &pose, b2BodyType body_type,
           const YAML::Node &properties, double linear_damping,
           double angular_damping)
    : entity_(entity), name_(name), color_(color), properties_(properties) {
  b2BodyDef body_def = b2DefaultBodyDef();
  body_def.type = body_type;
  body_def.position = {static_cast<float>(pose.x), static_cast<float>(pose.y)};
  body_def.rotation = b2MakeRot(static_cast<float>(pose.theta));
  body_def.linearDamping = static_cast<float>(linear_damping);
  body_def.angularDamping = static_cast<float>(angular_damping);

  physics_body_ = b2CreateBody(physics_world, &body_def);
  b2Body_SetUserData(physics_body_, this);
}

Body::~Body() {
  if (b2Body_IsValid(physics_body_)) {
    b2DestroyBody(physics_body_);
  }
}

int Body::GetShapesCount() const {
  return b2Body_GetShapeCount(physics_body_);
}

Entity *Body::GetEntity() { return entity_; }

const std::string &Body::GetName() const { return name_; }

b2BodyId Body::GetPhysicsBody() { return physics_body_; }

const Color &Body::GetColor() const { return color_; }

void Body::SetColor(const Color &color) { color_ = color; }

void Body::DebugOutput() const {
  b2Vec2 pos = b2Body_GetPosition(physics_body_);
  float angle = b2Rot_GetAngle(b2Body_GetRotation(physics_body_));
  ROS_DEBUG_NAMED(
      "Body",
      "Body %p: entity(%p, %s) name(%s) color(%f,%f,%f,%f) "
      "num_shapes(%d) type(%d) pose(%f, %f, %f) "
      "angular_damping(%f) linear_damping(%f)",
      this, entity_, entity_->name_.c_str(), name_.c_str(), color_.r, color_.g,
      color_.b, color_.a, GetShapesCount(),
      static_cast<int>(b2Body_GetType(physics_body_)),
      pos.x, pos.y, angle,
      b2Body_GetAngularDamping(physics_body_),
      b2Body_GetLinearDamping(physics_body_));
}

};  // namespace flatland_server
