/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   debug_visualization.cpp
 * @brief  Transform box2d types into published visualization messages
 * @author Joseph Duchesne
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

#include "flatland_server/debug_visualization.h"
#include <box2d/box2d.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <map>
#include <string>

namespace flatland_server {

DebugVisualization::DebugVisualization() : node_("~debug") {
  topic_list_publisher_ =
      node_.advertise<flatland_msgs::DebugTopicList>("topics", 0, true);
}

DebugVisualization& DebugVisualization::Get() {
  static DebugVisualization instance;
  return instance;
}

void DebugVisualization::JointToMarkers(
    visualization_msgs::MarkerArray& markers, b2JointId joint, float r,
    float g, float b, float a) {
  b2JointType jtype = b2Joint_GetType(joint);
  if (jtype == b2_distanceJoint || jtype == b2_mouseJoint) {
    ROS_ERROR_NAMED("DebugVis",
                    "Unimplemented visualization joints. See b2World.cpp for "
                    "implementation");
    return;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.type = marker.LINE_LIST;
  marker.scale.x = 0.01;

  b2BodyId bodyA = b2Joint_GetBodyA(joint);
  b2BodyId bodyB = b2Joint_GetBodyB(joint);
  b2Vec2 posA = b2Body_GetPosition(bodyA);
  b2Vec2 posB = b2Body_GetPosition(bodyB);

  // Compute world anchor points from local anchors (weld joint only for now)
  b2Vec2 localAnchorA = b2Joint_GetLocalAnchorA(joint);
  b2Vec2 localAnchorB = b2Joint_GetLocalAnchorB(joint);
  b2Vec2 worldAnchorA = b2TransformPoint(b2Body_GetTransform(bodyA), localAnchorA);
  b2Vec2 worldAnchorB = b2TransformPoint(b2Body_GetTransform(bodyB), localAnchorB);

  geometry_msgs::Point p_bodyA, p_anchorA, p_bodyB, p_anchorB;
  p_bodyA.x = posA.x; p_bodyA.y = posA.y;
  p_anchorA.x = worldAnchorA.x; p_anchorA.y = worldAnchorA.y;
  p_bodyB.x = posB.x; p_bodyB.y = posB.y;
  p_anchorB.x = worldAnchorB.x; p_anchorB.y = worldAnchorB.y;

  // Lines: bodyA->anchorA, bodyB->anchorB, anchorA->anchorB
  marker.id = markers.markers.size();
  marker.points.push_back(p_bodyA);
  marker.points.push_back(p_anchorA);
  marker.points.push_back(p_bodyB);
  marker.points.push_back(p_anchorB);
  marker.points.push_back(p_anchorA);
  marker.points.push_back(p_anchorB);

  markers.markers.push_back(marker);

  marker.id = markers.markers.size();
  marker.type = marker.CUBE_LIST;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.03;
  marker.points.clear();
  marker.points.push_back(p_anchorA);
  marker.points.push_back(p_anchorB);
  marker.points.push_back(p_bodyA);
  marker.points.push_back(p_bodyB);
  markers.markers.push_back(marker);
}

void DebugVisualization::BodyToMarkers(visualization_msgs::MarkerArray& markers,
                                       b2BodyId body, float r, float g,
                                       float b, float a) {
  int shape_count = b2Body_GetShapeCount(body);
  if (shape_count == 0) return;

  std::vector<b2ShapeId> shapes(shape_count);
  b2Body_GetShapes(body, shapes.data(), shape_count);

  b2Vec2 pos = b2Body_GetPosition(body);
  float angle = b2Rot_GetAngle(b2Body_GetRotation(body));

  for (int si = 0; si < shape_count; si++) {
    b2ShapeId shape = shapes[si];
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.id = markers.markers.size();
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.pose.position.x = pos.x;
    marker.pose.position.y = pos.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    marker.pose.orientation = tf2::toMsg(q);
    bool add_marker = true;

    b2ShapeType stype = b2Shape_GetType(shape);
    if (stype == b2_circleShape) {
      b2Circle circle = b2Shape_GetCircle(shape);

      marker.type = marker.SPHERE_LIST;
      float diameter = circle.radius * 2.0f;
      marker.scale.z = 0.01;
      marker.scale.x = diameter;
      marker.scale.y = diameter;

      geometry_msgs::Point p;
      p.x = circle.center.x;
      p.y = circle.center.y;
      marker.points.push_back(p);

    } else if (stype == b2_polygonShape) {
      b2Polygon poly = b2Shape_GetPolygon(shape);
      marker.type = marker.LINE_STRIP;
      marker.scale.x = 0.03;

      for (int i = 0; i < poly.count; i++) {
        geometry_msgs::Point p;
        p.x = poly.vertices[i].x;
        p.y = poly.vertices[i].y;
        marker.points.push_back(p);
      }
      if (poly.count > 0) marker.points.push_back(marker.points[0]);

    } else if (stype == b2_segmentShape) {
      b2Segment seg = b2Shape_GetSegment(shape);
      geometry_msgs::Point p;

      // If the last marker is a line list, extend it
      if (markers.markers.size() > 0 &&
          markers.markers.back().type == marker.LINE_LIST) {
        add_marker = false;
        p.x = seg.point1.x; p.y = seg.point1.y;
        markers.markers.back().points.push_back(p);
        p.x = seg.point2.x; p.y = seg.point2.y;
        markers.markers.back().points.push_back(p);
      } else {
        marker.type = marker.LINE_LIST;
        marker.scale.x = 0.03;
        p.x = seg.point1.x; p.y = seg.point1.y;
        marker.points.push_back(p);
        p.x = seg.point2.x; p.y = seg.point2.y;
        marker.points.push_back(p);
      }

    } else {
      ROS_WARN_THROTTLE_NAMED(1.0, "DebugVis", "Unsupported Box2D shape type %d",
                              static_cast<int>(stype));
      continue;
    }

    if (add_marker) {
      markers.markers.push_back(marker);
    }
  }
}

void DebugVisualization::Publish(const Timekeeper& timekeeper) {
  // Iterate over the topics_ map as pair(name, topic)

  std::vector<std::string> to_delete;

  for (auto& topic : topics_) {
    if (!topic.second.needs_publishing) {
      continue;
    }

    // since if empty markers are published rviz will continue to publish
    // using the old data, delete the topic list
    if (topic.second.markers.markers.size() == 0) {
      to_delete.push_back(topic.first);
    } else {
      // Iterate the marker array to update all the timestamps
      for (unsigned int i = 0; i < topic.second.markers.markers.size(); i++) {
        topic.second.markers.markers[i].header.stamp = timekeeper.GetSimTime();
      }
      topic.second.publisher.publish(topic.second.markers);
      topic.second.needs_publishing = false;
    }
  }

  if (to_delete.size() > 0) {
    for (const auto& topic : to_delete) {
      ROS_WARN_NAMED("DebugVis", "Deleting topic %s", topic.c_str());
      topics_.erase(topic);
    }
    PublishTopicList();
  }
}

void DebugVisualization::VisualizeLayer(std::string name, Body* body) {
  AddTopicIfNotExist(name);

  int shape_count = b2Body_GetShapeCount(body->physics_body_);

  visualization_msgs::Marker marker;
  if (shape_count == 0) return;

  std::vector<b2ShapeId> shapes(shape_count);
  b2Body_GetShapes(body->physics_body_, shapes.data(), shape_count);

  for (int si = 0; si < shape_count; si++) {
    b2ShapeId shape = shapes[si];
    if (b2Shape_GetType(shape) != b2_segmentShape) continue;  // layer only has segments
    b2Segment seg = b2Shape_GetSegment(shape);

    marker.header.frame_id = "map";
    marker.id = topics_[name].markers.markers.size();
    marker.color.r = body->color_.r;
    marker.color.g = body->color_.g;
    marker.color.b = body->color_.b;
    marker.color.a = body->color_.a;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.frame_locked = true;
    marker.pose.position.x = b2Body_GetPosition(body->physics_body_).x;
    marker.pose.position.y = b2Body_GetPosition(body->physics_body_).y;

    tf2::Quaternion q;
    q.setRPY(0, 0, b2Rot_GetAngle(b2Body_GetRotation(body->physics_body_)));
    marker.pose.orientation = tf2::toMsg(q);
    marker.type = marker.TRIANGLE_LIST;
    marker.points.clear();

    YamlReader reader(body->properties_);
    YamlReader debug_reader =
        reader.SubnodeOpt("debug", YamlReader::NodeTypeCheck::MAP);
    float min_z = debug_reader.Get<float>("min_z", 0.0);
    float max_z = debug_reader.Get<float>("max_z", 1.0);

    geometry_msgs::Point p;
    p.x = seg.point1.x; p.y = seg.point1.y; p.z = min_z;
    marker.points.push_back(p);
    p.x = seg.point2.x; p.y = seg.point2.y; p.z = min_z;
    marker.points.push_back(p);
    p.x = seg.point2.x; p.y = seg.point2.y; p.z = max_z;
    marker.points.push_back(p);
    p.x = seg.point1.x; p.y = seg.point1.y; p.z = min_z;
    marker.points.push_back(p);
    p.x = seg.point2.x; p.y = seg.point2.y; p.z = max_z;
    marker.points.push_back(p);
    p.x = seg.point1.x; p.y = seg.point1.y; p.z = max_z;
    marker.points.push_back(p);

    topics_[name].markers.markers.push_back(marker);
  }
  topics_[name].needs_publishing = true;
}

void DebugVisualization::Visualize(std::string name, b2BodyId body, float r,
                                   float g, float b, float a) {
  AddTopicIfNotExist(name);
  BodyToMarkers(topics_[name].markers, body, r, g, b, a);
  topics_[name].needs_publishing = true;
}

void DebugVisualization::Visualize(std::string name, b2JointId joint, float r,
                                   float g, float b, float a) {
  AddTopicIfNotExist(name);
  JointToMarkers(topics_[name].markers, joint, r, g, b, a);
  topics_[name].needs_publishing = true;
}

void DebugVisualization::Reset(std::string name) {
  if (topics_.count(name) > 0) {  // If the topic exists, clear it
    topics_[name].markers.markers.clear();
    topics_[name].needs_publishing = true;
  }
}

void DebugVisualization::AddTopicIfNotExist(const std::string& name) {
  // If the topic doesn't exist yet, create it
  if (topics_.count(name) == 0) {
    topics_[name] = {
        node_.advertise<visualization_msgs::MarkerArray>(name, 0, true), true,
        visualization_msgs::MarkerArray()};

    ROS_INFO_ONCE_NAMED("DebugVis", "Visualizing %s", name.c_str());
    PublishTopicList();
  }
}

void DebugVisualization::PublishTopicList() {
  flatland_msgs::DebugTopicList topic_list;
  for (auto const& topic_pair : topics_)
    topic_list.topics.push_back(topic_pair.first);
  topic_list_publisher_.publish(topic_list);
}
};  // namespace flatland_server
