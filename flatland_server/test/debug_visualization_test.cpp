/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	null.cpp
 * @brief	Sanity check / example test file
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
#include <flatland_server/timekeeper.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>

// Helper: create a minimal b2WorldId for unit tests.
static b2WorldId MakeTestWorld() {
  b2WorldDef wd = b2DefaultWorldDef();
  wd.gravity = {0.0f, 0.0f};
  return b2CreateWorld(&wd);
}

// Test the bodyToMarkers method on a polygon shape
TEST(DebugVizTest, testBodyToMarkersPolygon) {
  b2WorldId world = MakeTestWorld();

  b2BodyDef bodyDef = b2DefaultBodyDef();
  bodyDef.type = b2_dynamicBody;
  bodyDef.position = {3.0f, 4.0f};
  bodyDef.rotation = b2MakeRot(M_PI_2);
  b2BodyId body = b2CreateBody(world, &bodyDef);

  b2Polygon box = b2MakeBox(1.0f, 2.0f);
  b2ShapeDef shapeDef = b2DefaultShapeDef();
  shapeDef.density = 1.0f;
  shapeDef.friction = 0.3f;
  b2CreatePolygonShape(body, &shapeDef, &box);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::Get().BodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.5, 0.7);
  // check that marker was created
  ASSERT_EQ(markers.markers.size(), 1);

  // Check that
  ASSERT_EQ(markers.markers[0].header.frame_id, "map");
  ASSERT_EQ(markers.markers[0].header.stamp.sec, 0);
  ASSERT_EQ(markers.markers[0].header.stamp.nsec, 0);

  // Check color setting
  ASSERT_NEAR(markers.markers[0].color.r, 1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].color.g, 0.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].color.b, 0.5, 1e-5);
  ASSERT_NEAR(markers.markers[0].color.a, 0.7, 1e-5);

  // Check position
  ASSERT_NEAR(markers.markers[0].pose.position.x, 3.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.position.y, 4.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.position.z, 0.0, 1e-5);

  // Check orientation
  ASSERT_NEAR(markers.markers[0].pose.orientation.x, 0.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.orientation.y, 0.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.orientation.z, 0.70710678118, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.orientation.w, 0.70710678118, 1e-5);

  // Check the marker shape (box = 4 vertices + close = 5 points)
  ASSERT_EQ(markers.markers[0].type, markers.markers[0].LINE_STRIP);
  ASSERT_EQ(markers.markers[0].points.size(), 5);
  ASSERT_NEAR(markers.markers[0].points[0].x, -1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[0].y, -2.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].x, 1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].y, -2.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[2].x, 1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[2].y, 2.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[3].x, -1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[3].y, 2.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[4].x, -1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[4].y, -2.0, 1e-5);

  b2DestroyWorld(world);
}

// Test the bodyToMarkers method on a circle shape
TEST(DebugVizTest, testBodyToMarkersCircle) {
  b2WorldId world = MakeTestWorld();

  b2BodyDef bodyDef = b2DefaultBodyDef();
  bodyDef.type = b2_dynamicBody;
  bodyDef.position = {3.0f, 4.0f};
  bodyDef.rotation = b2MakeRot(M_PI_2);
  b2BodyId body = b2CreateBody(world, &bodyDef);

  b2Circle circle;
  circle.center = {2.0f, 3.0f};
  circle.radius = 0.2f;
  b2ShapeDef shapeDef = b2DefaultShapeDef();
  b2CreateCircleShape(body, &shapeDef, &circle);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::Get().BodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.0, 1.0);
  ASSERT_EQ(markers.markers.size(), 1);
  ASSERT_EQ(markers.markers[0].type, markers.markers[0].SPHERE_LIST);
  ASSERT_NEAR(markers.markers[0].scale.x, 0.4, 1e-5);
  ASSERT_NEAR(markers.markers[0].scale.y, 0.4, 1e-5);
  ASSERT_NEAR(markers.markers[0].scale.z, 0.01, 1e-5);

  b2DestroyWorld(world);
}

// Test the bodyToMarkers method on a segment (edge) shape
TEST(DebugVizTest, testBodyToMarkersEdge) {
  b2WorldId world = MakeTestWorld();

  b2BodyDef bodyDef = b2DefaultBodyDef();
  b2BodyId body = b2CreateBody(world, &bodyDef);

  b2Segment seg = {{0.5f, 1.5f}, {3.5f, 2.0f}};
  b2ShapeDef shapeDef = b2DefaultShapeDef();
  b2CreateSegmentShape(body, &shapeDef, &seg);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::Get().BodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.0, 1.0);
  ASSERT_EQ(markers.markers.size(), 1);
  ASSERT_EQ(markers.markers[0].type, markers.markers[0].LINE_LIST);
  ASSERT_EQ(markers.markers[0].points.size(), 2);
  ASSERT_NEAR(markers.markers[0].points[0].x, 0.5, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[0].y, 1.5, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].x, 3.5, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].y, 2.0, 1e-5);

  b2DestroyWorld(world);
}

// Test the bodyToMarkers method on an unsupported shape (body with no shapes)
TEST(DebugVizTest, testBodyToMarkersUnsupported) {
  b2WorldId world = MakeTestWorld();

  b2BodyDef bodyDef = b2DefaultBodyDef();
  b2BodyId body = b2CreateBody(world, &bodyDef);
  // No shapes added 窶・BodyToMarkers should produce no markers

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::Get().BodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.0, 1.0);
  ASSERT_EQ(markers.markers.size(), 0);

  b2DestroyWorld(world);
}

// test bodyToMarkers with a body with multiple segment shapes
TEST(DebugVizTest, testBodyToMarkersMultifixture) {
  b2WorldId world = MakeTestWorld();

  b2BodyDef bodyDef = b2DefaultBodyDef();
  b2BodyId body = b2CreateBody(world, &bodyDef);

  b2ShapeDef shapeDef = b2DefaultShapeDef();
  b2Segment edge1 = {{0.0f, 1.0f}, {1.0f, 2.0f}};
  b2Segment edge2 = {{-1.0f, 3.0f}, {5.0f, 7.0f}};
  b2CreateSegmentShape(body, &shapeDef, &edge1);
  b2CreateSegmentShape(body, &shapeDef, &edge2);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::Get().BodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.0, 1.0);
  // Both segments get combined into one LINE_LIST marker
  ASSERT_EQ(markers.markers.size(), 1);
  ASSERT_EQ(markers.markers[0].type, markers.markers[0].LINE_LIST);
  ASSERT_EQ(markers.markers[0].points.size(), 4);

  b2DestroyWorld(world);
}

// test bodyToMarkers with multiple bodies
TEST(DebugVizTest, testBodyToMarkersMultibody) {
  b2WorldId world = MakeTestWorld();

  b2BodyDef bodyDef = b2DefaultBodyDef();
  b2BodyId body = b2CreateBody(world, &bodyDef);
  b2BodyId body2 = b2CreateBody(world, &bodyDef);

  b2ShapeDef shapeDef = b2DefaultShapeDef();
  b2Segment edge1 = {{0.0f, 1.0f}, {1.0f, 2.0f}};
  b2Segment edge2 = {{-1.0f, 3.0f}, {5.0f, 7.0f}};
  b2CreateSegmentShape(body, &shapeDef, &edge1);
  b2CreateSegmentShape(body2, &shapeDef, &edge2);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::Get().BodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.0, 1.0);
  flatland_server::DebugVisualization::Get().BodyToMarkers(markers, body2, 1.0,
                                                           0.0, 0.0, 1.0);
  // Both bodies' segments extend the same LINE_LIST marker
  ASSERT_EQ(markers.markers.size(), 1);
  ASSERT_EQ(markers.markers[0].type, markers.markers[0].LINE_LIST);
  ASSERT_EQ(markers.markers[0].points.size(), 4);
  // First body edge
  ASSERT_NEAR(markers.markers[0].points[0].x, 0.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[0].y, 1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].x, 1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].y, 2.0, 1e-5);
  // Second body edge
  ASSERT_NEAR(markers.markers[0].points[2].x, -1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[2].y, 3.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[3].x, 5.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[3].y, 7.0, 1e-5);

  b2DestroyWorld(world);
}

// test JointToMarkers with multiple weld joints
TEST(DebugVizTest, testJointToMarkersMultiJoint) {
  b2WorldId world = MakeTestWorld();

  b2BodyDef bodyDef = b2DefaultBodyDef();
  b2BodyId b1 = b2CreateBody(world, &bodyDef);
  b2BodyId b2body = b2CreateBody(world, &bodyDef);

  b2WeldJointDef jd1 = b2DefaultWeldJointDef();
  jd1.bodyIdA = b1;
  jd1.bodyIdB = b2body;
  jd1.localAnchorA = {0.0f, 0.0f};
  jd1.localAnchorB = {0.0f, 0.0f};
  b2JointId j1 = b2CreateWeldJoint(world, &jd1);

  b2WeldJointDef jd2 = b2DefaultWeldJointDef();
  jd2.bodyIdA = b1;
  jd2.bodyIdB = b2body;
  jd2.localAnchorA = {1.0f, 2.0f};
  jd2.localAnchorB = {3.0f, 4.0f};
  b2JointId j2 = b2CreateWeldJoint(world, &jd2);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::Get().JointToMarkers(markers, j1, 0.1,
                                                            0.2, 0.3, 0.4);
  flatland_server::DebugVisualization::Get().JointToMarkers(markers, j2, 0.5,
                                                            0.6, 0.7, 0.8);
  ASSERT_EQ(markers.markers.size(), 4);

  // Check the 1st marker (j1: all anchors at origin)
  ASSERT_EQ(markers.markers[0].type, markers.markers[0].LINE_LIST);
  ASSERT_EQ(markers.markers[0].points.size(), 6);
  for (unsigned int i = 0; i < 6; i++) {
    ASSERT_FLOAT_EQ(markers.markers[0].points[i].x, 0.0) << "index: " << i;
    ASSERT_FLOAT_EQ(markers.markers[0].points[i].y, 0.0) << "index: " << i;
  }
  ASSERT_FLOAT_EQ(markers.markers[0].color.r, 0.1);
  ASSERT_FLOAT_EQ(markers.markers[0].color.g, 0.2);
  ASSERT_FLOAT_EQ(markers.markers[0].color.b, 0.3);
  ASSERT_FLOAT_EQ(markers.markers[0].color.a, 0.4);

  // Check the 2nd marker (j1 CUBE_LIST: anchors at origin)
  ASSERT_EQ(markers.markers[1].type, markers.markers[1].CUBE_LIST);
  ASSERT_EQ(markers.markers[1].points.size(), 4);
  for (unsigned int i = 0; i < 4; i++) {
    ASSERT_FLOAT_EQ(markers.markers[1].points[i].x, 0.0) << "index: " << i;
    ASSERT_FLOAT_EQ(markers.markers[1].points[i].y, 0.0) << "index: " << i;
  }

  // Check the 3rd marker (j2 LINE_LIST: bodyA->anchorA, bodyB->anchorB, anchorA->anchorB)
  // All bodies at (0,0); anchorA=(1,2), anchorB=(3,4)
  ASSERT_EQ(markers.markers[2].type, markers.markers[2].LINE_LIST);
  ASSERT_EQ(markers.markers[2].points.size(), 6);
  ASSERT_FLOAT_EQ(markers.markers[2].points[0].x, 0.0);   // bodyA pos
  ASSERT_FLOAT_EQ(markers.markers[2].points[0].y, 0.0);
  ASSERT_FLOAT_EQ(markers.markers[2].points[1].x, 1.0);   // anchorA world
  ASSERT_FLOAT_EQ(markers.markers[2].points[1].y, 2.0);
  ASSERT_FLOAT_EQ(markers.markers[2].points[2].x, 0.0);   // bodyB pos
  ASSERT_FLOAT_EQ(markers.markers[2].points[2].y, 0.0);
  ASSERT_FLOAT_EQ(markers.markers[2].points[3].x, 3.0);   // anchorB world
  ASSERT_FLOAT_EQ(markers.markers[2].points[3].y, 4.0);
  ASSERT_FLOAT_EQ(markers.markers[2].points[4].x, 1.0);   // anchorA
  ASSERT_FLOAT_EQ(markers.markers[2].points[4].y, 2.0);
  ASSERT_FLOAT_EQ(markers.markers[2].points[5].x, 3.0);   // anchorB
  ASSERT_FLOAT_EQ(markers.markers[2].points[5].y, 4.0);

  // Check the 4th marker (j2 CUBE_LIST)
  ASSERT_EQ(markers.markers[3].type, markers.markers[3].CUBE_LIST);
  ASSERT_EQ(markers.markers[3].points.size(), 4);
  ASSERT_FLOAT_EQ(markers.markers[3].points[0].x, 1.0);  // anchorA
  ASSERT_FLOAT_EQ(markers.markers[3].points[0].y, 2.0);
  ASSERT_FLOAT_EQ(markers.markers[3].points[1].x, 3.0);  // anchorB
  ASSERT_FLOAT_EQ(markers.markers[3].points[1].y, 4.0);
  ASSERT_FLOAT_EQ(markers.markers[3].points[2].x, 0.0);  // bodyA pos
  ASSERT_FLOAT_EQ(markers.markers[3].points[2].y, 0.0);
  ASSERT_FLOAT_EQ(markers.markers[3].points[3].x, 0.0);  // bodyB pos
  ASSERT_FLOAT_EQ(markers.markers[3].points[3].y, 0.0);

  b2DestroyWorld(world);
}

// A helper class to accept MarkerArray message callbacks
struct MarkerArraySubscriptionHelper {
  visualization_msgs::MarkerArray markers_;
  int count_;

  MarkerArraySubscriptionHelper() : count_(0) {}

  void callback(const visualization_msgs::MarkerArrayConstPtr& msg) {
    ++count_;
    ROS_INFO("GOT ONE");
    markers_ = visualization_msgs::MarkerArray(*msg);
  }

  bool waitForMessageCount(int count) {
    ros::Rate rate(10);
    for (unsigned int i = 0; i < 20; i++) {
      ros::spinOnce();
      if (count_ >= count) return true;
      rate.sleep();
    }
    return false;
  }
};

// Test publish/reset of visualization markers
TEST(DebugVizTest, testPublishMarkers) {
  flatland_server::Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);

  b2WorldId world = MakeTestWorld();

  b2BodyDef bodyDef = b2DefaultBodyDef();
  b2BodyId body = b2CreateBody(world, &bodyDef);
  b2BodyId body2 = b2CreateBody(world, &bodyDef);

  b2Circle circle;
  circle.center = {2.0f, 3.0f};
  circle.radius = 0.2f;
  b2ShapeDef shapeDef = b2DefaultShapeDef();
  b2CreateCircleShape(body, &shapeDef, &circle);

  b2WeldJointDef jd = b2DefaultWeldJointDef();
  jd.bodyIdA = body;
  jd.bodyIdB = body2;
  jd.localAnchorA = {0.0f, 0.0f};
  jd.localAnchorB = {0.0f, 0.0f};
  b2JointId joint = b2CreateWeldJoint(world, &jd);

  ros::NodeHandle nh;
  MarkerArraySubscriptionHelper helper;
  ros::Subscriber sub =
      nh.subscribe("/debug_visualization_test/debug/example", 0,
                   &MarkerArraySubscriptionHelper::callback, &helper);

  flatland_server::DebugVisualization::Get().Visualize("example", body, 1.0,
                                                       0.0, 0.0, 1.0);

  EXPECT_EQ(flatland_server::DebugVisualization::Get().topics_.size(), 1);
  ros::spinOnce();
  EXPECT_EQ(helper.count_, 0);
  EXPECT_EQ(flatland_server::DebugVisualization::Get()
                .topics_["example"]
                .needs_publishing,
            true);
  EXPECT_EQ(sub.getNumPublishers(), 1);

  flatland_server::DebugVisualization::Get().Publish(timekeeper);
  EXPECT_TRUE(helper.waitForMessageCount(1));
  EXPECT_EQ(helper.markers_.markers.size(), 1);

  flatland_server::DebugVisualization::Get().Publish(timekeeper);
  EXPECT_TRUE(helper.waitForMessageCount(1));
  EXPECT_EQ(1, helper.markers_.markers.size());

  flatland_server::DebugVisualization::Get().Visualize("example", body, 1.0,
                                                       0.0, 0.0, 1.0);
  flatland_server::DebugVisualization::Get().Visualize("example", body, 1.0,
                                                       0.0, 0.0, 1.0);
  // inserts two markers
  flatland_server::DebugVisualization::Get().Visualize("example", joint, 1.0,
                                                       0.0, 0.0, 1.0);
  flatland_server::DebugVisualization::Get().Publish(timekeeper);
  EXPECT_TRUE(helper.waitForMessageCount(2));
  EXPECT_EQ(5, helper.markers_.markers.size());

  flatland_server::DebugVisualization::Get().Reset("example");
  flatland_server::DebugVisualization::Get().Publish(timekeeper);
  EXPECT_TRUE(helper.waitForMessageCount(2));

  flatland_server::DebugVisualization::Get().Visualize("example", joint, 1.0,
                                                       0.0, 0.0, 1.0);
  flatland_server::DebugVisualization::Get().Publish(timekeeper);
  EXPECT_TRUE(helper.waitForMessageCount(3));
  EXPECT_EQ(2, helper.markers_.markers.size());

  b2DestroyWorld(world);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "debug_visualization_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
