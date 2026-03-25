/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  plugin_manager_test.cpp
 * @brief Testing plugin manager functionalities
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

#include <flatland_server/debug_visualization.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;

class TestModelPlugin : public ModelPlugin {
 public:
  // variables used for testing
  double timestep_before;
  double timestep_after;
  Entity *entity;
  b2ShapeId shape_A;
  b2ShapeId shape_B;

  std::map<std::string, bool> function_called;

  TestModelPlugin() { ClearTestingVariables(); }

  void ClearTestingVariables() {
    entity = nullptr;
    shape_A = b2_nullShapeId;
    shape_B = b2_nullShapeId;

    function_called["OnInitialize"] = false;
    function_called["BeforePhysicsStep"] = false;
    function_called["AfterPhysicsStep"] = false;
    function_called["BeginContact"] = false;
    function_called["EndContact"] = false;
  }

  void OnInitialize(const YAML::Node &config) override {
    function_called["OnInitialize"] = true;
  }

  void BeforePhysicsStep(const Timekeeper &timekeeper) override {
    function_called["BeforePhysicsStep"] = true;
  }

  void AfterPhysicsStep(const Timekeeper &timekeeper) override {
    function_called["AfterPhysicsStep"] = true;
  }

  void BeginContact(b2ShapeId shapeA, b2ShapeId shapeB) override {
    function_called["BeginContact"] = true;
    b2BodyId bodyA, bodyB;
    FilterContact(shapeA, shapeB, entity, bodyA, bodyB);
    shape_A = shapeA;
    shape_B = shapeB;
  }

  void EndContact(b2ShapeId shapeA, b2ShapeId shapeB) override {
    function_called["EndContact"] = true;
    b2BodyId bodyA, bodyB;
    FilterContact(shapeA, shapeB, entity, bodyA, bodyB);
    shape_A = shapeA;
    shape_B = shapeB;
  }
};

class PluginManagerTest : public ::testing::Test {
 protected:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  Timekeeper timekeeper;
  World *w;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    w = nullptr;
  }

  void TearDown() override {
    if (w != nullptr) {
      delete w;
    }
  }

  PluginManagerTest() {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }

  bool fltcmp(double n1, double n2) {
    bool ret = fabs(n1 - n2) < 1e-7;
    return ret;
  }

  // checks if tow maps have the same keys
  bool key_compare(std::map<std::string, bool> const &lhs,
                   std::map<std::string, bool> const &rhs) {
    auto pred = [](decltype(*lhs.begin()) a, decltype(a) b) {
      return a.first == b.first;
    };

    return lhs.size() == rhs.size() &&
           std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
  }

  // Check the true/false if the function is called
  bool FunctionCallEq(TestModelPlugin *p,
                      std::map<std::string, bool> function_called) {
    if (!key_compare(p->function_called, function_called)) {
      printf("Two maps does not have the same keys (set of function)\n");
      return false;
    }

    for (const auto &func : p->function_called) {
      if (func.second != function_called[func.first]) {
        printf("%s is %s, expected to be %s\n", func.first.c_str(),
               func.second ? "called" : "not called",
               function_called[func.first] ? "called" : "not called");
        return false;
      }
    }
    return true;
  }
};

/**
 * This test moves bodies around and test if all the correct functions are
 * called with the expected inputs
 */
TEST_F(PluginManagerTest, collision_test) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/collision_test/world.yaml");
  timekeeper.SetMaxStepSize(1.0);
  w = World::MakeWorld(world_yaml.string());
  Layer *l = w->layers_[0];
  Model *m0 = w->models_[0];
  Model *m1 = w->models_[1];
  Body *b0 = m0->bodies_[0];
  Body *b1 = m1->bodies_[0];
  PluginManager *pm = &w->plugin_manager_;
  boost::shared_ptr<TestModelPlugin> shared_p(new TestModelPlugin());
  shared_p->Initialize("TestModelPlugin", "test_model_plugin", m0,
                       YAML::Node());
  pm->model_plugins_.push_back(shared_p);
  TestModelPlugin *p = shared_p.get();

  // step the world with everything at zero velocity, this should make sure
  // the collision callbacks gets called
  w->Update(timekeeper);
  w->Update(timekeeper);

  // model 0 is placed right on top of a layer edge at the beginning. Thus,
  // begin contact should trigger, as well as before and after physics step.
  EXPECT_TRUE(FunctionCallEq(p, {{"OnInitialize", true},
                                 {"BeforePhysicsStep", true},
                                 {"AfterPhysicsStep", true},
                                 {"BeginContact", true},
                                 {"EndContact", false}}));
  EXPECT_EQ(p->entity, l);
  // shape_A should belong to b0's body
  EXPECT_TRUE(B2_ID_EQUALS(b2Shape_GetBody(p->shape_A), b0->physics_body_));
  // shape_B should be a segment shape (layer edge)
  EXPECT_EQ(b2Shape_GetType(p->shape_B), b2_segmentShape);
  p->ClearTestingVariables();

  // move the body 2m to the left over two 1s timesteps, this should remove any
  // contacts between the body and the layer
  b2Body_SetLinearVelocity(b0->physics_body_, b2Vec2{-1, 0});
  // takes two steps for Box2D to generate collision events
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(p, {{"OnInitialize", false},
                                 {"BeforePhysicsStep", true},
                                 {"AfterPhysicsStep", true},
                                 {"BeginContact", false},
                                 {"EndContact", true}}));
  EXPECT_EQ(p->entity, l);
  EXPECT_TRUE(B2_ID_EQUALS(b2Shape_GetBody(p->shape_A), b0->physics_body_));
  EXPECT_EQ(b2Shape_GetType(p->shape_B), b2_segmentShape);
  p->ClearTestingVariables();

  // move the body 1m down over 2 timesteps, this should place model 0 in
  // contact with model 1
  b2Body_SetLinearVelocity(b0->physics_body_, b2Vec2{0, 0});
  b2Body_SetLinearVelocity(b1->physics_body_, b2Vec2{0, -0.5f});
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(p, {{"OnInitialize", false},
                                 {"BeforePhysicsStep", true},
                                 {"AfterPhysicsStep", true},
                                 {"BeginContact", true},
                                 {"EndContact", false}}));
  EXPECT_EQ(p->entity, m1);
  EXPECT_TRUE(B2_ID_EQUALS(b2Shape_GetBody(p->shape_B), b1->physics_body_));
  EXPECT_TRUE(B2_ID_EQUALS(b2Shape_GetBody(p->shape_A), b0->physics_body_));
  p->ClearTestingVariables();

  // move the body 2m down over 2 timesteps, this should clear any contacts for
  // model 0
  b2Body_SetLinearVelocity(b0->physics_body_, b2Vec2{0, 0});
  b2Body_SetLinearVelocity(b1->physics_body_, b2Vec2{0, -1.0f});
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(p, {{"OnInitialize", false},
                                 {"BeforePhysicsStep", true},
                                 {"AfterPhysicsStep", true},
                                 {"BeginContact", false},
                                 {"EndContact", true}}));
  EXPECT_EQ(p->entity, m1);
  EXPECT_TRUE(B2_ID_EQUALS(b2Shape_GetBody(p->shape_B), b1->physics_body_));
  EXPECT_TRUE(B2_ID_EQUALS(b2Shape_GetBody(p->shape_A), b0->physics_body_));
  p->ClearTestingVariables();

  // now teleport b0 back to (0, 0) on top of the layer edge
  b2Body_SetLinearVelocity(b0->physics_body_, b2Vec2{0, 0});
  b2Body_SetTransform(b0->physics_body_, b2Vec2{0, 0}, b2MakeRot(0));
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(p, {{"OnInitialize", false},
                                 {"BeforePhysicsStep", true},
                                 {"AfterPhysicsStep", true},
                                 {"BeginContact", true},
                                 {"EndContact", false}}));
  EXPECT_EQ(p->entity, l);
  EXPECT_TRUE(B2_ID_EQUALS(b2Shape_GetBody(p->shape_A), b0->physics_body_));
  EXPECT_EQ(b2Shape_GetType(p->shape_B), b2_segmentShape);
  p->ClearTestingVariables();

  // w->DebugVisualize();
  // DebugVisualization::Get().Publish();
  // ros::spin();
}

TEST_F(PluginManagerTest, load_dummy_test) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/load_dummy_test/world.yaml");

  w = World::MakeWorld(world_yaml.string());

  ModelPlugin *p = w->plugin_manager_.model_plugins_[0].get();

  EXPECT_STREQ(p->GetType().c_str(), "DummyModelPlugin");
  EXPECT_STREQ(p->GetName().c_str(), "dummy_test_plugin");
}

TEST_F(PluginManagerTest, plugin_throws_exception) {
  world_yaml =
      this_file_dir /
      fs::path("plugin_manager_tests/plugin_throws_exception/world.yaml");

  try {
    w = World::MakeWorld(world_yaml.string());
    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException &e) {
    // do a regex match against error message
    std::string regex_str =
        ".*dummy_param_float must be dummy_test_123456, instead it was "
        "\"wrong_message\".*";
    std::cmatch match;
    std::regex regex(regex_str);
    EXPECT_TRUE(std::regex_match(e.what(), match, regex))
        << "Exception Message '" + std::string(e.what()) + "'" +
               " did not match against regex '" + regex_str + "'";
  } catch (const std::exception &e) {
    ADD_FAILURE() << "Was expecting a PluginException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

TEST_F(PluginManagerTest, nonexistent_plugin) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/nonexistent_plugin/world.yaml");

  try {
    w = World::MakeWorld(world_yaml.string());
    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException &e) {
    std::cmatch match;
    std::string regex_str =
        ".*RandomPlugin with base class type flatland_server::ModelPlugin does "
        "not exist.*";
    std::regex regex(regex_str);
    EXPECT_TRUE(std::regex_match(e.what(), match, regex))
        << "Exception Message '" + std::string(e.what()) + "'" +
               " did not match against regex '" + regex_str + "'";
  } catch (const std::exception &e) {
    ADD_FAILURE() << "Was expecting a PluginException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

TEST_F(PluginManagerTest, invalid_plugin_yaml) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/invalid_plugin_yaml/world.yaml");

  try {
    w = World::MakeWorld(world_yaml.string());
    FAIL() << "Expected an exception, but none were raised";
  } catch (const YAMLException &e) {
    EXPECT_STREQ(
        "Flatland YAML: Entry \"name\" does not exist (in model \"turtlebot1\" "
        "\"plugins\" index=0)",
        e.what());
  } catch (const std::exception &e) {
    ADD_FAILURE() << "Was expecting a YAMLException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

TEST_F(PluginManagerTest, duplicate_plugin) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/duplicate_plugin/world.yaml");

  try {
    w = World::MakeWorld(world_yaml.string());
    FAIL() << "Expected an exception, but none were raised";
  } catch (const YAMLException &e) {
    EXPECT_STREQ(
        "Flatland YAML: Invalid \"plugins\" in \"turtlebot1\" model, plugin "
        "with name \"dummy_test_plugin\" already exists",
        e.what());
  } catch (const std::exception &e) {
    ADD_FAILURE() << "Was expecting a YAMLException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  ros::init(argc, argv, "plugin_manager_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
