/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 world.cpp
 * @brief	 Loads world file
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

#include <box2d/box2d.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/types.h>
#include <flatland_server/world.h>
#include <flatland_server/yaml_reader.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <map>
#include <string>

namespace flatland_server {

// --------------- enkiTS <-> Box2D v3 task adapter ---------------

class FlatlandTask : public enki::ITaskSet {
public:
  FlatlandTask() = default;

  void ExecuteRange(enki::TaskSetPartition range,
                    uint32_t threadIndex) override {
    m_task(range.start, range.end, threadIndex, m_taskContext);
  }

  b2TaskCallback *m_task = nullptr;
  void *m_taskContext = nullptr;
};

static constexpr int kMaxTasks = 128;
static FlatlandTask s_tasks[kMaxTasks];
static int s_taskCount = 0;

static void *EnkiEnqueueTask(b2TaskCallback *fcn, int32_t itemCount,
                              int32_t minRange, void *taskContext,
                              void *userContext) {
  auto *scheduler = static_cast<enki::TaskScheduler *>(userContext);
  if (s_taskCount < kMaxTasks) {
    FlatlandTask &task = s_tasks[s_taskCount];
    task.m_SetSize = itemCount;
    task.m_MinRange = minRange;
    task.m_task = fcn;
    task.m_taskContext = taskContext;
    scheduler->AddTaskSetToPipe(&task);
    ++s_taskCount;
    return &task;
  }
  // Fallback: run inline if pool exhausted
  fcn(0, itemCount, 0, taskContext);
  return nullptr;
}

static void EnkiFinishTask(void *userTask, void *userContext) {
  if (userTask != nullptr) {
    auto *scheduler = static_cast<enki::TaskScheduler *>(userContext);
    auto *task = static_cast<FlatlandTask *>(userTask);
    scheduler->WaitforTask(task);
  }
}

// ----------------------------------------------------------------

World::World()
    : gravity_({0.0f, 0.0f}),
      service_paused_(false),
      skip_physics_step_(false),
      int_marker_manager_(&models_, &plugin_manager_) {
  task_scheduler_.Initialize();

  b2WorldDef world_def = b2DefaultWorldDef();
  world_def.gravity = gravity_;
  world_def.workerCount = task_scheduler_.GetNumTaskThreads();
  world_def.enqueueTask = EnkiEnqueueTask;
  world_def.finishTask = EnkiFinishTask;
  world_def.userTaskContext = &task_scheduler_;
  world_id_ = b2CreateWorld(&world_def);
}

World::~World() {
  ROS_INFO_NAMED("World", "Destroying world...");

  // The order of things matters in the destructor. The contact listener is
  // removed first to avoid the triggering the contact functions in plugin
  // manager which might cause it to work with deleted layers/models.

  // the physics body of layers are set to b2_nullBodyId because there are tons
  // of shapes in a layer and it is too slow for the DestroyBody method to
  // remove them since the AABB tree gets restructured every time a shape is
  // removed. The memory will later be freed by destroying the world.
  for (auto &layer : layers_) {
    if (layer->body_ != nullptr) {
      layer->body_->physics_body_ = b2_nullBodyId;
    }
    delete layer;
  }

  // The bodies of models are not set to null like layers because there aren't
  // nearly as many fixtures, and we might hide some memory problems by using
  // the shortcut
  for (unsigned int i = 0; i < models_.size(); i++) {
    delete models_[i];
  }

  // This frees the entire Box2D world with everything in it
  b2DestroyWorld(world_id_);
  world_id_ = b2_nullWorldId;
  task_scheduler_.WaitforAllAndShutdown();

  ROS_INFO_NAMED("World", "World destroyed");
}

void World::Update(Timekeeper &timekeeper) {
  if (!IsPaused()) {
    // Apply dynamic step size if feature is enabled
    if (use_dynamic_fast_sim_ && dynamic_step_size_ > 0.0) {
      timekeeper.SetMaxStepSize(dynamic_step_size_);
    }

    plugin_manager_.BeforePhysicsStep(timekeeper);

    if (!skip_physics_step_) {
      s_taskCount = 0;  // Reset task pool for this step
      b2World_Step(world_id_, timekeeper.GetStepSize(),
                   physics_velocity_iterations_);

      // Poll contact events (replaces b2ContactListener callbacks from v2)
      b2ContactEvents events = b2World_GetContactEvents(world_id_);
      for (int i = 0; i < events.beginCount; i++) {
        const b2ContactBeginTouchEvent &e = events.beginEvents[i];
        plugin_manager_.BeginContact(e.shapeIdA, e.shapeIdB);
      }
      for (int i = 0; i < events.endCount; i++) {
        const b2ContactEndTouchEvent &e = events.endEvents[i];
        plugin_manager_.EndContact(e.shapeIdA, e.shapeIdB);
      }
      for (int i = 0; i < events.hitCount; i++) {
        const b2ContactHitEvent &e = events.hitEvents[i];
        plugin_manager_.OnContactHit(e.shapeIdA, e.shapeIdB, e.point, e.normal,
                                     e.approachSpeed);
      }
    }

    timekeeper.StepTime();
    plugin_manager_.AfterPhysicsStep(timekeeper);
  }
  int_marker_manager_.update();
}

World *World::MakeWorld(const std::string &yaml_path) {
  YamlReader world_reader = YamlReader(yaml_path);
  YamlReader prop_reader = world_reader.Subnode("properties", YamlReader::MAP);
  int v = prop_reader.Get<int>("velocity_iterations", 10);
  int p = prop_reader.Get<int>("position_iterations", 10);
  bool skip_physics = prop_reader.Get<bool>("skip_physics_step", false);
  prop_reader.EnsureAccessedAllKeys();

  World *w = new World();

  w->world_yaml_dir_ = boost::filesystem::path(yaml_path).parent_path();
  w->yaml_path_ = yaml_path;
  w->physics_velocity_iterations_ = v;
  w->skip_physics_step_ = skip_physics;

  try {
    YamlReader layers_reader = world_reader.Subnode("layers", YamlReader::LIST);
    YamlReader models_reader =
        world_reader.SubnodeOpt("models", YamlReader::LIST);
    YamlReader world_plugin_reader =
        world_reader.SubnodeOpt("plugins", YamlReader::LIST);
    world_reader.EnsureAccessedAllKeys();
    w->LoadLayers(layers_reader);
    w->LoadModels(models_reader);
    w->LoadWorldPlugins(world_plugin_reader, w, world_reader);
  } catch (const YAMLException &e) {
    ROS_FATAL_NAMED("World", "Error loading from YAML");
    delete w;
    throw e;
  } catch (const PluginException &e) {
    ROS_FATAL_NAMED("World", "Error loading plugins");
    delete w;
    throw e;
  } catch (const Exception &e) {
    ROS_FATAL_NAMED("World", "Error loading world");
    delete w;
    throw e;
  }
  return w;
}

World *World::MakeWorld(const std::string &yaml_path,
                        const std::string &models_path,
                        const std::string &world_plugins_path) {
  YamlReader world_settings_reader = YamlReader(world_plugins_path);
  YamlReader prop_reader =
      world_settings_reader.Subnode("properties", YamlReader::MAP);
  YamlReader world_plugin_reader =
      world_settings_reader.SubnodeOpt("plugins", YamlReader::LIST);

  int v = prop_reader.Get<int>("velocity_iterations", 10);
  int p = prop_reader.Get<int>("position_iterations", 10);
  bool skip_physics = prop_reader.Get<bool>("skip_physics_step", false);
  prop_reader.EnsureAccessedAllKeys();

  World *w = new World();

  w->world_yaml_dir_ = boost::filesystem::path(yaml_path).parent_path();
  w->physics_velocity_iterations_ = v;
  w->physics_position_iterations_ = p;
  w->skip_physics_step_ = skip_physics;
  w->models_path_ = models_path;
  w->yaml_path_ = yaml_path;

  try {
    w->LoadWorldPlugins(world_plugin_reader, w, world_settings_reader);
  } catch (const YAMLException &e) {
    ROS_FATAL_NAMED("World", "Error loading world plugins");
    delete w;
    throw e;
  } catch (const PluginException &e) {
    ROS_FATAL_NAMED("World", "Error loading plugins");
    delete w;
    throw e;
  }
  return w;
}

void World::LoadWorldEntities() {
  try {
    YamlReader map_info_reader = YamlReader(yaml_path_);
    YamlReader layers_reader =
        map_info_reader.Subnode("layers", YamlReader::LIST);
    YamlReader models_reader =
        map_info_reader.SubnodeOpt("models", YamlReader::LIST);
    LoadLayers(layers_reader);
    LoadModels(models_reader);
  } catch (const YAMLException &e) {
    ROS_WARN_STREAM_DELAYED_THROTTLE_NAMED(1, "World",
                                          yaml_path_ << " not loaded yet");
    throw e;
  }
}

void World::recomputeDynamicStepSize() {
  auto n = agents_in_slow_time_.size();
  if (n == 0) {
    // All agents are processing sim tasks: run at max speed
    dynamic_step_size_ = max_lower_speed_dynamic_sim_;
  } else if ((int)n <= num_robots_threshold_dynamic_sim_) {
    // Some agents are slow: run at mid speed
    dynamic_step_size_ = (min_lower_speed_dynamic_sim_ + max_lower_speed_dynamic_sim_) * 0.5;
  } else {
    // Many agents are slow: run at min speed
    dynamic_step_size_ = min_lower_speed_dynamic_sim_;
  }
}

void World::SlowSimTime(const std::string &agent) {
  if (!use_dynamic_fast_sim_) return;
  agents_in_slow_time_.insert(agent);
  recomputeDynamicStepSize();
}

void World::FastSimTime(const std::string &agent) {
  if (!use_dynamic_fast_sim_) return;
  agents_in_slow_time_.erase(agent);
  recomputeDynamicStepSize();
}

void World::InitializeDynamicFastSim(double max_lower_speed,
                                      double min_lower_speed,
                                      int num_robots_threshold) {
  use_dynamic_fast_sim_ = true;
  max_lower_speed_dynamic_sim_ = max_lower_speed;
  min_lower_speed_dynamic_sim_ = min_lower_speed;
  num_robots_threshold_dynamic_sim_ = num_robots_threshold;
  dynamic_step_size_ = max_lower_speed;  // start at max (assume all agents are fast initially)
  ROS_INFO_NAMED("World",
                 "Dynamic fast-sim initialized: max_step=%.4f min_step=%.4f threshold=%d",
                 max_lower_speed, min_lower_speed, num_robots_threshold);
}

void World::LoadLayers(YamlReader &layers_reader) {
  // loop through each layer and parse the data
  for (int i = 0; i < layers_reader.NodeSize(); i++) {
    YamlReader reader = layers_reader.Subnode(i, YamlReader::MAP);
    YamlReader name_reader = reader.Subnode("name", YamlReader::NO_CHECK);

    // allow names to be either a just a string or a list of strings
    std::vector<std::string> names;
    if (name_reader.Node().IsSequence()) {
      names = name_reader.AsList<std::string>(1, -1);
    } else {
      names.push_back(name_reader.As<std::string>());
    }

    if (cfr_.LayersCount() + names.size() > cfr_.MAX_LAYERS) {
      throw YAMLException(
          "Unable to add " + std::to_string(names.size()) +
          " additional layer(s) {" + boost::algorithm::join(names, ", ") +
          "}, current layers count is " + std::to_string(cfr_.LayersCount()) +
          ", max allowed is " + std::to_string(cfr_.MAX_LAYERS));
    }

    boost::filesystem::path map_path(reader.Get<std::string>("map", ""));
    Color color = reader.GetColor("color", Color(1, 1, 1, 1));
    auto properties =
        reader.SubnodeOpt("properties", YamlReader::NodeTypeCheck::MAP).Node();
    reader.EnsureAccessedAllKeys();

    for (const auto &name : names) {
      if (cfr_.RegisterLayer(name) == cfr_.LAYER_ALREADY_EXIST) {
        throw YAMLException("Layer with name " + Q(name) + " already exists");
      }
    }

    if (map_path.string().front() != '/' && map_path.string().length() > 0) {
      map_path = world_yaml_dir_ / map_path;
    }

    ROS_INFO_NAMED("World", "Loading layer \"%s\" from path=\"%s\"",
                   names[0].c_str(), map_path.string().c_str());

    Layer *layer = Layer::MakeLayer(world_id_, &cfr_, map_path.string(),
                                    names, color, properties);
    layers_name_map_.insert(
        std::pair<std::vector<std::string>, Layer *>(names, layer));
    layers_.push_back(layer);

    ROS_INFO_NAMED("World", "Layer \"%s\" loaded", layer->name_.c_str());
    layer->DebugOutput();
  }
}

void World::LoadModels(YamlReader &models_reader) {
  if (!models_reader.IsNodeNull()) {
    for (int i = 0; i < models_reader.NodeSize(); i++) {
      YamlReader reader = models_reader.Subnode(i, YamlReader::MAP);

      std::string name = reader.Get<std::string>("name");
      std::string ns = reader.Get<std::string>("namespace", "");
      Pose pose = reader.GetPose("pose", Pose(0, 0, 0));
      std::string path = reader.Get<std::string>("model");
      reader.EnsureAccessedAllKeys();
      LoadModel(path, ns, name, pose);
    }
  }
}

void World::LoadWorldPlugins(YamlReader &world_plugin_reader, World *world,
                             YamlReader &world_config) {
  if (!world_plugin_reader.IsNodeNull()) {
    for (int i = 0; i < world_plugin_reader.NodeSize(); i++) {
      YamlReader reader = world_plugin_reader.Subnode(i, YamlReader::MAP);
      ROS_INFO_NAMED("World", "loading world_plugin");
      plugin_manager_.LoadWorldPlugin(world, reader, world_config);
    }
  }
}
void World::LoadModel(const std::string &model_yaml_path, const std::string &ns,
                      const std::string &name, const Pose &pose) {
  // ensure no duplicate model names
  if (std::count_if(models_.begin(), models_.end(),
                    [&](Model *m) { return m->name_ == name; }) >= 1) {
    throw YAMLException("Model with name " + Q(name) + " already exists");
  }

  boost::filesystem::path abs_path(model_yaml_path);
  if (model_yaml_path.front() != '/') {
    abs_path = world_yaml_dir_ / abs_path;
  }

  ROS_INFO_NAMED("World", "Loading model from path=\"%s\"",
                 abs_path.string().c_str());

  Model *m =
      Model::MakeModel(this, world_id_, &cfr_, abs_path.string(), ns, name);
  m->TransformAll(pose);

  try {
    for (int i = 0; i < m->plugins_reader_.NodeSize(); i++) {
      YamlReader plugin_reader = m->plugins_reader_.Subnode(i, YamlReader::MAP);
      plugin_manager_.LoadModelPlugin(m, plugin_reader);
    }
  } catch (const YAMLException &e) {
    plugin_manager_.DeleteModelPlugin(m);
    delete m;
    throw e;
  } catch (const PluginException &e) {
    plugin_manager_.DeleteModelPlugin(m);
    delete m;
    throw e;
  }

  models_.push_back(m);

  visualization_msgs::MarkerArray body_markers;
  for (size_t i = 0; i < m->bodies_.size(); i++) {
    DebugVisualization::Get().BodyToMarkers(
        body_markers, m->bodies_[i]->physics_body_, 1.0, 0.0, 0.0, 1.0);
  }
  int_marker_manager_.createInteractiveMarker(name, pose, body_markers);

  ROS_INFO_NAMED("World", "Model \"%s\" loaded", m->name_.c_str());
  m->DebugOutput();
}

void World::DeleteModel(const std::string &name) {
  bool found = false;

  for (unsigned int i = 0; i < models_.size(); i++) {
    // name is unique, so there will only be one object with this name
    if (models_[i]->GetName() == name) {
      // delete the plugins associated with the model
      plugin_manager_.DeleteModelPlugin(models_[i]);
      delete models_[i];
      models_.erase(models_.begin() + i);
      int_marker_manager_.deleteInteractiveMarker(name);
      found = true;
      break;
    }
  }

  if (!found) {
    throw Exception("Flatland World: failed to delete model, model with name " +
                    Q(name) + " does not exist");
  }
}

void World::MoveModel(const std::string &name, const Pose &pose) {
  // Find desired model
  bool found = false;

  for (unsigned int i = 0; i < models_.size(); i++) {
    if (models_[i]->GetName() == name) {
      // move the model
      models_[i]->SetPose(pose);
      found = true;
      break;
    }
  }

  if (!found) {
    throw Exception("Flatland World: failed to move model, model with name " +
                    Q(name) + " does not exist");
  }
}

void World::Pause() { service_paused_ = true; }

void World::Resume() { service_paused_ = false; }

void World::TogglePaused() { service_paused_ = !service_paused_; }

bool World::IsPaused() {
  return service_paused_ || int_marker_manager_.isManipulating();
}

void World::DebugVisualize(bool update_layers) {
  if (update_layers) {
    for (const auto &layer : layers_) {
      layer->DebugVisualize();
    }
  }

  for (const auto &model : models_) {
    model->DebugVisualize();
  }
}
};  // namespace flatland_server
