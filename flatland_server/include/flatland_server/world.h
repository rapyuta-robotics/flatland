/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 world.h
 * @brief	 Definition for the simulation world
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

#ifndef FLATLAND_SERVER_WORLD_H
#define FLATLAND_SERVER_WORLD_H

#include <set>
#include <string>

#include <box2d/box2d.h>
#include "TaskScheduler.h"
#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/message_server.h>
#include <flatland_server/interactive_marker_manager.h>
#include <flatland_server/layer.h>
#include <flatland_server/model.h>
#include <flatland_server/plugin_manager.h>
#include <flatland_server/timekeeper.h>
#include <map>
#include <string>
#include <string>
#include <vector>

namespace flatland_server {

/**
 * This class defines a world in the simulation. A world contains layers
 * that can represent environments at multiple levels, and models which are
 * can be robots or obstacles.
 */
class World {
 public:
  boost::filesystem::path world_yaml_dir_;  ///<directory containing world file
  std::string yaml_path_;                   ///< path to the world YAML file
  std::string models_path_;                 ///< path to models directory
  b2WorldId world_id_;                      ///< Box2D v3 physics world ID
  b2Vec2 gravity_;  ///< Box2D world gravity, always (0, 0)
  std::map<std::vector<std::string>, Layer *>
      layers_name_map_;           ///< map of all layers and thier name
  std::vector<Layer *> layers_;   ///< list of layers
  std::vector<Model *> models_;   ///< list of models
  CollisionFilterRegistry cfr_;   ///< collision registry for layers and models
  PluginManager plugin_manager_;  ///< for loading and updating plugins
  bool service_paused_;  ///< indicates if simulation is paused by a service
                         /// call or not
  InteractiveMarkerManager
      int_marker_manager_;  ///< for dynamically moving models from Rviz
  int physics_velocity_iterations_;  ///< Box2D solver velocity iterations
  int physics_position_iterations_;  ///< Box2D solver position iterations
  bool skip_physics_step_;  ///< when true, skip b2World_Step and contact events
  enki::TaskScheduler task_scheduler_;  ///< enkiTS multi-core task scheduler
  MessageServer message_server;          ///< internal message passing system

  // Dynamic fast-sim-time state
  bool use_dynamic_fast_sim_{false};
  double max_lower_speed_dynamic_sim_{0.0};
  double min_lower_speed_dynamic_sim_{0.0};
  int num_robots_threshold_dynamic_sim_{0};
  std::set<std::string> agents_in_slow_time_;
  double dynamic_step_size_{0.0};  ///< computed step size (0 = use SimulationManager default)

  void recomputeDynamicStepSize();  ///< recompute dynamic_step_size_ from agents_in_slow_time_

  /**
   * @brief Constructor for the world class. All data required for
   * initialization should be passed in here
   */
  World();

  /**
   * @brief Destructor for the world class
   */
  ~World();

  /**
   * @brief trigger world update include all physics and plugins
   * @param[in] timekeeper The time keeping object
   */
  void Update(Timekeeper &timekeeper);

  /*
   * @brief Load world plugins
   * @param[in] world_plugin_reader, readin the info about the plugin
   * @param[in] world, the world where the plugin will be applied to
   * @param[in] world config, the yaml reader of world.yaml
  */
  void LoadWorldPlugins(YamlReader &world_plugin_reader, World *world,
                        YamlReader &world_config);
  /**
   * @brief load layers into the world. Throws YAMLException.
   * @param[in] layers_reader Yaml reader for node that has list of layers
   */
  void LoadLayers(YamlReader &layers_reader);

  /**
   * @brief load models into the world. Throws YAMLException.
   * @param[in] layers_reader Yaml reader for node that has a list of models
   */
  void LoadModels(YamlReader &models_reader);

  /**
   * @brief load models into the world. Throws YAMLException.
   * @param[in] model_yaml_path Relative path to the model yaml file
   * @param[in] ns Namespace of the robot
   * @param[in] name Name of the model
   * @param[in] pose Initial pose of the model in x, y, yaw
   */
  void LoadModel(const std::string &model_yaml_path, const std::string &ns,
                 const std::string &name, const Pose &pose);

  /// @brief Reload layers and models from the world YAML (used by map_info plugin)
  void LoadWorldEntities();

  /// @brief Mark an agent as needing slower simulation time
  void SlowSimTime(const std::string &agent);

  /// @brief Remove an agent from the slow-simulation set
  void FastSimTime(const std::string &agent);

  /// @brief Initialize dynamic fast-sim parameters (stub — not yet ported to Box2D v3)
  void InitializeDynamicFastSim(double max_lower_speed, double min_lower_speed,
                                 int num_robots_threshold);

  /**
   * @brief remove model with a given name
   * @param[in] name The name of the model to remove
   */
  void DeleteModel(const std::string &name);

  /**
   * @brief move model with a given name
   * @param[in] name The name of the model to move
   * @param[in] pose The desired new pose of the model
   */
  void MoveModel(const std::string &name, const Pose &pose);

  /**
   * @brief set the paused state of the simulation to true
   */
  void Pause();

  /**
   * @brief set the paused state of the simulation to false
   */
  void Resume();

  /**
   * @brief toggle the paused state of the simulation
   */
  void TogglePaused();

  /**
   * @brief returns true if service_paused_ is true or an interactive marker is
   * currently being dragged
   */
  bool IsPaused();

  /**
   * @brief factory method to create a instance of the world class. Cleans all
   * the inputs before instantiation of the class. TThrows YAMLException.
   * @param[in] yaml_path Path to the world yaml file
   * @return pointer to a new world
   */
  static World *MakeWorld(const std::string &yaml_path);

  /**
   * @brief Create a world from separate world yaml, models path and plugins yaml.
   * @param[in] yaml_path Path to world yaml (layers/models)
   * @param[in] models_path Path to models directory
   * @param[in] world_plugins_path Path to world plugins yaml
   */
  static World *MakeWorld(const std::string &yaml_path,
                          const std::string &models_path,
                          const std::string &world_plugins_path);

  /**
   * @brief Publish debug visualizations for everything
   * @param[in] update_layers since layers are pretty much static, this
   * parameter is used to skip updating layers
   */
  void DebugVisualize(bool update_layers = true);
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_WORLD_H
