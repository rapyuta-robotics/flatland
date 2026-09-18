/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2020 Avidbots Corp.
 * @name	flatland_server_node.cpp
 * @brief	Load params and run the ros node for flatland_server
 * @author Joseph Duchesne
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Avidbots Corp.
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

#include <signal.h>

#include <string>

#include "flatland_server/debug_visualization.h"
#include "flatland_server/simulation_manager.h"
#include "rclcpp/rclcpp.hpp"

/**
 * Without explicit teardown, FlatlandServerNode is never destroyed before
 * the process exits, because two independent references keep it alive:
 *
 * 1. A shared_ptr reference cycle. FlatlandServerNode owns a
 *    SimulationManager (simulation_manager_ below), and SimulationManager's
 *    own node_ member (simulation_manager.h) is a shared_ptr back to the
 *    very node that constructed it (shared_from_this() at construction).
 *    Neither object's refcount can ever reach zero while the other is
 *    alive, so absent an explicit break, both are leaked for the life of
 *    the process and neither destructor ever runs.
 * 2. DebugVisualization::Get()'s singleton (debug_visualization.h)
 *    independently holds a third shared_ptr to this node, taken the first
 *    time visualization is published.
 *
 * With the cycle alone, ~FlatlandServerNode() -- and, transitively,
 * ~rclcpp::Node(), which is what triggers rmw_destroy_node() for this
 * node's publishers, subscriptions and DDS participant -- never runs at
 * all, not merely late. `rclcpp::shutdown()` does not help: it tears down
 * the global rclcpp Context, not any individual node's rmw entities, so
 * calling it does not stop this node's DDS receive threads. What actually
 * happens instead is that the process eventually reaches the C++ runtime's
 * own exit-time atexit/DSO-finalizer chain, which can unmap a shared
 * library (e.g. the DDS middleware) while that library's own threads are
 * still executing in it -- a module-unload race, not a deterministic
 * teardown.
 *
 * The fix is not about SIGINT delivery (see the fork's separate
 * fix/teardown-shutdown-race branch for that, a different mechanism, not
 * touched here): it is to make sure nothing outlives main(). Run() below
 * drops this node's own reference to SimulationManager once Main() returns;
 * main() drops DebugVisualization's singleton reference and then this
 * node's last remaining shared_ptr, in that order, before calling
 * rclcpp::shutdown() -- so ~FlatlandServerNode() (and whatever rmw/DDS
 * teardown it triggers) runs synchronously on the main thread while every
 * library is still fully mapped, not from an exit-time destructor racing
 * DDS's own receive threads.
 */

class FlatlandServerNode : public rclcpp::Node
{
public:
  FlatlandServerNode() : Node("flatland_server")
  {
    declare_parameter<std::string>("world_path");
    declare_parameter<float>("update_rate");
    declare_parameter<float>("step_size");
    declare_parameter<bool>("show_viz");
    declare_parameter<float>("viz_pub_rate");

    // Load parameters
    if (!get_parameter("world_path", world_path_)) {
      RCLCPP_INFO(get_logger(), "No world_path parameter given!");
      rclcpp::shutdown();
      return;
    }
    get_parameter_or<float>("update_rate", update_rate_, 200.0f);
    get_parameter_or<float>("step_size", step_size_, 1.0f / 200.0f);
    get_parameter_or<bool>("show_viz", show_viz_, false);
    get_parameter_or<float>("viz_pub_rate", viz_pub_rate_, 30.0f);
  }

  void Run()
  {
    // Create simulation manager object
    simulation_manager_ = std::make_shared<flatland_server::SimulationManager>(
      shared_from_this(), world_path_, update_rate_, step_size_, show_viz_, viz_pub_rate_);

    RCLCPP_INFO(this->get_logger(), "Initialized");
    simulation_manager_->Main();

    RCLCPP_INFO(this->get_logger(), "Returned from simulation manager main2");

    // Break the FlatlandServerNode <-> SimulationManager shared_ptr cycle:
    // SimulationManager::node_ (simulation_manager.h) holds a shared_ptr
    // back to this node. Without this reset neither object's refcount can
    // reach zero, so ~FlatlandServerNode() never runs at all -- see the
    // file header comment above for the full reference-cycle explanation.
    simulation_manager_.reset();
  }

  // TODO: Allow updates to step size, update rate etc. with new ros2 dynamic
  // params

private:
  std::string world_path_;  // The file path to the world.yaml file
  float update_rate_;       // The physics update rate (Hz)
  float step_size_;
  bool show_viz_;
  float viz_pub_rate_;
  std::shared_ptr<flatland_server::SimulationManager> simulation_manager_;
};

/**
 * @name        main
 * @brief       Entrypoint for Flatland Server ros node
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto flatland_server = std::make_shared<FlatlandServerNode>();
  flatland_server->Run();

  // Explicit, ordered teardown while the process, rclcpp context and every
  // library are still fully mapped. Run() already dropped this node's own
  // reference to SimulationManager; DebugVisualization::Reset() drops the
  // debug-viz singleton's reference (debug_visualization.h) -- the last
  // other place this node's shared_ptr refcount could still be pinned.
  // Resetting `flatland_server` then drops the only remaining reference,
  // so ~FlatlandServerNode() runs synchronously right here, not from an
  // exit-time destructor racing Fast DDS's own receive threads. See the
  // file header comment above for the full reference-cycle explanation.
  flatland_server::DebugVisualization::Reset();
  flatland_server.reset();
  rclcpp::shutdown();
  return 0;
}
