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

#include <atomic>
#include <csignal>

#include <string>

#include "flatland_server/simulation_manager.h"
#include "rclcpp/rclcpp.hpp"

/**
 * The default rclcpp::init() leaves shutdown_on_signal=true, which tears the
 * global context down from rclcpp's own internal signal-handling thread the
 * instant SIGINT is delivered -- asynchronously with respect to wherever the
 * simulation loop happens to be. That race produces two distinct teardown
 * failures: an abort inside the loop, where a fresh SingleThreadedExecutor's
 * GuardCondition is constructed against a context that has just been shut
 * down mid-iteration; and a module-unload SIGSEGV at process exit, where Fast
 * DDS's UDP receive threads are still executing while an exit handler
 * dlclose()s the rmw cluster out from under them, because the loop -- and
 * everything downstream of it -- never reaches a quiescent state before the
 * context dies.
 *
 * The fix: disable shutdown_on_signal and register our own SIGINT handler
 * that only flips SimulationManager::run_simulator_, the loop's own exit
 * flag. That lets the main thread -- and only the main thread -- decide when
 * the loop is done, drive the world/thread-pool teardown that follows it,
 * and only then call rclcpp::shutdown() once, from a known, non-signal
 * context. A signal handler must stay async-signal-safe, so it does nothing
 * but this store; no logging, no rclcpp calls.
 */
namespace
{
std::atomic<flatland_server::SimulationManager *> g_active_simulation_manager{nullptr};

extern "C" void SigintHandler(int /*sig*/)
{
  flatland_server::SimulationManager * manager =
    g_active_simulation_manager.load(std::memory_order_acquire);
  if (manager != nullptr) {
    manager->run_simulator_ = false;
  }
}
}  // namespace

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

    // Published for SigintHandler's duration only: the handler must reach a
    // live SimulationManager without calling into anything non-signal-safe.
    g_active_simulation_manager.store(simulation_manager_.get(), std::memory_order_release);

    RCLCPP_INFO(this->get_logger(), "Initialized");
    simulation_manager_->Main();
    g_active_simulation_manager.store(nullptr, std::memory_order_release);

    RCLCPP_INFO(this->get_logger(), "Returned from simulation manager main2");
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
  // shutdown_on_signal=false: see the comment above SigintHandler. Shutdown
  // is driven from here, once, after Run() returns with the loop already
  // stopped and the world already torn down -- not from rclcpp's internal
  // signal-handling thread while either is still live.
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);

  struct sigaction sigint_action = {};
  sigint_action.sa_handler = SigintHandler;
  sigemptyset(&sigint_action.sa_mask);
  sigint_action.sa_flags = 0;
  sigaction(SIGINT, &sigint_action, nullptr);

  auto flatland_server = std::make_shared<FlatlandServerNode>();
  flatland_server->Run();
  rclcpp::shutdown();
  return 0;
}
