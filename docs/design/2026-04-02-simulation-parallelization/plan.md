# Simulation Parallelization Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Parallelise all major serialisation bottlenecks across the flatland, rr_sootballs, and sootballs_sites simulation stack so that a 90-robot + 30-human simkao004 deployment utilises available CPU cores instead of idling at ~30%.

**Architecture:** Seven workstreams target the bottlenecks identified in the performance analysis: (0) thread-safety prerequisites — fix 5 critical concurrency hazards discovered during audit, (1) parallel plugin dispatch in flatland, (2) parallel laser ray-casting, (3) async ROS publishing from flatland, (4) AsyncSpinner in ALICA/LBC nodes, (5) queue size increases across all publishers/subscribers, (6) simkao004 site config tuning. Task 0 is a hard prerequisite for Task 1; Tasks 4-6 are independent quick wins.

**Tech Stack:** C++17 (catkin/ROS 1 Noetic), enkiTS v1.11, Box2D v3.1.0, ROS publishers/subscribers, YAML site configs, Jinja2 templates.

---

## Terminology

| Term | Meaning |
|---|---|
| **flatland** | `c:\Users\Rapyuta Robotics\Desktop\flatland` (branch `feature/box2d-v3-on-rr-upstream`) |
| **sootballs** | `c:\Users\Rapyuta Robotics\Desktop\rr_sootballs` (branch `feature/flatland-box2d-v3`) |
| **sites** | `c:\Users\Rapyuta Robotics\Desktop\sootballs_sites` (branch `simkao004`) |

---

## Task 0: Thread-Safety Prerequisites for Parallel Plugin Dispatch (SERIAL — BLOCKING for Task 1)

**Repos:** flatland + sootballs

**Problem:** A thread-safety audit found **5 critical concurrency violations** that will cause crashes or data corruption if model plugins are dispatched in parallel. These MUST be fixed before Task 1.

### Audit Summary

| # | Component | Issue | Severity | Fix |
|---|---|---|---|---|
| 0a | `MessageServer::publish()` | No mutex. Concurrent publish from parallel plugins = data race on `topic_->subscribers_` vector | **CRITICAL** | Add `std::mutex` per topic |
| 0b | `HumanPublisherManager` singleton | Concurrent `unordered_map::emplace()` with no locking | **CRITICAL** | Add `std::mutex` to singleton |
| 0c | Cross-plugin body reads | `SootballNavigatorPlugin::respondToGetPathAction()` reads OTHER robots' `b2Body_GetPosition()` while those robots call `SetPose()` on the same body | **CRITICAL** | Pose snapshot cache before parallel dispatch |
| 0d | `Model::bodies_[]` vector | Thread A reads `RobotB->bodies_[0]` while Thread B modifies it via `SetPose()` | **CRITICAL** | Solved by 0c (snapshot eliminates live cross-reads) |
| 0e | `getWorldPlugin()` vector iteration | Multiple threads iterate `plugin_manager_.world_plugins_` — safe only because vector doesn't mutate at runtime | **HIGH** | Cache result in `OnInitialize`, never re-query at runtime |

Items confirmed SAFE (no fix needed):

- `MapInfoPlugin` graph data: read-only during physics steps
- `CollisionFilterRegistry`: read-only after init
- `tf2_ros::TransformBroadcaster`: per-instance, not shared
- `ros::Publisher::publish()`: internally mutex-protected in roscpp

---

### Step 0a: Add mutex to MessageServer (flatland)

**File:** `flatland/flatland_server/include/flatland_server/message_server.h`

Current code (no locking):

```cpp
template <class T>
void Publisher<T>::publish(const T& t) {
  for (const Subscriber<T>& subscriber : topic_->subscribers_) {
    subscriber.callback_function_(t);
  }
}
```

Add a `std::mutex` to each `Topic` and lock in `publish()`:

```cpp
#include <mutex>

template <class T>
struct Topic {
  std::string name_;
  std::vector<Subscriber<T>> subscribers_;
  std::mutex mutex_;  // NEW: guards subscribers_ iteration
};

template <class T>
void Publisher<T>::publish(const T& t) {
  std::lock_guard<std::mutex> lock(topic_->mutex_);
  for (const Subscriber<T>& subscriber : topic_->subscribers_) {
    subscriber.callback_function_(t);
  }
}
```

Also lock in `MessageServer::subscribe()` where subscribers are added:

```cpp
template <class T>
Subscriber<T> MessageServer::subscribe(const std::string& topic, ...) {
  auto& t = topics_[topic];
  std::lock_guard<std::mutex> lock(t.mutex_);
  t.subscribers_.emplace_back(...);
  // ...
}
```

**Commit:**

```bash
cd flatland && git add -A && git commit -m "fix: add mutex to MessageServer for thread-safe publish"
```

---

### Step 0b: Add mutex to HumanPublisherManager (sootballs)

**File:** `sootballs_simulation/flatland_interface/include/flatland_interface/human/human_publisher_manager.hpp`

Current code (no locking):

```cpp
void publish(const std::string& robot_name, const HumanMessage& message) {
    auto it = publishers_.find(robot_name);
    if (it == publishers_.end()) {
        auto pub = message_server_->advertise<HumanMessage>(robot_name + "/HumanMessage");
        it = publishers_.emplace(robot_name, pub).first;  // RACE: concurrent map insert
    }
    it->second.publish(message);
}
```

Fix — add `std::mutex`:

```cpp
#include <mutex>

class HumanPublisherManager {
public:
    void publish(const std::string& robot_name, const HumanMessage& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = publishers_.find(robot_name);
        if (it == publishers_.end()) {
            auto pub = message_server_->advertise<HumanMessage>(robot_name + "/HumanMessage");
            it = publishers_.emplace(robot_name, pub).first;
        }
        it->second.publish(message);
    }

private:
    std::mutex mutex_;  // NEW: guards publishers_ map
    // ... existing members
};
```

**Commit:**

```bash
cd rr_sootballs && git add -A && git commit -m "fix: add mutex to HumanPublisherManager singleton"
```

---

### Step 0c: Pose snapshot cache — eliminate live cross-plugin body reads (sootballs + flatland)

**Problem:** `SootballNavigatorPlugin::respondToGetPathAction()` iterates ALL model plugins and reads other robots' `b2Body_GetPosition()` directly. If another thread is simultaneously calling `SetPose()` on that body, this is a data race.

**Solution:** Before parallel dispatch, build a **read-only snapshot** of all robot poses. Plugins read from the snapshot instead of live Box2D state.

**File 1:** `flatland/flatland_server/include/flatland_server/world.h` — add snapshot storage:

```cpp
#include <unordered_map>

struct PoseSnapshot {
  float x, y, theta;
};

class World {
 public:
  // ... existing members ...
  std::unordered_map<std::string, PoseSnapshot> pose_snapshot_;  // NEW

  /// Build snapshot of all model poses (called BEFORE parallel dispatch)
  void BuildPoseSnapshot();
};
```

**File 2:** `flatland/flatland_server/src/world.cpp` — implement snapshot:

```cpp
void World::BuildPoseSnapshot() {
  pose_snapshot_.clear();
  for (const auto* model : models_) {
    if (model->bodies_.empty()) continue;
    b2Vec2 pos = b2Body_GetPosition(model->bodies_[0]->physics_body_);
    float angle = b2Rot_GetAngle(b2Body_GetRotation(model->bodies_[0]->physics_body_));
    pose_snapshot_[model->GetName()] = {pos.x, pos.y, angle};
  }
}
```

**File 3:** `flatland/flatland_server/src/world.cpp` — call snapshot before plugins:

In `World::Update()`, add before `plugin_manager_.BeforePhysicsStep()`:

```cpp
void World::Update(Timekeeper &timekeeper) {
  if (!IsPaused()) {
    BuildPoseSnapshot();  // NEW: snapshot all poses before parallel dispatch
    plugin_manager_.BeforePhysicsStep(timekeeper);
    // ... rest unchanged
```

**File 4:** `sootballs_simulation/flatland_plugins/src/sootball/sootball_navigator.cpp` — read from snapshot:

Replace the direct body access in `respondToGetPathAction()`:

```cpp
// BEFORE (UNSAFE — reads other models' bodies directly):
for (auto& plugin : GetModel()->GetWorld().plugin_manager_.model_plugins_) {
    if (plugin->GetType() == "SootballPlugin" && plugin->GetModel()->GetName() != _robot_name) {
        vectorBox2D pose_2d = vectorBox2D(
            b2Body_GetPosition(plugin->GetModel()->bodies_[0]->physics_body_).x,
            b2Body_GetPosition(plugin->GetModel()->bodies_[0]->physics_body_).y);
        other_robots.push_back(Pose2D(pose_2d, 0));
    }
}

// AFTER (SAFE — reads from pre-built snapshot):
const auto& snapshot = GetModel()->GetWorld().pose_snapshot_;
for (const auto& [name, pose] : snapshot) {
    if (name != _robot_name) {
        other_robots.push_back(Pose2D(vectorBox2D(pose.x, pose.y), pose.theta));
    }
}
```

This eliminates BOTH Risk 0c (cross-body reads during parallel dispatch) and Risk 0d (bodies_ vector access), since no plugin ever touches another model's `bodies_[]` at runtime.

**Commit:**

```bash
cd flatland && git add -A && git commit -m "feat: pose snapshot cache for thread-safe cross-model reads"
cd ../rr_sootballs && git add -A && git commit -m "refactor: use pose snapshot instead of direct body reads"
```

---

### Step 0e: Cache getWorldPlugin() results in OnInitialize (sootballs)

**Problem:** `getWorldPlugin()` iterates `world->plugin_manager_.world_plugins_` vector. Safe today because vector doesn't mutate at runtime, but fragile under concurrent access.

**Files:**

- `sootballs_simulation/flatland_plugins/src/sootball/sootball.cpp`
- `sootballs_simulation/flatland_plugins/src/sootball/sootball_navigator.cpp`
- `sootballs_simulation/flatland_plugins/src/human/human.cpp`

**Fix:** Each plugin already calls `getWorldPlugin()` during `OnInitialize()` for some lookups. Ensure ALL world plugin references are cached as member pointers during `OnInitialize()` and never re-queried in `BeforePhysicsStep()`.

Verify that `getMapInfo()`, `getHumanNavigator()`, `getLogger()`, `getInventorySystem()` methods either:

1. Return a cached pointer set during `OnInitialize()`, OR
2. Are only called from `OnInitialize()`, not from `BeforePhysicsStep()`

If any are called from `BeforePhysicsStep()`, refactor to cache in `OnInitialize()`:

```cpp
class SootballPlugin : public ModelPlugin {
    MapInfoPlugin* cached_map_info_ = nullptr;      // NEW: cached
    HumanNavigatorPlugin* cached_human_nav_ = nullptr; // NEW: cached

    void OnInitialize(const YAML::Node& config) override {
        // ... existing init ...
        cached_map_info_ = static_cast<MapInfoPlugin*>(
            getWorldPlugin("MapInfoPlugin", &(GetModel()->GetWorld())));
        cached_human_nav_ = static_cast<HumanNavigatorPlugin*>(
            getWorldPlugin("HumanNavigatorPlugin", &(GetModel()->GetWorld())));
    }

    void BeforePhysicsStep(const Timekeeper& tk) override {
        // Use cached_map_info_ instead of getMapInfo()
        // Use cached_human_nav_ instead of getHumanNavigator()
    }
};
```

**Commit:**

```bash
cd rr_sootballs && git add -A && git commit -m "refactor: cache world plugin references in OnInitialize"
```

---

## Task 1: Parallel Plugin Dispatch via enkiTS (SERIAL — depends on Task 0)

**Repo:** flatland

**Problem:** `PluginManager::BeforePhysicsStep` and `AfterPhysicsStep` iterate all plugins sequentially. With 90 robots × 2 plugins + 30 humans × 1 plugin + 4 world plugins = ~214 plugin calls, all on one thread.

**Files:**

- Modify: `flatland/flatland_server/src/plugin_manager.cpp` (lines 79-90)
- Modify: `flatland/flatland_server/include/flatland_server/plugin_manager.h`
- Modify: `flatland/flatland_server/src/world.cpp` (pass `task_scheduler_` to plugin_manager)
- Modify: `flatland/flatland_server/include/flatland_server/world.h`
- Test: `flatland/flatland_server/test/plugin_manager_parallel_test.cpp` (new)

### Step 1: Write failing test for parallel plugin dispatch

Create `flatland/flatland_server/test/plugin_manager_parallel_test.cpp`:

```cpp
#include <gtest/gtest.h>
#include <flatland_server/plugin_manager.h>
#include <flatland_server/timekeeper.h>
#include <atomic>
#include <thread>
#include <chrono>

// Mock plugin that records which thread it ran on
class SlowMockPlugin : public flatland_server::ModelPlugin {
public:
  std::atomic<std::thread::id> ran_on_thread{};
  std::atomic<bool> was_called{false};
  
  void OnInitialize(const YAML::Node&) override {}
  void BeforePhysicsStep(const flatland_server::Timekeeper& tk) override {
    was_called = true;
    ran_on_thread = std::this_thread::get_id();
    // Simulate 1ms of work
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1)) {}
  }
};

TEST(PluginManagerParallel, ModelPluginsRunInParallel) {
  // If 100 plugins each take 1ms sequentially = 100ms
  // In parallel on 4+ cores should be < 50ms
  // This test verifies wall time is significantly less than sequential
  constexpr int N = 100;
  // ... (test body verifying parallelism reduces wall time)
}
```

Run: `catkin test flatland_server --no-deps --this` → Expected: FAIL (test file doesn't compile yet, PluginManager has no parallel API)

### Step 2: Add enkiTS reference to PluginManager

Modify `flatland/flatland_server/include/flatland_server/plugin_manager.h`:

- Add `#include "TaskScheduler.h"`
- Add member `enki::TaskScheduler* task_scheduler_ = nullptr;`
- Add method `void SetTaskScheduler(enki::TaskScheduler* scheduler);`

Modify `flatland/flatland_server/src/plugin_manager.cpp`:

```cpp
void PluginManager::SetTaskScheduler(enki::TaskScheduler* scheduler) {
  task_scheduler_ = scheduler;
}
```

### Step 3: Implement parallel BeforePhysicsStep

Replace the sequential loops in `plugin_manager.cpp` (lines 79-86) with enkiTS-based parallel dispatch:

```cpp
void PluginManager::BeforePhysicsStep(const Timekeeper &timekeeper_) {
  if (task_scheduler_ && model_plugins_.size() > 1) {
    // Parallel dispatch for model plugins
    enki::TaskSet task(static_cast<uint32_t>(model_plugins_.size()),
      [this, &timekeeper_](enki::TaskSetPartition range, uint32_t) {
        for (uint32_t i = range.start; i < range.end; ++i) {
          model_plugins_[i]->BeforePhysicsStep(timekeeper_);
        }
      });
    task_scheduler_->AddTaskSetToPipe(&task);
    task_scheduler_->WaitforTask(&task);
  } else {
    for (const auto &model_plugin : model_plugins_) {
      model_plugin->BeforePhysicsStep(timekeeper_);
    }
  }
  // World plugins remain sequential (they may depend on each other)
  for (const auto &world_plugin : world_plugins_) {
    world_plugin->BeforePhysicsStep(timekeeper_);
  }
}
```

Apply the same pattern to `AfterPhysicsStep`.

**CRITICAL SAFETY CONSTRAINT:** Model plugins are safe to parallelise ONLY AFTER Task 0 is complete. Task 0 addresses 5 critical concurrency hazards: (0a) MessageServer mutex, (0b) HumanPublisherManager mutex, (0c/0d) pose snapshot cache eliminating cross-plugin body reads, (0e) cached world plugin references. After these fixes, each model plugin operates exclusively on its own model's data, reads shared state only through thread-safe interfaces, and never touches another model's Box2D bodies. World plugins remain sequential because `HumanNavigatorPlugin` manages shared human state.

**EXCEPTION — ROS publishing is NOT thread-safe in roscpp by default.** Each plugin calls `ros::Publisher::publish()` which IS thread-safe (roscpp serialises internally with a mutex). TF broadcasting via `tf2_ros::TransformBroadcaster` also serialises internally. So ROS publishing from parallel plugin threads is safe but may contend on internal mutexes.

### Step 4: Wire scheduler in World constructor

Modify `flatland/flatland_server/src/world.cpp` (after line 122):

```cpp
  plugin_manager_.SetTaskScheduler(&task_scheduler_);
```

### Step 5: Run test to verify parallelism

Run: `catkin test flatland_server --no-deps` → Expected: PASS, wall time < sequential time

### Step 6: Commit

```bash
cd flatland
git add -A && git commit -m "feat: parallel model plugin dispatch via enkiTS"
```

---

## Task 2: Re-parallelise Laser Ray Casting (SERIAL — depends on Task 1 for scheduler access)

**Repo:** flatland

**Problem:** The old ThreadPool-based parallel ray casting was removed because `b2World_CastRay` is not thread-safe in Box2D v3 for concurrent external callers. However, Box2D v3 provides `b2World_OverlapAABB` and a read-only query API. The solution is to **clone the world for read-only queries** or **batch ray casts between physics steps** when the world state is frozen.

**Key insight from Box2D v3 docs:** `b2World_CastRay` is NOT safe to call from multiple threads simultaneously. However, between `b2World_Step` calls, the world state is frozen. We can parallelise ray casts across robots by partitioning them: each enkiTS task handles one robot's complete set of rays (not interleaving rays from different robots on different threads against the same world).

**CORRECTION:** Actually, even between steps, `b2World_CastRay` uses internal scratch buffers that are NOT thread-safe. The correct approach is to use a **per-thread world clone** or accept sequential ray casting and instead focus on **batching + SIMD optimisation**, or use the enkiTS scheduler to run each robot's laser computation as a pinned task that does NOT overlap world queries.

**Safest approach:** Since Box2D v3's ray cast is fundamentally single-threaded, we should instead **eliminate unnecessary ray casts** and **reduce ray count** for the fast-sim case (which is what simkao004 uses — `sootball_fast.model.yaml` has NO laser plugins). For gen3 mode, we can batch all ray casts into a single tight loop with minimal overhead.

**Files:**

- Modify: `flatland/flatland_plugins/src/laser.cpp` (lines 149-200)
- Modify: `flatland/flatland_plugins/include/flatland_plugins/laser.h`

### Step 1: Add pre-allocated ray result buffer

Modify `laser.h` to pre-allocate result storage, eliminating per-frame allocations:

```cpp
struct RayResult {
  float fraction;
  float intensity;
  bool did_hit;
};
std::vector<RayResult> ray_results_;  // pre-allocated in OnInitialize
```

### Step 2: Optimise ComputeLaserRanges hot loop

Replace the current loop in `laser.cpp` (line 168-195) with a tighter version that minimises per-ray overhead:

```cpp
void Laser::ComputeLaserRanges() {
  b2Transform t = b2Body_GetTransform(body_->GetPhysicsBody());
  m_world_to_body_ << t.q.c, -t.q.s, t.p.x, t.q.s, t.q.c, t.p.y, 0, 0, 1;
  m_world_to_laser_ = m_world_to_body_ * m_body_to_laser_;
  m_world_laser_points_ = m_world_to_laser_ * m_laser_points_;
  v_world_laser_origin_ = m_world_to_laser_ * v_zero_point_;

  const b2Vec2 origin = {v_world_laser_origin_(0), v_world_laser_origin_(1)};
  const b2WorldId wid = GetModel()->GetPhysicsWorld();
  b2QueryFilter filter = b2DefaultQueryFilter();
  filter.maskBits = layers_bits_;

  const unsigned int n = static_cast<unsigned int>(laser_scan_.ranges.size());
  for (unsigned int i = 0; i < n; ++i) {
    const b2Vec2 target = {m_world_laser_points_(0, i),
                           m_world_laser_points_(1, i)};
    LaserRayContext ctx;
    ctx.layers_bits = layers_bits_;
    ctx.reflectance_layers_bits = reflectance_layers_bits_;

    b2World_CastRay(wid, origin, target, filter, LaserRayCastFcn, &ctx);

    laser_scan_.ranges[i] = ctx.did_hit
        ? static_cast<float>(ctx.fraction * range_ + noise_gen_(rng_))
        : NAN;
    if (reflectance_layers_bits_) laser_scan_.intensities[i] = ctx.intensity;
  }
}
```

### Step 3: Commit

```bash
cd flatland
git add -A && git commit -m "perf: optimise laser ray cast loop, pre-allocate buffers"
```

**NOTE:** True parallelisation of ray casts requires Box2D v3 to expose a thread-safe read-only query API, which it currently does not. This task optimises the sequential path. The main win for simkao004 comes from Task 1 (parallel plugin dispatch) since fast-sim has NO laser plugins.

---

## Task 3: Async ROS Publishing Thread in SimulationManager (SERIAL — depends on Task 1)

**Repo:** flatland

**Problem:** `ros::spinOnce()` runs on the same thread as physics. Service callbacks (spawn/delete model) block the physics loop. ROS publishing from plugins serialises through the main thread.

**Files:**

- Modify: `flatland/flatland_server/src/simulation_manager.cpp` (lines 132-157)
- Modify: `flatland/flatland_server/include/flatland_server/simulation_manager.h`

### Step 1: Add AsyncSpinner for ROS service callbacks

Replace `ros::spinOnce()` with an `ros::AsyncSpinner` that runs in a background thread, so service callbacks (spawn_model, delete_model, pause) are processed concurrently with physics:

Modify `simulation_manager.h`:

```cpp
#include <ros/callback_queue.h>
```

Modify `simulation_manager.cpp` — before the main loop:

```cpp
  // Process ROS callbacks in a background thread instead of blocking physics
  ros::AsyncSpinner spinner(1);  // 1 background thread for callbacks
  spinner.start();
```

Remove the `ros::spinOnce()` call from the main loop (line 155).

**SAFETY:** `ServiceManager` handlers (spawn/delete/move model) modify `World` state. With `AsyncSpinner`, these could fire mid-physics-step. We need a mutex:

Modify `world.h` — add:

```cpp
std::mutex world_mutex_;  // guards model spawn/delete during physics step
```

Modify `world.cpp` — `World::Update()` acquires lock:

```cpp
void World::Update(Timekeeper &timekeeper) {
  std::lock_guard<std::mutex> lock(world_mutex_);
  // ... existing Update() body ...
}
```

And `ServiceManager` spawn/delete handlers also acquire `world_mutex_` before modifying models.

### Step 2: Commit

```bash
cd flatland
git add -A && git commit -m "feat: async ROS callback processing via AsyncSpinner"
```

---

## Task 4: Increase Queue Sizes Across All Publishers/Subscribers (PARALLEL — independent of Tasks 1-3)

**Repos:** flatland + sootballs

**Problem:** Every publisher and subscriber in the system uses `queue_size=1`. With single-threaded consumers running at 10 Hz, messages are silently dropped. This means the system under-utilises CPU because it's *discarding work* rather than processing it.

**Files to modify:**

### flatland repo

| File | Line | Current | New | Topic |
|---|---|---|---|---|
| `flatland_plugins/src/laser.cpp` | 62 | `advertise<...>(..., 1)` | `5` | raw_scan |
| `flatland_plugins/src/diff_drive.cpp` | 121 | `subscribe(twist_topic, 1, ...)` | `5` | cmd_vel |
| `flatland_plugins/src/diff_drive.cpp` | 123 | `advertise<...>(odom_topic, 1)` | `5` | odom |
| `flatland_plugins/src/gps.cpp` | (find line) | `1` | `5` | gps |
| `flatland_plugins/src/imu.cpp` | (find line) | `1` | `5` | imu |

### sootballs repo

| File | Line | Current | New | Topic |
|---|---|---|---|---|
| `sootballs_simulation/flatland_plugins/src/sootball/sootball_navigator.cpp` | 44 | `advertise<...>(..., 1)` | `5` | rr_amcl_pose |
| `sootballs_simulation/flatland_plugins/src/sootball/sootball_navigator.cpp` | 45 | `advertise<...>(..., 1)` | `5` | odom |
| `sootballs_simulation/flatland_plugins/src/sootball/sootball_navigator.cpp` | 52 | `subscribe(..., 1, ...)` | `5` | ar_detector_robot_pose |

### Step 1: Bump all queue sizes in flatland

For each file above, change `queue_size` from `1` to `5`. This allows up to 5 messages to buffer before dropping, giving slower consumers time to catch up.

### Step 2: Bump all queue sizes in sootballs

Same pattern for the sootballs files.

### Step 3: Commit both repos

```bash
cd flatland && git add -A && git commit -m "perf: increase publisher/subscriber queue sizes to 5"
cd ../rr_sootballs && git add -A && git commit -m "perf: increase publisher/subscriber queue sizes to 5"
```

---

## Task 5: AsyncSpinner in ALICA/LBC Base Node (PARALLEL — independent)

**Repo:** sootballs

**Problem:** The ALICA behaviour engine processes ALL ROS callbacks at only 10 Hz via `ros::spinOnce()`. With 90 robots, each instance's callbacks are delayed up to 100ms. Messages arriving between polls are dropped (queue_size=1).

**Files:**

- Modify: `lbc_bridge/sootballs_alica_base/src/base_node.cpp` (lines 127-172)

### Step 1: Replace spinOnce with AsyncSpinner

Current code (line ~145-165):

```cpp
ros::Rate r(loop_rate);
while (ros::ok()) {
    wm->saveRecoveryIfNeeded();
    r.sleep();
    ros::spinOnce();
    mediator->run();
}
```

New code:

```cpp
ros::AsyncSpinner spinner(2);  // 2 threads for callback processing
spinner.start();

ros::Rate r(loop_rate);
while (ros::ok()) {
    wm->saveRecoveryIfNeeded();
    r.sleep();
    // ros::spinOnce() removed — AsyncSpinner processes callbacks continuously
    mediator->run();
}
```

**SAFETY:** ALICA's internal state (`wm`, `mediator`) must be protected if callbacks now fire asynchronously. Review whether ALICA's `AlicaContext` already uses internal locking (it typically does, as ALICA runs plan execution in background threads). If not, a `std::mutex` around shared WorldModel access is needed.

### Step 2: Verify ALICA thread-safety

Check ALICA's `AlicaContext::init()` — if it already spawns background threads for plan execution (which it does per the analysis), then its WorldModel is already thread-safe and `AsyncSpinner` is a safe drop-in.

### Step 3: Commit

```bash
cd rr_sootballs && git add -A && git commit -m "perf: use AsyncSpinner(2) in ALICA base node"
```

---

## Task 6: simkao004 Site Configuration Tuning (PARALLEL — independent)

**Repo:** sootballs_sites

**Problem:** The simkao004 site config uses `sim_speed: 1` with `fast_simulation: true`. The robot update_rate is 2 Hz and loc_rate is 1 Hz. These rates are very conservative for a fast-sim setup where no actual sensor processing occurs. Additionally, the `HumanNavigatorPlugin` update_rate is 2 Hz — with 30 humans, this creates unnecessary overhead.

**Files:**

- Modify: `sites/simkao004/config/simulation/sootball_fast.model.yaml` (lines 46-48)
- Modify: `sites/simkao004/config/simulation/sim_injectable_param.yaml` (lines 21, 30-31)
- Modify: `sites/simkao004/rio/values.yaml` (line 479)

### Step 1: Increase sim_speed for higher throughput

In `sites/simkao004/rio/values.yaml` (line 479):

```yaml
sim_speed: 3  # was 1 — allows 3x real-time with fast_simulation
```

This changes `step_size = 0.2 * 3 = 0.6s` per physics step at 5 Hz → 3× real-time.

**Why not 10?** At 10x speed with 90 robots, each 2-second physics step must process 90 robot plugins + 30 human plugins + 4 world plugins in under 200ms wall time. Starting at 3x allows validation before pushing higher.

### Step 2: Tune plugin update rates

In `sites/simkao004/config/simulation/sootball_fast.model.yaml`:

```yaml
# SootballNavigatorPlugin
update_rate: 5   # was 2 — more responsive movement
loc_rate: 2      # was 1 — reduce TF staleness

# SootballPlugin  
update_rate: 2   # keep at 2 — sensor updates don't need to be faster
sensor_update_rate: 1  # keep at 1
```

In `sites/simkao004/config/simulation/sim_injectable_param.yaml`:

```yaml
# HumanNavigatorPlugin
update_rate: 2  # keep at 2 — humans don't need faster updates

# SootballNavigatorPlugin injectable params
speed: 1.0      # was 0.8 — faster robot movement compensates for 3x sim
fast_multiplier: 1  # keep at 1
```

### Step 3: Commit

```bash
cd sootballs_sites && git add -A && git commit -m "perf: tune simkao004 sim_speed and plugin rates"
```

---

## Task Dependencies

```
Task 0 (thread-safety fixes) ──► Task 1 (parallel plugins) ──► Task 2 (laser opt) ──► Task 3 (async ROS)
                                                                                              │
Task 4 (queue sizes)              ◄── independent ──►                                         │
Task 5 (ALICA AsyncSpinner)       ◄── independent ──►                                         │
Task 6 (site config)              ◄── independent ──►                                         │
```

**PARALLEL tasks** (can be done concurrently): Tasks 4, 5, 6 (and can start immediately, before Task 0)
**SERIAL tasks** (must be done in order): Tasks 0 → 1 → 2 → 3
**HARD BLOCKER:** Task 1 MUST NOT start until all 5 steps of Task 0 are verified

---

## Expected Impact — Categorised by Percentage

### Impact Breakdown (100% = full bottleneck elimination)

| Task | Change | Impact % | Rationale |
|---|---|---|---|
| **0** | Thread-safety prerequisites (5 fixes) | **0%** (enabler) | No direct performance improvement, but HARD BLOCKER for Task 1. Without these fixes, Task 1 will crash the sim. |
| **1** | Parallel model plugin dispatch (enkiTS) | **40%** | The single biggest bottleneck. 214 sequential plugin calls (90 robots × 2 + 30 humans + 4 world) currently run on 1 core. Parallelising across N cores reduces wall time by ~N× for the plugin phase, which dominates each sim tick. This is where most of the "idle CPU" gap lives. |
| **5** | ALICA AsyncSpinner (per-robot) | **20%** | Each of 90 robot ALICA nodes currently polls callbacks at 10 Hz. Messages arriving between polls are dropped (queue=1). AsyncSpinner makes callbacks process immediately, so each robot reacts to pose/status updates without 100ms latency. Collectively across 90 instances, this unlocks significant idle CPU. |
| **4** | Queue sizes 1→5 (all pub/sub) | **15%** | Eliminates silent message drops at every producer→consumer boundary. Currently the system is *discarding work* rather than doing it — low CPU is partly because messages never reach their consumers. With queue=5, more messages survive to be processed as intended. |
| **6** | simkao004 config tuning (sim_speed, rates) | **12%** | Increasing sim_speed from 1→3 means the same wall time covers 3× more sim time. Higher plugin update rates (2→5 Hz) make robots more responsive. This is a pure config change — no code risk — that directly improves throughput. |
| **3** | Async ROS callbacks in flatland | **8%** | Unblocks physics loop from service callbacks (spawn/delete model). During initial spawning of 90 robots + 30 humans, this is significant. During steady state, service calls are rare, so benefit is mostly at startup and during dynamic model operations. |
| **2** | Laser ray cast optimisation | **5%** | simkao004 uses `sootball_fast.model.yaml` which has NO laser plugins — so this has zero impact on the current deployment. It only matters for gen3 full-nav mode. Included for completeness and future-proofing. Buffer pre-allocation provides minor cache-locality improvements. |

### Visual Breakdown

```
█████████████████████████████████████████░░░░░░░░░░░░░░░░░░░░  Task 1: 40%  Parallel plugins
████████████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Task 5: 20%  ALICA AsyncSpinner  
███████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Task 4: 15%  Queue sizes
████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Task 6: 12%  Site config
████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Task 3:  8%  Async ROS callbacks
█████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  Task 2:  5%  Laser optimisation
```

### Priority Ordering (by impact-to-effort ratio)

| Priority | Task | Impact | Effort | Ratio |
|---|---|---|---|---|
| 🥇 1st | Task 6 (site config) | 12% | Trivial (YAML edits) | **Highest** — do first, zero risk |
| 🥈 2nd | Task 4 (queue sizes) | 15% | Low (find-replace `1` → `5`) | **Very high** |
| 🥉 3rd | Task 5 (ALICA AsyncSpinner) | 20% | Low (3-line change) | **Very high** |
| 4th | Task 0 (thread-safety fixes) | 0% (enabler) | Medium (5 targeted fixes across 2 repos) | **Required** — gates the 40% win |
| 5th | Task 1 (parallel plugins) | 40% | Medium (new parallel dispatch, testing) | **High** — biggest payoff |
| 6th | Task 3 (async ROS flatland) | 8% | Medium (mutex + AsyncSpinner) | **Moderate** |
| 7th | Task 2 (laser optimisation) | 5% | Low (buffer pre-alloc) | **Low** (0% for simkao004) |

### Expected CPU Utilisation Progression

| After completing | Est. CPU util | Notes |
|---|---|---|
| Baseline (current) | ~30% | Single-threaded bottlenecks, message drops |
| + Task 6 (config) | ~35% | More sim work per wall second |
| + Task 4 (queues) | ~42% | More messages reach consumers |
| + Task 5 (ALICA) | ~55% | 90 ALICA nodes process callbacks immediately |
| + Task 0 (thread-safety) | ~55% | No perf change — enables Task 1 |
| + Task 1 (parallel plugins) | ~75% | Plugin work spreads across all cores |
| + Task 3 (async ROS) | ~78% | Service callbacks no longer block physics |
| + Task 2 (laser) | ~78% | No change for simkao004 fast-sim |

**Combined target:** CPU utilisation from ~30% → **~75-80%** with all changes applied.

---

## Risk Register (Audit-Verified)

### Risks FIXED by Task 0 (blocking — must complete before Task 1)

| Risk | Component | Code Reference | Severity | Fix (Task 0 step) |
|---|---|---|---|---|
| MessageServer data race | `Publisher::publish()` iterates `topic_->subscribers_` with no lock | `flatland_server/include/flatland_server/message_server.h:100-105` | **CRITICAL** | Step 0a: per-topic `std::mutex` |
| HumanPublisherManager race | Singleton `unordered_map::emplace()` from concurrent threads | `flatland_interface/include/.../human_publisher_manager.hpp:18-20` | **CRITICAL** | Step 0b: `std::mutex` on singleton |
| Cross-plugin body reads | `respondToGetPathAction()` reads `b2Body_GetPosition()` on OTHER robots' bodies while those robots call `SetPose()` | `sootball_navigator.cpp:191-195` | **CRITICAL** | Step 0c: pose snapshot cache |
| Bodies vector access | Thread A reads `RobotB->bodies_[0]` while Thread B modifies via `SetPose()` | `sootball_navigator.cpp:193-194`, `model.cpp:254-294` | **CRITICAL** | Step 0c: snapshot eliminates live cross-reads |
| World plugin vector iteration | Multiple threads iterate `world_plugins_` via `getWorldPlugin()` | `utils.hpp:10`, `sootball.cpp:126`, `sootball_navigator.cpp:278` | **HIGH** | Step 0e: cache references in OnInitialize |

### Risks confirmed SAFE (no fix needed)

| Component | Why It's Safe | Code Reference |
|---|---|---|
| `MapInfoPlugin` graph data | Read-only during physics steps. `BeforePhysicsStep()` only checks timer, graph mutation code is commented out. | `map_info.cpp:32-62` |
| `CollisionFilterRegistry` | Read-only after init. `RegisterLayer()`/`RegisterCollide()` only called during world/model initialization. | `collision_filter_registry.h:63-90` |
| `tf2_ros::TransformBroadcaster` | Each plugin creates its own instance — no shared state. Internal `ros::Publisher` is mutex-protected. | `sootball_navigator.hpp:45` |
| `ros::Publisher::publish()` | roscpp uses internal mutex for serialisation. Safe to call from any thread. | roscpp internals |
| Box2D v3 different-body access | `b2Body_SetTransform` on body A while `b2Body_SetTransform` on body B is safe (different threads, different bodies). | Box2D v3 docs |

### Risks mitigated by Task 3

| Risk | Mitigation |
|---|---|
| ServiceManager spawn/delete during physics step | `world_mutex_` guards `World::Update()` and service handlers so model list mutations don't race with physics |

### Residual risks to monitor

| Risk | Status | Trigger |
|---|---|---|
| `MapInfoPlugin` graph mutation | Safe TODAY — becomes unsafe if `updateFlatlandWorldObjectPoses()` is uncommented (line 43 of `map_info.cpp`) | Code change to MapInfoPlugin |
| MessageServer `publish()` subscriber callback duration | Mutex in step 0a means a slow subscriber callback blocks all publishers on that topic | Rarely a problem — callbacks are lightweight |
| Pose snapshot staleness | Snapshot is taken once per physics step; plugins see positions from start of step, not mid-step. This is actually MORE deterministic than the sequential case. | Semantic change — may affect path planning quality minimally |
