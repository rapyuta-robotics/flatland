# Box2D v3 Migration + Multi-Core Physics

**Date:** 2026-03-24  
**Repo:** rapyuta-robotics/flatland  
**Branch:** `feature/box2d-v3`

## Goal

Replace the vendored Box2D 2.3.2 physics engine with Box2D v3.x and wire in enkiTS as the task scheduler, enabling the Flatland simulator to use all available CPU cores for physics simulation. All existing ROS interfaces, launch parameters, and simulation behavior are preserved.

## Background

Flatland is a 2D robot simulator used in rr_sootballs warehouse simulation. It currently runs on a single CPU core because Box2D 2.x's `b2World::Step()` is single-threaded by design. Box2D v3 (released August 2024) is a complete rewrite in C17 with native multi-threading via a pluggable task-system interface. By wiring enkiTS into the `b2WorldDef` task callbacks, the solver, broad-phase, and island solving all run across all available cores with no changes to simulation logic.

## Non-Goals

- No ROS interface changes (topics, services, parameters)
- No changes in the rr_sootballs repo
- No new simulation features
- No GPU physics

## Architecture

### Threading Model

**Option A (chosen):** Box2D v3 internal solver threading via enkiTS. Box2D v3 exposes `b2WorldDef.enqueueTask` / `b2WorldDef.finishTask` callbacks. We provide an enkiTS scheduler instance and hook these callbacks. Box2D partitions its internal work graph (island solving, broad-phase) and schedules parallel tasks through the provided callbacks. Ray casts remain single-threaded — the existing `ThreadPool`-based ray cast parallelism in `laser.cpp` must be removed as `b2World_CastRay` is not thread-safe to call concurrently from external threads in v3.

### Migration Strategy

**Approach 2 (chosen):** Incremental layer-by-layer on feature branch. Six layers, each independently compilable and testable:

```
Layer 0: CMake — swap Box2D 2.3.2 for v3 + add enkiTS
Layer 1: Core lifecycle — world, body, joint, layer (pointers → IDs)
Layer 2: Contact system — callbacks → event polling
Layer 3: Ray casts — virtual classes → free functions
Layer 4: Remaining plugins
Layer 5: Threading — wire enkiTS into b2WorldDef
```

## Key API Differences

### Pointer → ID System
Box2D v3 replaces all `b2Body*`, `b2Fixture*`, `b2Joint*` raw pointers with opaque integer ID structs (`b2BodyId`, `b2ShapeId`, `b2JointId`). All code that stores, passes, or dereferences these pointers must be updated.

### Contact System
Box2D v3 removes the `b2ContactListener` virtual callback interface entirely. Contacts are collected internally and exposed via `b2World_GetContactEvents()` which must be polled after each `b2World_Step()`. The dispatch chain `World → PluginManager → plugin callbacks` is preserved but the entry point changes from virtual overrides to a polling loop in `World::Update()`.

### Ray Cast System
Box2D v3 replaces `b2RayCastCallback` virtual classes with a C-style function pointer `b2CastResultFcn`. The `LaserCallback` and `RayTrace` virtual classes are deleted and replaced with free functions. The parallel `ThreadPool` in `laser.cpp` is removed.

### User Data
`b2Body::SetUserData(void*)` → `b2Body_SetUserData(b2BodyId, void*)`. All fixture→body→`Body*` casts in `bumper.cpp` and `model_plugin.cpp` must be updated accordingly.

### enkiTS Threading
```cpp
b2WorldDef worldDef = b2DefaultWorldDef();
worldDef.workerCount = std::thread::hardware_concurrency();
worldDef.enqueueTask = [](b2TaskCallback* task, int itemCount, int minRange,
                           void* taskContext, void* userContext) -> void* {
    auto* ts = static_cast<enkiTaskScheduler*>(userContext);
    auto* set = new enkiTaskSet;   // pool-managed in production
    enkiInitTaskSet(ts, set, task, taskContext, itemCount, minRange);
    enkiAddTaskSet(ts, set);
    return set;
};
worldDef.finishTask = [](void* taskPtr, void* userContext) {
    auto* ts = static_cast<enkiTaskScheduler*>(userContext);
    enkiWaitForTaskSet(ts, static_cast<enkiTaskSet*>(taskPtr));
};
worldDef.userTaskContext = enkiTaskSchedulerPtr_;
```

## Acceptance Criteria

1. All existing tests in `flatland_server/test/` and `flatland_plugins/test/` pass with 0 failures
2. `catkin_test_results` clean
3. Running `rr_sootballs` with `SIM_TYPE=flatland` produces no behaviour regressions
4. CPU monitoring during `b2World_Step` shows >1 core utilization on multi-core machine
