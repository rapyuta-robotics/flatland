# Box2D v3 Migration Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Replace Box2D 2.3.2 with Box2D v3.1.0 in the rapyuta-robotics/flatland fork, wire in enkiTS for multi-core solver, preserving all existing behavior.

**Architecture:** Incremental layer-by-layer on `feature/box2d-v3` branch. Each layer is independently compilable. Six layers: CMake → Core lifecycle → Contact system → Ray casts → Plugins → Threading.

**Tech Stack:** C++17, ROS1 Catkin, Box2D v3.1.0 (C17 API), enkiTS v2.10, CMake FetchContent.

**Reference:** Read `docs/design/2026-03-24-box2d-v3-migration/agent.spec.md` for complete API mapping before starting any layer.

---

## Task 0: Create feature branch

**Files:** none

**Step 1: Create and checkout branch**
```bash
cd C:\Users\Rapyuta Robotics\Desktop\flatland
git checkout -b feature/box2d-v3
```

**Step 2: Verify**
```bash
git branch --show-current
```
Expected: `feature/box2d-v3`

**Step 3: Commit**
```bash
git commit --allow-empty -m "chore: start box2d-v3 migration branch"
```

---

## Task 1: Layer 0 — CMake — Swap Box2D 2.x for v3 + enkiTS

**Files:**
- Modify: `flatland_server/CMakeLists.txt`

**Step 1: Read the current CMakeLists.txt**

Read `flatland_server/CMakeLists.txt` fully to understand current structure before editing.

**Step 2: Replace Box2D vendored build with FetchContent for v3 + enkiTS**

Find the block `add_subdirectory("thirdparty/Box2D")` and the `flatland_Box2D` target references. Replace with:

```cmake
include(FetchContent)

FetchContent_Declare(
  box2d
  GIT_REPOSITORY https://github.com/erincatto/box2d.git
  GIT_TAG        v3.1.0
)
FetchContent_MakeAvailable(box2d)

FetchContent_Declare(
  enkiTS
  GIT_REPOSITORY https://github.com/dougbinks/enkiTS.git
  GIT_TAG        v2.10
)
FetchContent_MakeAvailable(enkiTS)
```

Replace all `flatland_Box2D` → `box2d` in `target_link_libraries`.
Add `enkiTS` to `target_link_libraries` for `flatland_server`.

**Step 3: Remove thirdparty Box2D directory reference**

Remove or comment out the `add_subdirectory("thirdparty/Box2D")` line.

**Step 4: Attempt build — it WILL fail (all #includes are still Box2D v2)**
```bash
cd <catkin_ws>
catkin build flatland_server --no-deps 2>&1 | head -50
```
Expected: compile errors about `Box2D/Box2D.h` not found — this is correct at this stage. CMake configure step should succeed.

**Step 5: Commit the CMake change**
```bash
git add flatland_server/CMakeLists.txt
git commit -m "build(layer0): replace vendored Box2D 2.x with Box2D v3.1.0 + enkiTS via FetchContent"
```

---

## Task 2: Layer 1a — Update all #includes

**Files:**
- All `.cpp` and `.h` files in `flatland_server/` and `flatland_plugins/` that `#include <Box2D/Box2D.h>`

**Step 1: Find all files with the old include**
```bash
grep -r "#include <Box2D/Box2D.h>" flatland_server/ flatland_plugins/ --include="*.h" --include="*.cpp" -l
```

**Step 2: Batch replace in every file found**

Replace `#include <Box2D/Box2D.h>` → `#include <box2d/box2d.h>` in all matching files.

Also remove `#include <thirdparty/ThreadPool.h>` from `flatland_plugins/include/flatland_plugins/laser.h` (ThreadPool is being deleted in Layer 3).

**Step 3: Commit**
```bash
git add -A
git commit -m "refactor(layer1): update Box2D include paths to v3"
```

---

## Task 3: Layer 1b — Migrate world.h + entity.h headers

**Files:**
- Modify: `flatland_server/include/flatland_server/world.h`
- Modify: `flatland_server/include/flatland_server/entity.h`

**Step 1: Read both files in full**

**Step 2: Update world.h**
- Remove `public b2ContactListener` from `World` class declaration
- Change `b2World* physics_world_` → `b2WorldId worldId_`
- Change `b2Vec2 gravity_` — `b2Vec2` is still valid in v3, keep
- Remove declarations: `BeginContact`, `EndContact`, `PreSolve`, `PostSolve`
- Add member: `enkiTaskScheduler* taskScheduler_ = nullptr;`

**Step 3: Update entity.h**
- If `b2World*` is stored, change to `b2WorldId`

**Step 4: Attempt header compile check**
```bash
catkin build flatland_server --no-deps 2>&1 | grep "world.h\|entity.h" | head -20
```

**Step 5: Commit**
```bash
git add flatland_server/include/flatland_server/world.h flatland_server/include/flatland_server/entity.h
git commit -m "refactor(layer1): update world.h and entity.h to Box2D v3 types"
```

---

## Task 4: Layer 1c — Migrate body.h + body.cpp

**Files:**
- Modify: `flatland_server/include/flatland_server/body.h`
- Modify: `flatland_server/src/body.cpp`

**Step 1: Read both files in full**

**Step 2: Update body.h**
- `b2Body* physics_body_` → `b2BodyId physics_body_`
- Any `b2World*` parameter → `b2WorldId`
- Any `b2BodyType` field — check if still used (v3 keeps `b2BodyType` enum with same names)

**Step 3: Update body.cpp** using the API mapping in agent.spec.md:
- `world->CreateBody(&body_def)` → `b2CreateBody(worldId, &body_def)`
  - `b2BodyDef` fields are mostly the same in v3; verify `type`, `position`, `angle`, `linearVelocity`, `angularVelocity`, `linearDamping`, `angularDamping`
- `physics_body_->SetUserData(this)` → `b2Body_SetUserData(physics_body_, this)`
- `physics_body_->GetWorld()->DestroyBody(physics_body_)` → `b2DestroyBody(physics_body_)`
- `GetPosition()` → `b2Body_GetPosition(physics_body_)`
- `GetAngle()` → `b2Rot_GetAngle(b2Body_GetRotation(physics_body_))`
- `GetType()` → `b2Body_GetType(physics_body_)`
- `GetLinearDamping()` / `GetAngularDamping()` → `b2Body_GetLinearDamping` / `b2Body_GetAngularDamping`
- `GetFixtureList()` — replace with `b2Body_GetShapeCount` + `b2Body_GetShapes`

**Step 4: Attempt compile**
```bash
catkin build flatland_server --no-deps 2>&1 | grep "body\." | head -30
```

**Step 5: Commit**
```bash
git add flatland_server/include/flatland_server/body.h flatland_server/src/body.cpp
git commit -m "refactor(layer1): migrate body.h/cpp to Box2D v3 ID API"
```

---

## Task 5: Layer 1d — Migrate joint.h + joint.cpp

**Files:**
- Modify: `flatland_server/include/flatland_server/joint.h`
- Modify: `flatland_server/src/joint.cpp`

**Step 1: Read both files**

**Step 2: Update joint.h**
- `b2Joint* physics_joint_` → `b2JointId physics_joint_`
- `b2JointDef` stored? Replace with appropriate typed def or remove

**Step 3: Update joint.cpp**
- `world->CreateJoint(&def)` for revolute → `b2CreateRevoluteJoint(worldId, &revoluteDef)`
- `world->CreateJoint(&def)` for weld → `b2CreateWeldJoint(worldId, &weldDef)`
- `physics_joint_->SetUserData(this)` → `b2Joint_SetUserData(physics_joint_, this)` (check v3 API — may be `b2RevoluteJoint_SetUserData` or body user data)
- `b2WeldJointDef.frequencyHz` → `angularHertz`
- `b2WeldJointDef.dampingRatio` → `angularDampingRatio`
- `world->DestroyJoint(physics_joint_)` → `b2DestroyJoint(physics_joint_)`
- `GetBodyA()` / `GetBodyB()` → `b2Joint_GetBodyA(jId)` / `b2Joint_GetBodyB(jId)`
- `GetAnchorA()` / `GetAnchorB()` — compute via `b2Body_GetWorldPoint`

**Step 4: Compile check**
```bash
catkin build flatland_server --no-deps 2>&1 | grep "joint\." | head -30
```

**Step 5: Commit**
```bash
git add flatland_server/include/flatland_server/joint.h flatland_server/src/joint.cpp
git commit -m "refactor(layer1): migrate joint.h/cpp to Box2D v3 ID API"
```

---

## Task 6: Layer 1e — Migrate layer.h + layer.cpp

**Files:**
- Modify: `flatland_server/include/flatland_server/layer.h`
- Modify: `flatland_server/src/layer.cpp`

**Step 1: Read both files**

**Step 2: Update layer.h**
- `b2World*` → `b2WorldId`

**Step 3: Update layer.cpp**
- Static body creation: `b2BodyDef` → use `b2DefaultBodyDef()`, `type = b2_staticBody`
- `b2EdgeShape edge; edge.Set(v1, v2); b2FixtureDef fd; body->CreateFixture(&fd)` →
  ```cpp
  b2ShapeDef shapeDef = b2DefaultShapeDef();
  shapeDef.filter = layerFilter;
  b2Segment seg = {v1, v2};
  b2CreateSegmentShape(bodyId, &shapeDef, &seg);
  ```
- Update all `b2FixtureDef` usages similarly

**Step 4: Compile and fix**
```bash
catkin build flatland_server --no-deps 2>&1 | grep "layer\." | head -30
```

**Step 5: Commit**
```bash
git add flatland_server/include/flatland_server/layer.h flatland_server/src/layer.cpp
git commit -m "refactor(layer1): migrate layer.h/cpp to Box2D v3 shape API"
```

---

## Task 7: Layer 1f — Migrate model_body.h + model_body.cpp

**Files:**
- Modify: `flatland_server/include/flatland_server/model_body.h`
- Modify: `flatland_server/src/model_body.cpp`

**Step 1: Read both files**

**Step 2: Update model_body.h**
- `b2FixtureDef` stored → `b2ShapeDef`

**Step 3: Update model_body.cpp**
- Circle: `b2CircleShape shape; shape.m_p.Set(x,y); shape.m_radius = r;` →
  ```cpp
  b2Circle circle;
  circle.center = {x, y};
  circle.radius = r;
  b2CreateCircleShape(bodyId, &shapeDef, &circle);
  ```
- Polygon: `b2PolygonShape shape; shape.Set(pts, n);` →
  ```cpp
  b2Hull hull = b2ComputeHull(pts, n);
  b2Polygon polygon = b2MakePolygon(&hull, 0.0f);
  b2CreatePolygonShape(bodyId, &shapeDef, &polygon);
  ```
- `b2_maxPolygonVertices` → `B2_MAX_POLYGON_VERTICES`

**Step 4: Compile and fix**
```bash
catkin build flatland_server --no-deps 2>&1 | grep "model_body\." | head -30
```

**Step 5: Commit**
```bash
git add flatland_server/include/flatland_server/model_body.h flatland_server/src/model_body.cpp
git commit -m "refactor(layer1): migrate model_body to Box2D v3 shape API"
```

---

## Task 8: Layer 1g — Migrate world.cpp core lifecycle

**Files:**
- Modify: `flatland_server/src/world.cpp`

**Step 1: Read world.cpp in full**

**Step 2: Replace world creation**
```cpp
// Remove:
physics_world_ = new b2World(gravity_);
physics_world_->SetContactListener(this);

// Add:
b2WorldDef worldDef = b2DefaultWorldDef();
worldDef.gravity = gravity_;
worldId_ = b2CreateWorld(&worldDef);
```

**Step 3: Replace world destruction**
```cpp
// Remove:
physics_world_->SetContactListener(nullptr);
delete physics_world_;

// Add:
b2DestroyWorld(worldId_);
```

**Step 4: Replace Step call**
```cpp
// Remove:
physics_world_->Step(timekeeper_.GetStepSize(),
                     physics_velocity_iterations_,
                     physics_position_iterations_);

// Add (subSteps replaces position_iterations — use 4 as default):
b2World_Step(worldId_,
             static_cast<float>(timekeeper_.GetStepSize()),
             physics_velocity_iterations_);  // velocity iterations still used
```

**Step 5: Remove BeginContact/EndContact/PreSolve/PostSolve implementations**

These 4 method bodies are deleted from world.cpp. The contact dispatch moves to Layer 2.

**Step 6: Compile Layer 1 fully**
```bash
catkin build flatland_server --no-deps 2>&1 | tail -30
```
Expected: `flatland_server` builds cleanly (plugins still broken — that's fine).

**Step 7: Run existing server tests**
```bash
catkin run_tests flatland_server --no-deps
catkin_test_results build/flatland_server
```
Expected: some failures due to contact/fixture access in tests — note which, fix in subsequent tasks.

**Step 8: Commit**
```bash
git add flatland_server/src/world.cpp
git commit -m "refactor(layer1): migrate world.cpp core lifecycle to Box2D v3"
```

---

## Task 9: Layer 2a — Update plugin contact callback signatures

**Files:**
- Modify: `flatland_server/include/flatland_server/flatland_plugin.h`
- Modify: `flatland_server/include/flatland_server/plugin_manager.h`
- Modify: `flatland_server/include/flatland_server/model_plugin.h`

**Step 1: Read all three files**

**Step 2: Update flatland_plugin.h**

Change contact callback virtual method signatures:
```cpp
// Remove:
virtual void BeginContact(b2Contact* contact) {}
virtual void EndContact(b2Contact* contact) {}
virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {}
virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {}

// Add:
virtual void BeginContact(b2ShapeId shapeIdA, b2ShapeId shapeIdB) {}
virtual void EndContact(b2ShapeId shapeIdA, b2ShapeId shapeIdB) {}
// PreSolve removed — no equivalent in v3
// PostSolve replaced by hit events:
virtual void OnContactHit(b2ShapeId shapeIdA, b2ShapeId shapeIdB,
                           b2Vec2 point, b2Vec2 normal, float approachSpeed) {}
```

**Step 3: Update plugin_manager.h** — same signature changes for dispatch methods.

**Step 4: Update model_plugin.h** — same signature changes.

**Step 5: Commit headers**
```bash
git add flatland_server/include/flatland_server/flatland_plugin.h \
        flatland_server/include/flatland_server/plugin_manager.h \
        flatland_server/include/flatland_server/model_plugin.h
git commit -m "refactor(layer2): update contact callback signatures for Box2D v3"
```

---

## Task 10: Layer 2b — Migrate plugin_manager.cpp + model_plugin.cpp

**Files:**
- Modify: `flatland_server/src/plugin_manager.cpp`
- Modify: `flatland_server/src/model_plugin.cpp`

**Step 1: Read both files**

**Step 2: Update plugin_manager.cpp**

Dispatch methods now take `(b2ShapeId, b2ShapeId)` — update body of each method to call plugins with new signature.

**Step 3: Update model_plugin.cpp**

The fixture→body→Body* pattern:
```cpp
// Remove:
Body* bodyA = static_cast<Body*>(contact->GetFixtureA()->GetBody()->GetUserData());

// Add:
Body* bodyA = static_cast<Body*>(b2Body_GetUserData(b2Shape_GetBody(shapeIdA)));
```

**Step 4: Compile check**
```bash
catkin build flatland_server --no-deps 2>&1 | tail -30
```

**Step 5: Commit**
```bash
git add flatland_server/src/plugin_manager.cpp flatland_server/src/model_plugin.cpp
git commit -m "refactor(layer2): migrate plugin_manager and model_plugin to v3 contact API"
```

---

## Task 11: Layer 2c — Implement contact event polling in world.cpp

**Files:**
- Modify: `flatland_server/src/world.cpp`

**Step 1: Read world.cpp Update() / step function**

**Step 2: Add event polling after b2World_Step**

```cpp
void World::Update(Timekeeper& timekeeper) {
    // ... existing pre-step logic ...

    b2World_Step(worldId_, static_cast<float>(timekeeper.GetStepSize()),
                 physics_velocity_iterations_);

    // --- Contact event dispatch (replaces b2ContactListener callbacks) ---
    b2ContactEvents events = b2World_GetContactEvents(worldId_);

    for (int i = 0; i < events.beginCount; i++) {
        const b2ContactBeginTouchEvent& e = events.beginEvents[i];
        plugin_manager_.BeginContact(e.shapeIdA, e.shapeIdB);
    }
    for (int i = 0; i < events.endCount; i++) {
        const b2ContactEndTouchEvent& e = events.endEvents[i];
        plugin_manager_.EndContact(e.shapeIdA, e.shapeIdB);
    }
    for (int i = 0; i < events.hitCount; i++) {
        const b2ContactHitEvent& e = events.hitEvents[i];
        plugin_manager_.OnContactHit(e.shapeIdA, e.shapeIdB,
                                      e.point, e.normal, e.approachSpeed);
    }
    // --- End contact dispatch ---

    // ... existing post-step logic ...
}
```

**Step 3: Compile and run tests**
```bash
catkin build flatland_server --no-deps
catkin run_tests flatland_server --no-deps
catkin_test_results build/flatland_server
```
Expected: contact-related tests pass.

**Step 4: Commit**
```bash
git add flatland_server/src/world.cpp
git commit -m "feat(layer2): implement Box2D v3 contact event polling in World::Update"
```

---

## Task 12: Layer 2d — Migrate bumper.cpp + bool_sensor.cpp

**Files:**
- Modify: `flatland_plugins/src/bumper.cpp`
- Modify: `flatland_plugins/include/flatland_plugins/bumper.h`
- Modify: `flatland_plugins/src/bool_sensor.cpp`

**Step 1: Read all three files in full**

**Step 2: Update bumper.h + bumper.cpp**
- Change `BeginContact(b2Contact*)` → `BeginContact(b2ShapeId, b2ShapeId)`
- Change `EndContact(b2Contact*)` → `EndContact(b2ShapeId, b2ShapeId)`
- Change `PostSolve(b2Contact*, b2ContactImpulse*)` → `OnContactHit(..., float approachSpeed)`
- Replace `contact->GetFixtureA()` → use `shapeIdA` directly
- Replace `fixture->GetBody()->GetUserData()` → `b2Body_GetUserData(b2Shape_GetBody(shapeIdA))`
- Replace `b2WorldManifold` usage → use `point` and `normal` from `OnContactHit` parameters
- Replace impulse access (`impulse->normalImpulses[0]`) → use `approachSpeed` from hit event

**Step 3: Update bool_sensor.cpp**
- Same signature updates for `BeginContact` / `EndContact`
- `contact->GetFixtureA()->IsSensor()` → `b2Shape_IsSensor(shapeIdA)`
- `contact->GetFixtureA()` → `shapeIdA`

**Step 4: Build plugins**
```bash
catkin build flatland_plugins --no-deps 2>&1 | grep "bumper\|bool_sensor" | head -30
```

**Step 5: Run plugin tests**
```bash
catkin run_tests flatland_plugins --no-deps
catkin_test_results build/flatland_plugins
```

**Step 6: Commit**
```bash
git add flatland_plugins/src/bumper.cpp flatland_plugins/include/flatland_plugins/bumper.h \
        flatland_plugins/src/bool_sensor.cpp
git commit -m "refactor(layer2): migrate bumper and bool_sensor to Box2D v3 contact events"
```

---

## Task 13: Layer 3a — Migrate laser.cpp (ray casts)

**Files:**
- Modify: `flatland_plugins/include/flatland_plugins/laser.h`
- Modify: `flatland_plugins/src/laser.cpp`

**Step 1: Read both files in full**

**Step 2: Update laser.h**
- Delete `LaserCallback` nested class entirely
- Delete `ThreadPool pool_` member
- Delete `#include <thirdparty/ThreadPool.h>`
- Add struct for ray cast context:
  ```cpp
  struct LaserRayContext {
      bool hit = false;
      float fraction = 1.0f;
      b2Vec2 point = {0, 0};
      b2Vec2 normal = {0, 0};
  };
  ```

**Step 3: Update laser.cpp**

Delete the thread pool parallel loop. Replace it with a sequential loop:

```cpp
// Old parallel pattern (DELETE):
std::vector<std::future<LaserData>> results(num_beams);
for (int i = 0; i < num_beams; i++) {
    results[i] = pool_.enqueue([i, this, ...] {
        LaserCallback cb(...);
        GetModel()->GetPhysicsWorld()->RayCast(&cb, p1, p2);
        ...
    });
}

// New sequential pattern:
for (int i = 0; i < num_beams; i++) {
    float angle = angle_min + i * angle_increment;
    b2Vec2 origin = ...; // compute from body transform
    b2Vec2 end    = ...; // origin + range * direction

    LaserRayContext ctx;
    b2QueryFilter filter = b2DefaultQueryFilter();
    filter.maskBits = laser_layers_bits_;
    b2Vec2 translation = {end.x - origin.x, end.y - origin.y};
    b2World_CastRay(worldId_, origin, translation, filter, LaserRayCastFcn, &ctx);

    scan_msg.ranges[i] = ctx.hit ? ctx.fraction * range_max_ : range_max_;
    // intensities if needed
}
```

Add free function (outside class, before the method):
```cpp
static float LaserRayCastFcn(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal,
                               float fraction, void* context) {
    // Filter sensors
    if (b2Shape_IsSensor(shapeId)) return -1.0f;
    auto* ctx = static_cast<Laser::LaserRayContext*>(context);
    ctx->hit = true;
    ctx->fraction = fraction;
    ctx->point = point;
    ctx->normal = normal;
    return fraction;  // return fraction to continue looking for closer hits
}
```

**Step 4: Update how worldId is accessed**

The `Laser` plugin needs access to `b2WorldId`. Currently it calls `GetModel()->GetPhysicsWorld()` returning `b2World*`. Update `model.h` / `world.h` to expose `b2WorldId GetPhysicsWorldId()` accessor.

**Step 5: Build and test laser**
```bash
catkin build flatland_plugins --no-deps 2>&1 | grep "laser" | head -30
catkin run_tests flatland_plugins --no-deps -- --gtest_filter="*Laser*"
catkin_test_results build/flatland_plugins
```

**Step 6: Commit**
```bash
git add flatland_plugins/include/flatland_plugins/laser.h flatland_plugins/src/laser.cpp
git commit -m "refactor(layer3): replace parallel ThreadPool ray casts with sequential b2World_CastRay"
```

---

## Task 14: Layer 3b — Migrate world_modifier.cpp (ray casts)

**Files:**
- Modify: `flatland_plugins/include/flatland_plugins/world_modifier.h`
- Modify: `flatland_plugins/src/world_modifier.cpp`

**Step 1: Read both files**

**Step 2: Delete `RayTrace` callback class from world_modifier.h**

Add context struct if needed.

**Step 3: Replace RayCast call in world_modifier.cpp with free function pattern**

Same pattern as laser — create a `WorldModifierRayCastFcn` free function, use `b2World_CastRay`.

**Step 4: Build and commit**
```bash
catkin build flatland_plugins --no-deps 2>&1 | grep "world_modifier" | head -20
git add flatland_plugins/include/flatland_plugins/world_modifier.h \
        flatland_plugins/src/world_modifier.cpp
git commit -m "refactor(layer3): migrate world_modifier ray cast to Box2D v3"
```

---

## Task 15: Layer 4 — Migrate remaining plugins

**Files:**
- Modify: `flatland_plugins/src/diff_drive.cpp`
- Modify: `flatland_plugins/src/tricycle_drive.cpp`
- Modify: `flatland_plugins/src/gps.cpp`
- Modify: `flatland_plugins/src/model_tf_publisher.cpp`
- Modify: `flatland_plugins/src/tween.cpp`
- Modify: `flatland_plugins/src/world_random_wall.cpp`
- Modify: `flatland_server/src/debug_visualization.cpp`

**Step 1: Read all files** (read them in parallel)

**Step 2: Migrate diff_drive.cpp**
Using agent.spec.md body accessor table:
- `GetPosition()` → `b2Body_GetPosition`
- `GetAngle()` → `b2Rot_GetAngle(b2Body_GetRotation(id))`
- `GetWorldVector(v)` → `b2Body_GetWorldVector(id, v)`
- `GetWorldCenter()` → `b2Body_GetWorldCenterOfMass(id)`
- `SetLinearVelocity` → `b2Body_SetLinearVelocity`
- `SetAngularVelocity` → `b2Body_SetAngularVelocity`
- `GetLinearVelocityFromLocalPoint` → manual: `vel + cross(omega, localPt)` or check v3 API

**Step 3: Migrate tricycle_drive.cpp**
Same as diff_drive plus:
- `dynamic_cast<b2RevoluteJoint*>(joint)` → `b2RevoluteJoint_GetAngle(jId)` directly
- `rev_joint->SetMotorSpeed(s)` → `b2RevoluteJoint_SetMotorSpeed(jId, s)`

**Step 4: Migrate gps.cpp, model_tf_publisher.cpp, tween.cpp**
- `GetTransform()` → `b2Body_GetTransform(id)` — returns `b2Transform` (same struct layout as v2)
- `t.q.c`, `t.q.s` — `b2Rot` fields are `c` and `s` in v3 (same)
- `SetTransform(pos, angle)` → `b2Body_SetTransform(id, pos, b2MakeRot(angle))`

**Step 5: Migrate world_random_wall.cpp**
- `b2MulT(transform, point)` → `b2InvTransformPoint(transform, point)`
- Shape/fixture access: `fixture->GetShape()` → use `b2ShapeId` based accessor
- `fixture->GetNext()` → iterate via `b2Body_GetShapes`

**Step 6: Migrate debug_visualization.cpp**
- `b2Shape::e_circle` → `b2_circleShape`
- `b2Shape::e_polygon` → `b2_polygonShape`  
- `b2Shape::e_edge` → `b2_segmentShape`
- `shape->m_count`, `shape->m_vertices[i]` → `b2Shape_GetPolygon(shapeId)` returns `b2Polygon` struct
- `shape->m_p`, `shape->m_radius` → `b2Shape_GetCircle(shapeId)` returns `b2Circle` struct
- `fixture->GetNext()` → batch `b2Body_GetShapes`
- `body->GetFixtureList()` → `b2Body_GetShapes`

**Step 7: Build all plugins**
```bash
catkin build flatland_plugins --no-deps 2>&1 | tail -20
```
Expected: clean build.

**Step 8: Run all plugin tests**
```bash
catkin run_tests flatland_plugins --no-deps
catkin_test_results build/flatland_plugins
```
Expected: 0 failures.

**Step 9: Commit**
```bash
git add flatland_plugins/src/diff_drive.cpp flatland_plugins/src/tricycle_drive.cpp \
        flatland_plugins/src/gps.cpp flatland_plugins/src/model_tf_publisher.cpp \
        flatland_plugins/src/tween.cpp flatland_plugins/src/world_random_wall.cpp \
        flatland_server/src/debug_visualization.cpp
git commit -m "refactor(layer4): migrate all remaining plugins to Box2D v3 API"
```

---

## Task 16: Layer 5 — Wire enkiTS for multi-core solver

**Files:**
- Modify: `flatland_server/include/flatland_server/world.h`
- Modify: `flatland_server/src/world.cpp`
- Modify: `flatland_server/CMakeLists.txt` (verify enkiTS is linked)

**Step 1: Read world.h and world.cpp**

**Step 2: Add enkiTS header include and member in world.h**
```cpp
#include <enkiTS/TaskScheduler.h>

class World : ... {
    ...
private:
    enki::TaskScheduler taskScheduler_;  // owns the thread pool
    ...
};
```

**Step 3: Wire enkiTS task callbacks in world.cpp CreateWorld**

```cpp
// Define task callbacks (file scope static functions):
static void* EnkiEnqueueTask(b2TaskCallback* box2dTask, int itemCount,
                               int minRange, void* box2dTaskContext, void* userContext) {
    auto* scheduler = static_cast<enki::TaskScheduler*>(userContext);
    auto* taskSet = new enki::TaskSet(itemCount,
        [box2dTask, box2dTaskContext](enki::TaskSetPartition range, uint32_t threadNum) {
            box2dTask(range.start, range.end, threadNum, box2dTaskContext);
        });
    taskSet->m_MinRange = minRange;
    scheduler->AddTaskSetToPipe(taskSet);
    return taskSet;
}

static void EnkiFinishTask(void* taskPtr, void* userContext) {
    auto* scheduler = static_cast<enki::TaskScheduler*>(userContext);
    auto* taskSet = static_cast<enki::TaskSet*>(taskPtr);
    scheduler->WaitforTask(taskSet);
    delete taskSet;
}

// In World constructor / MakeWorld():
taskScheduler_.Initialize();  // uses hardware_concurrency() threads

b2WorldDef worldDef = b2DefaultWorldDef();
worldDef.gravity = gravity_;
worldDef.workerCount = taskScheduler_.GetNumTaskThreads();
worldDef.enqueueTask = EnkiEnqueueTask;
worldDef.finishTask  = EnkiFinishTask;
worldDef.userTaskContext = &taskScheduler_;
worldId_ = b2CreateWorld(&worldDef);
```

**Step 4: Destroy scheduler after world**
In destructor / Shutdown:
```cpp
b2DestroyWorld(worldId_);
taskScheduler_.WaitforAllAndShutdown();
```

**Step 5: Build**
```bash
catkin build flatland_server flatland_plugins --no-deps 2>&1 | tail -10
```
Expected: clean build.

**Step 6: Run full test suite**
```bash
catkin run_tests flatland_server flatland_plugins
catkin_test_results build/flatland_server build/flatland_plugins
```
Expected: 0 failures.

**Step 7: Verify multi-core utilization**

Start the sim and monitor:
```bash
roslaunch sootballs_bringup simulation.launch &
# In another terminal:
top -d 1 | grep flatland
# or:
htop  # observe CPU columns
```
Expected: CPU utilization spread across multiple cores during `b2World_Step`.

**Step 8: Final commit**
```bash
git add flatland_server/include/flatland_server/world.h flatland_server/src/world.cpp
git commit -m "feat(layer5): wire enkiTS multi-core task scheduler into Box2D v3 world"
```

---

## Task 17: Final verification

**Step 1: Full build from clean**
```bash
catkin clean flatland_server flatland_plugins
catkin build flatland_server flatland_plugins
```

**Step 2: All tests pass**
```bash
catkin run_tests flatland_server flatland_plugins
catkin_test_results build/flatland_server build/flatland_plugins
```
Expected output: `Summary: X tests, 0 errors, 0 failures`

**Step 3: Integration smoke test with rr_sootballs**
```bash
cd C:\Users\Rapyuta Robotics\Desktop\rr_sootballs
SIM_TYPE=flatland tilt up -- --site demo --num_robots 5
```
Observe: simulation runs, robots navigate, laser scans publish correctly.

**Step 4: Push branch**
```bash
cd C:\Users\Rapyuta Robotics\Desktop\flatland
git log --oneline feature/box2d-v3 | head -20
git push origin feature/box2d-v3
```
