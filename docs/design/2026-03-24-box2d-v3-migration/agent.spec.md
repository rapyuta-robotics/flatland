# Box2D v3 Migration — Agent-Facing Spec

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

## Repo & Branch

- **Repo:** `C:\Users\Rapyuta Robotics\Desktop\flatland` (rapyuta-robotics/flatland fork)
- **Branch to create:** `feature/box2d-v3`
- **Box2D v3 version:** `v3.1.0` from `https://github.com/erincatto/box2d`
- **enkiTS version:** `v2.10` from `https://github.com/dougbinks/enkiTS`

## Build System

- ROS1 Catkin workspace
- `flatland_server/CMakeLists.txt` — main package, builds `flatland_server` node and library
- `flatland_plugins/CMakeLists.txt` — builds plugin shared libraries
- Box2D 2.3.2 is vendored at `flatland_server/thirdparty/Box2D/` and added via `add_subdirectory` as target `flatland_Box2D`
- ThreadPool header at `flatland_plugins/thirdparty/ThreadPool.h` — used only in `laser.h`, to be removed

## File Inventory (all files that need changes)

### Layer 0 — CMake
- `flatland_server/CMakeLists.txt` — remove Box2D 2.x, add Box2D v3 + enkiTS via FetchContent

### Layer 1 — Core Lifecycle
- `flatland_server/include/flatland_server/world.h` — `b2World*`→`b2WorldId`, remove `b2ContactListener` base
- `flatland_server/src/world.cpp` — `new b2World`→`b2CreateWorld`, `Step`→`b2World_Step`
- `flatland_server/include/flatland_server/body.h` — `b2Body*`→`b2BodyId`
- `flatland_server/src/body.cpp` — `CreateBody`→`b2CreateBody`, `SetUserData`, `DestroyBody`
- `flatland_server/include/flatland_server/joint.h` — `b2Joint*`→`b2JointId`
- `flatland_server/src/joint.cpp` — `CreateJoint`→`b2CreateRevoluteJoint`/`b2CreateWeldJoint`, `frequencyHz`→`stiffness`
- `flatland_server/include/flatland_server/layer.h` — `b2World*`→`b2WorldId`
- `flatland_server/src/layer.cpp` — body/fixture creation, `b2EdgeShape::Set`→`b2Segment`
- `flatland_server/src/model_body.cpp` — `b2FixtureDef`→`b2ShapeDef`, shape structs
- `flatland_server/include/flatland_server/model_body.h` — `b2FixtureDef`→`b2ShapeDef`
- `flatland_server/include/flatland_server/entity.h` — may store `b2World*`, update to `b2WorldId`

### Layer 2 — Contact System
- `flatland_server/src/world.cpp` — poll `b2World_GetContactEvents()` after Step
- `flatland_server/include/flatland_server/flatland_plugin.h` — callback signatures `(b2Contact*)` → `(b2ShapeId, b2ShapeId)`
- `flatland_server/src/plugin_manager.cpp` — dispatch with new signatures
- `flatland_server/include/flatland_server/plugin_manager.h` — same
- `flatland_server/include/flatland_server/model_plugin.h` — same
- `flatland_server/src/model_plugin.cpp` — `fixture->GetBody()->GetUserData()` → `b2Shape_GetBody` + `b2Body_GetUserData`
- `flatland_plugins/src/bumper.cpp` — rewrite BeginContact/EndContact/PostSolve
- `flatland_plugins/include/flatland_plugins/bumper.h` — signature updates
- `flatland_plugins/src/bool_sensor.cpp` — rewrite BeginContact/EndContact

### Layer 3 — Ray Casts
- `flatland_plugins/include/flatland_plugins/laser.h` — delete `LaserCallback` class, `ThreadPool pool_`
- `flatland_plugins/src/laser.cpp` — replace parallel pool with sequential `b2World_CastRay` + free function
- `flatland_plugins/include/flatland_plugins/world_modifier.h` — delete `RayTrace` class
- `flatland_plugins/src/world_modifier.cpp` — replace with free function

### Layer 4 — Remaining Plugins
- `flatland_plugins/src/diff_drive.cpp` — body accessor API updates
- `flatland_plugins/src/tricycle_drive.cpp` — body accessors + revolute joint accessor
- `flatland_plugins/src/gps.cpp` — `GetTransform`→`b2Body_GetTransform`
- `flatland_plugins/src/model_tf_publisher.cpp` — same
- `flatland_plugins/src/tween.cpp` — `SetTransform`→`b2Body_SetTransform`
- `flatland_server/src/debug_visualization.cpp` — shape type enum + field accessor updates
- `flatland_plugins/src/world_random_wall.cpp` — `b2MulT`→`b2InvTransformPoint`

### Layer 5 — Threading
- `flatland_server/src/world.cpp` — add enkiTS scheduler, wire `b2WorldDef` task callbacks
- `flatland_server/include/flatland_server/world.h` — add `enkiTaskScheduler*` member

## Critical v2→v3 API Mapping

### World
| v2 | v3 |
|---|---|
| `new b2World(gravity)` | `b2CreateWorld(&b2DefaultWorldDef())` |
| `delete physics_world_` | `b2DestroyWorld(worldId_)` |
| `physics_world_->Step(dt, vIter, pIter)` | `b2World_Step(worldId_, dt, subSteps)` (note: `position_iterations` deprecated — use `subSteps=4`) |
| `physics_world_->SetContactListener(this)` | removed — poll events instead |
| `physics_world_->RayCast(&cb, p1, p2)` | `b2World_CastRay(worldId_, origin, translation, filter, fcn, ctx)` |

### Body
| v2 | v3 |
|---|---|
| `b2Body* body = world->CreateBody(&def)` | `b2BodyId bodyId = b2CreateBody(worldId, &def)` |
| `body->SetUserData(ptr)` | `b2Body_SetUserData(bodyId, ptr)` |
| `body->GetUserData()` | `b2Body_GetUserData(bodyId)` |
| `world->DestroyBody(body)` | `b2DestroyBody(bodyId)` |
| `body->CreateFixture(&fixtureDef)` | `b2CreatePolygonShape(bodyId, &shapeDef, &polygon)` etc. |
| `body->GetPosition()` | `b2Body_GetPosition(bodyId)` |
| `body->GetAngle()` | `b2Body_GetRotation(bodyId)` → `.angle` or `b2Rot_GetAngle(rot)` |
| `body->GetTransform()` | `b2Body_GetTransform(bodyId)` |
| `body->SetTransform(pos, angle)` | `b2Body_SetTransform(bodyId, pos, b2MakeRot(angle))` |
| `body->GetLinearVelocity()` | `b2Body_GetLinearVelocity(bodyId)` |
| `body->SetLinearVelocity(v)` | `b2Body_SetLinearVelocity(bodyId, v)` |
| `body->GetAngularVelocity()` | `b2Body_GetAngularVelocity(bodyId)` |
| `body->SetAngularVelocity(w)` | `b2Body_SetAngularVelocity(bodyId, w)` |
| `body->GetWorldCenter()` | `b2Body_GetWorldCenterOfMass(bodyId)` |
| `body->GetWorldVector(v)` | `b2Body_GetWorldVector(bodyId, v)` |
| `body->GetWorldPoint(p)` | `b2Body_GetWorldPoint(bodyId, p)` |
| `body->GetLinearVelocityFromLocalPoint(p)` | `b2Body_GetLinearVelocityFromLocalPoint(bodyId, p)` (check v3 API — may need manual: vel + cross(omega, r)) |

### Fixture → Shape
| v2 | v3 |
|---|---|
| `b2FixtureDef` | `b2ShapeDef` |
| `fixtureDef.filter.categoryBits` | `shapeDef.filter.categoryBits` |
| `fixtureDef.isSensor` | `shapeDef.isSensor` |
| `fixtureDef.friction` | `shapeDef.friction` |
| `fixtureDef.restitution` | `shapeDef.restitution` |
| `b2CircleShape shape; shape.m_p = center; shape.m_radius = r;` | `b2Circle circle = {center, r};` then `b2CreateCircleShape(bodyId, &shapeDef, &circle)` |
| `b2PolygonShape shape; shape.Set(pts, n);` | `b2Polygon polygon = b2MakePolygon(&hull, 0);` then `b2CreatePolygonShape(bodyId, &shapeDef, &polygon)` |
| `b2EdgeShape edge; edge.Set(v1, v2);` | `b2Segment seg = {v1, v2};` then `b2CreateSegmentShape(bodyId, &shapeDef, &seg)` |
| `fixture->GetFilterData().categoryBits` | `b2Shape_GetFilter(shapeId).categoryBits` |
| `fixture->IsSensor()` | `b2Shape_IsSensor(shapeId)` |
| `fixture->GetBody()` | `b2Shape_GetBody(shapeId)` |
| `fixture->GetNext()` | `b2Body_GetShapes(bodyId, shapes, capacity)` (batch query) |
| `fixture->GetShape()->GetType()` | `b2Shape_GetType(shapeId)` |

### Joint
| v2 | v3 |
|---|---|
| `b2RevoluteJointDef def; def.Initialize(bA,bB,anchor);` | `b2RevoluteJointDef def = b2DefaultRevoluteJointDef();` then set `bodyIdA/B`, `localAnchorA/B` |
| `world->CreateJoint(&def)` → `b2Joint*` | `b2CreateRevoluteJoint(worldId, &def)` → `b2JointId` |
| `b2WeldJointDef.frequencyHz` | `b2WeldJointDef.angularHertz` (or `linearHertz`) |
| `b2WeldJointDef.dampingRatio` | `b2WeldJointDef.angularDampingRatio` |
| `dynamic_cast<b2RevoluteJoint*>(joint)` | Direct `b2JointId` typed calls `b2RevoluteJoint_GetAngle(jId)` |
| `joint->SetMotorSpeed(s)` | `b2RevoluteJoint_SetMotorSpeed(jId, s)` |
| `joint->GetBodyA()` | `b2Joint_GetBodyA(jId)` |
| `joint->GetAnchorA()` | `b2Joint_GetConstraintForce(jId, dt)` etc. / `b2Body_GetWorldPoint(b2Joint_GetBodyA(jId), localAnchor)` |

### Contact Events (v3 only — replaces b2ContactListener)
```cpp
// After b2World_Step():
b2ContactEvents events = b2World_GetContactEvents(worldId_);
for (int i = 0; i < events.beginCount; i++) {
    b2ContactBeginTouchEvent* e = &events.beginEvents[i];
    // e->shapeIdA, e->shapeIdB
    // To get Body*: static_cast<Body*>(b2Body_GetUserData(b2Shape_GetBody(e->shapeIdA)))
}
for (int i = 0; i < events.endCount; i++) {
    b2ContactEndTouchEvent* e = &events.endEvents[i];
}
// Hit events replace PostSolve:
b2ContactHitEvent* hitEvents = events.hitEvents;  // events.hitCount
```

### Ray Cast (v3)
```cpp
// Free function signature:
float LaserRayCastFcn(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal,
                       float fraction, void* context) {
    auto* ctx = static_cast<LaserRayContext*>(context);
    // check filter: b2Shape_GetFilter(shapeId).categoryBits
    if (b2Shape_IsSensor(shapeId)) return -1.0f;  // ignore sensors
    ctx->hit = true;
    ctx->fraction = fraction;
    ctx->point = point;
    ctx->normal = normal;
    return fraction;  // return fraction to find closest hit
}
// Dispatch:
b2QueryFilter filter = b2DefaultQueryFilter();
filter.maskBits = laser_layers_mask;
b2World_CastRay(worldId_, origin, translation, filter, LaserRayCastFcn, &context);
```

### Transform
| v2 | v3 |
|---|---|
| `b2Transform t; t.p.x; t.p.y; t.q.c; t.q.s;` | `b2Transform t; t.p.x; t.p.y; t.q.c; t.q.s;` (same struct layout) |
| `b2MulT(transform, point)` | `b2InvTransformPoint(transform, point)` |

### Math
| v2 | v3 |
|---|---|
| `b2Vec2` | `b2Vec2` (unchanged) |
| `b2_staticBody`, `b2_dynamicBody`, `b2_kinematicBody` | `b2_staticBody`, `b2_dynamicBody`, `b2_kinematicBody` (same names in v3) |
| `b2_maxPolygonVertices` | `B2_MAX_POLYGON_VERTICES` (renamed) |

## Test Files
- `flatland_server/test/` — gtest suite, run with `catkin run_tests flatland_server`
- `flatland_plugins/test/` — gtest suite, run with `catkin run_tests flatland_plugins`

## Includes
- v2: `#include <Box2D/Box2D.h>`
- v3: `#include <box2d/box2d.h>`
