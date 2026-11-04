# openarmx_task_executor

This package owns the competition-side sequencing. It does not contain MoveIt
planning code and it does not drive arm joints directly.

The referee-gated three-phase supermarket framework is documented in
[`SUPERMARKET_COMPETITION_FRAMEWORK_CN.md`](SUPERMARKET_COMPETITION_FRAMEWORK_CN.md).
It adds `/competition/execute_phase` above the existing single-item
`/execute_transport` action.

## Runtime flow

`ExecuteTransport` performs:

1. Nav2 `NavigateToPose` to the source station.
2. Optional marker `DockToStation` or scan-based `ShelfFineAlign`.
3. A base-stop hold check using the configured odometry topic.
4. `DetectObject` to obtain a stamped object pose.
5. `PickObject` to let the MoveIt integration plan and execute the grasp.
6. If the source station explicitly enables it, `RetreatFromStation` clears the
   close-station zone before the next Nav2 goal.
7. Nav2 navigation and alignment at the destination station.
8. `PlaceObject` to release the object.
9. If configured for the destination, retreat again before finishing.

The default policy is stop-then-manipulate. The executor rejects concurrent
transport tasks, supports cancellation, reports phase feedback, and retries
navigation/docking only. It never retries a physical grasp automatically.

## Build and run

```bash
cd /home/openarmx/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  openarmx_integrated_moveit_msgs \
  openarmx_integrated_moveit_config \
  openarmx_task_msgs \
  openarmx_task_executor \
  --symlink-install
source install/setup.bash
ros2 launch openarmx_task_executor task_executor.launch.py
```

To start the executor and its baseline docking server through one launch (while
Nav2, perception, and MoveIt remain independently owned), use:

```bash
ros2 launch openarmx_task_executor competition_task_api.launch.py
```

### Shelf-view observation test

After the base has finished Nav2 and shelf fine alignment, the shelf-view
action moves `lift_joint` and one fixed arm posture together through the
MoveIt command bridge. This is a guarded, taught joint trajectory; it does not
invoke MoveIt collision planning. MoveIt planning is used by the subsequent
grasp pre-planning, Cartesian feed, and retreat stages.
Before testing on hardware, put measured `lift_joint` coordinates in the
station file. Do not use the inventory database `world_z` as a motor
coordinate:

```yaml
stations:
  SHELF_TABLE_APPROACH:
    level_lift_positions:
      0: <measured lift_joint>
      1: <measured lift_joint>
      2: <measured lift_joint>
      3: <measured lift_joint>
```

The trajectory bridge and MoveIt planning stack must already be running (the
integrated MoveIt launch starts them by default). Start the task-side
level mapper with the same measured station configuration. This command starts
only the task API and its shelf-view server, so it is suitable when Nav2 and
MoveIt are owned by separate launch files:

```bash
source /opt/ros/humble/setup.bash
source /home/openarmx/openflex_all/openflex_ws/install/setup.bash

ros2 launch openarmx_task_executor competition_task_api.launch.py \
  station_config:=/home/openarmx/openflex_all/openflex_ws/src/openflex_integrated/openarmx_task_executor/config/stations_supermarket_test_07.yaml \
  start_shelf_view_server:=true \
  start_shelf_fine_align_server:=false \
  start_docking_server:=false \
  start_competition_coordinator:=false \
  start_mock_pick_server:=false
```

With the robot physically stopped and fresh `/odom_safe` and `/joint_states`,
send a manual observation goal. `arm: 1` selects the right arm; use `arm: 0`
for the left arm. An empty `joint_names`/`joint_positions` pair selects the
station's calibrated per-level arm posture, and `execute: true` performs the
MoveIt trajectory and verifies the final joints:

```bash
ros2 action send_goal --feedback \
  /competition/move_to_shelf_view \
  openarmx_task_msgs/action/MoveToShelfView \
  "{station_id: SHELF_TABLE_APPROACH, shelf_level: 2, arm: 1, use_explicit_lift_position: false, lift_position: 0.0, joint_names: [], joint_positions: [], velocity_scale: 0.2, acceleration_scale: 0.2, planning_timeout_sec: 5.0, timeout_sec: 30.0, execute: true}"
```

For the first controller-chain check, bypass the YAML lookup with a physically
measured value:

```bash
ros2 action send_goal --feedback \
  /competition/move_to_shelf_view \
  openarmx_task_msgs/action/MoveToShelfView \
  "{station_id: SHELF_TABLE_APPROACH, shelf_level: 2, arm: 1, use_explicit_lift_position: true, lift_position: <measured lift_joint>, joint_names: [], joint_positions: [], velocity_scale: 0.2, acceleration_scale: 0.2, planning_timeout_sec: 5.0, timeout_sec: 30.0, execute: true}"
```

### Full transport with automatic shelf view

For the complete competition path, enable both the task-side server and the
executor hook. After Nav2 reaches the source, docking/fine alignment confirms
the base is stopped, and before `DetectObject`, `ExecuteTransport` automatically
calls `/competition/move_to_shelf_view` using the source slot's shelf level:

```bash
ros2 launch openarmx_task_executor supermarket_full_system.launch.py \
  station_config:=/home/openarmx/openflex_all/openflex_ws/src/openflex_integrated/openarmx_task_executor/config/stations_supermarket_test_07.yaml \
  start_shelf_view_server:=true \
  enable_shelf_view:=true

ros2 action send_goal --feedback \
  /execute_transport \
  openarmx_task_msgs/action/ExecuteTransport \
  "{task_id: shelf_view_demo_001, object_id: demo_object, source_station: SHELF_TABLE_APPROACH, destination_station: DROP_OFF, source_slot: '*', destination_slot: '*', use_source_level: true, source_level: 2, use_nominal_object_pose: false, max_retries: 1, timeout_sec: 180.0}"
```

The destination station must have a measured `place_pose`, and the source slot
must resolve to the configured shelf. The database's map coordinates are not
used as a motor command or to replace the station's Nav2 pose until the two
maps have been registered against one another.

The default `stations.yaml` is a placeholder and must be replaced with measured
poses. It is intentionally not a hardware or MoveIt launch file.

## supermarket_scene inventory integration

`supermarket_scene` is an external, SQLite-backed source of SKU instances. The
task executor does not import or write that repository. When
`competition.inventory.enabled` is true, the coordinator opens the configured
database read-only, resolves an order SKU to a deterministic
`shelf_id-face-level-y_cm` slot, and selects the pre-validated station mapped
by `shelf_face_stations`. The layer changes the manipulation target, not the
Nav2 `z` coordinate; different horizontal shelf faces or reach regions should
be represented by different measured stations.

The current database uses `height_cm` and the older database used
`z_offset_cm`; both are accepted. Its world position is converted using the
database's `map` convention and is passed as a nominal detection pose. The
perception server must still return a fresh measured pose before a physical
grasp.

The merge fragment
`config/inventory_supermarket_scene.example.yaml` shows the database path and
the required face-to-station registration. It intentionally contains a
placeholder station name: the database's `map` frame must first be registered
against the same physical map as the measured Nav2 poses. Pass the completed
competition profile through the coordinator/competition launch with
`competition_config:=...`.

The task API forwards `source_slot` and `source_level` into `DetectObject` and
`PickObject`. The pick contract also carries an optional calibrated
`nominal_lift_position` and sets `max_dof=8`, requiring fixed-base
`left/right_arm_with_lift` planning. A missing lift calibration is intentional:
the adapter must use the real detected pose and MoveIt IK rather than guessing
a motor coordinate or publishing raw lift controller commands.

## MoveIt integration contract

The MoveIt teammate should provide Action Servers named `/pick_object` and
`/place_object` using the actions in `openarmx_task_msgs`. A pose must include a
valid `header.frame_id`; the first integration should use `base_link` after
fine docking. The action server must return a deterministic error code, honor
cancel requests, and retreat the arm to a transport-safe pose after picking.

For shelf work the executor supplies `slot_id`, `shelf_level` and
`max_dof=8`. The adapter should keep the base fixed, use
`left_arm_with_lift`/`right_arm_with_lift`, and treat `nominal_lift_position`
as an optional calibrated seed only. It must plan from the measured object
pose; the database `world_z` is not a motor command.

The perception node should provide `/detect_object`. This package includes a
baseline `/dock_to_station` implementation. During bench testing either action
can be disabled with launch arguments, but production operation should keep
both enabled.

## TF docking implementation

`station_docking.launch.py` provides a baseline closed-loop docking server. It
expects a marker broadcaster to publish a frame named
`station_marker_<marker_id>`. The marker should appear at
the station's configured `dock.target_marker_pose` in `base_link` when aligned.
If no station target is supplied, the node uses `target_marker_x/y/yaw`. The
default command topic is `/cmd_vel`, which is the configured input of the Nav2
collision monitor. The active chain is `/cmd_vel` → collision monitor →
`/cmd_vel_safe_in` → localization safety gate → `/cmd_vel_safe` → swerve
controller. The controller publishes zero velocity on success, timeout, cancel,
and shutdown. Do not remap docking directly to `/cmd_vel_safe_in` or
`/cmd_vel_safe`, because that would bypass one or both safety layers.

The proportional controller is deliberately conservative and must be tuned on
the real base. It is a docking primitive, not a replacement for collision
monitoring or an emergency-stop circuit.

## Shelf scan fine alignment

`shelf_fine_align.launch.py` starts `/shelf_fine_align`. It is a separate
second-stage action used after Nav2 has stopped at a waypoint whose GUI role is
`shelf_approach`; ordinary Nav2 goals are unchanged. The server transforms
`/scan` returns from their message frame into `base_link` through TF, measures
sparse shelf uprights/front edges independently in the left and right front
corner bands, and filters both clearances across several Livox frames. Their
difference supplies the live shelf-relative yaw error; the waypoint's
configured map yaw remains an initial Nav2 handoff guard. Translation is held
for shelf-relative yaw errors above `3 degrees`. Inside that gate, the server
approaches at `0.030..0.040 m/s` while finishing the angular correction, but
holds translation again if either corner is already near the target. Close-
range completion uses ten stable samples, a `3 mm` clearance hysteresis, and
the `3 degree` guarded yaw gate because sparse returns can alternate between
different shelf uprights; it does not claim that those near-field returns can
certify a `1 degree` shelf normal. The lower speed bound stays above both the
deployed swerve controller's `0.015 m/s` chassis-command deadband and its
`0.020 m/s` per-wheel deadband.
If the scan TF is unavailable, the frame is discarded and the action fails
closed. The competition GUI requests a
`0.05 m` physical clearance and a `1 degree` yaw tolerance.

Commands use the same `/cmd_vel` safety chain described above. Stale scan,
odometry, missing/false localization state, missing or one-sided corner-band
returns, either front-corner clearance below `0.035 m`, excessive initial yaw
error, timeout, and cancellation all produce a zero command and abort the
action. `/localization_ok` is a reliable, transient-local state published on
health changes rather than a periodic heartbeat; a received `true` remains
valid until a later `false`. Reverse correction is deliberately not used.

## Post-pick shelf retreat

Competition launches start `/retreat_from_station` and enable task use of that
Action. Reverse motion remains opt-in per station through
`retreat.after_pick`, `retreat.after_place`, and `retreat.distance`; a station
without those flags never causes an automatic reverse. For
`supermarket_test_07`, the measured shelf entry currently requests `0.15 m`.

The task executor calls retreat only after `/pick_object` reports both
`success=true` and `object_attached=true` (or after a successful release for a
configured place station). The dedicated retreat server measures progress from
`/odom_safe`, commands the configured low speed on `/cmd_vel`, stops, and waits
for the base to settle before the task executor sends the next Nav2 goal. The
generic `task_executor.launch.py` keeps task use disabled unless
`enable_post_pick_retreat:=true` is supplied.

Nav2 remains forward-only. While retreating, the retreat server refreshes
`/competition/post_pick_reverse_authorized`; the final velocity safety gate
accepts only a low-speed command with negative `linear.x` and zero lateral and
angular velocity. The authorization expires after `0.25 s` without a heartbeat.
Stale odometry, more than `0.03 m` lateral drift, more than `3 degrees` yaw
drift, cancellation, or timeout revokes authorization and forces zero velocity.

## GUI-only mock grasp

`competition_task_api.launch.py` also starts the isolated
`/competition/mock_pick_object` Action for chassis handoff testing. It returns
`success=true` and `object_attached=true` after a short delay but does not run
real detection, SAM2, GraspNetBaseline, MoveIt grasp execution, a gripper, or
attachment sensing. It deliberately does
not use `/pick_object`, so a future real grasp adapter can own that production
endpoint without an Action-name conflict.

The full-body competition GUI uses this isolated endpoint only for the visible
test chain `mock pick -> RetreatFromStation -> NavigateToPose`. At least two
physically verified mission stations are required so the source and next goal
cannot be the same. Stopping the navigation workflow cancels any active mock
pick or retreat goal.

## Send a transport task

```bash
ros2 action send_goal /execute_transport \
  openarmx_task_msgs/action/ExecuteTransport \
  "{task_id: demo_001, object_id: demo_object, source_station: shelf_A, destination_station: drop_A, max_retries: 1, timeout_sec: 180.0}" \
  --feedback
```

Before enabling hardware, verify the action graph:

```bash
ros2 action info /execute_transport
ros2 action info /retreat_from_station
ros2 action info /competition/mock_pick_object
ros2 action info /pick_object
ros2 action info /place_object
```

The mock endpoint is only an integration aid. `/pick_object` and
`/place_object` must still show the real manipulation team's servers before a
real transport task can succeed.
