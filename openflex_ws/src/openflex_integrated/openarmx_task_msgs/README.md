# openarmx_task_msgs

This package defines the interface boundary between the competition task
executor and the navigation, perception, and MoveIt components.

The task executor calls standard Nav2 `NavigateToPose` and these actions:

- `DockToStation`: local station alignment after coarse navigation.
- `ShelfFineAlign`: scan-based forward-only shelf alignment after coarse navigation.
- `ShelfTraverse`: guarded shelf-parallel return to a calibrated anchor after pick.
- `RetreatFromStation`: guarded straight retreat after pick or shelf placement.
- `DetectObject`: object pose and confidence after the base is stopped.
- `PickObject`: MoveIt-side grasp and retreat.
- `PlaceObject`: MoveIt-side release and retreat.
- `ExecuteTransport`: the public end-to-end competition action.

The three-stage supermarket controller adds these higher-level contracts:

- `ExecuteCompetitionPhase`: consumes exactly one referee start command for one
  ordered phase and finishes inside the configured judging area.
- `ReadOrder`: converts the delivery note into validated SKU/specification/count
  lines. OCR or VLM implementations stay behind this interface.
- `InspectRetailScene`: converts a stopped shelf observation, optionally backed
  by DREAM semantic memory, into missing-stock or misplaced-item work items.
- `VerifyPlacement`: confirms that a placed object is stable and inside the
  requested shelf slot.
- `SetPerceptionMode`: reports competition perception workflow state. The
  Qwen+SAM2 and GraspNetBaseline workers use separate environments and remain
  resident; this action must not silently switch the selected box provider.

`CompetitionWorkItem` is the boundary between perception and deterministic task
execution. Perception proposes evidence-backed work; the coordinator validates,
deduplicates and orders it before calling `ExecuteTransport`.

`ExecuteTransport` carries the resolved `source_slot`, optional shelf level and
nominal object pose. `DetectObject` receives the same slot/level as a search
hint. `PickObject` receives the level, an optional calibrated lift joint seed,
and `max_dof=8`; this reserves the fixed-base arm-plus-lift planning contract.
The nominal pose and lift seed are hints only: the physical adapters must use a
fresh perception pose, MoveIt joint limits, and attachment verification.

All poses are `geometry_msgs/PoseStamped`; consumers must honor `header.frame_id`
and `header.stamp`. The first integration should use `base_link` for pick/place
poses after docking. `TaskError.msg` is the shared error-code registry.
