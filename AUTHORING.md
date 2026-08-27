# SSA scenery authoring guide

## Vehicle models and in-game routes

Add a vehicle model entry at the top level of `ssa.json`:

```json
"vehicle_models": [
  {
    "id": "gapura_bus",
    "label": "Gapura Apron Bus",
    "object": "object/Gapura_bus.obj",
    "ground_offset_m": 0.445,
    "heading_offset_deg": 180.0,
    "steering_multiplier": -1.0,
    "smoothing_iterations": 3,
    "speed_mps": 4.0
  }
]
```

The OBJ should use these animation datarefs:

```text
boldstudio31/ssa/vehicle/wheel_spin  (0..1, loop 1)
boldstudio31/ssa/vehicle/steering    (-1..0..1, no loop)
```

In X-Plane, open the SSA tablet, enable Developer Mode under Settings and open
the DEV tab. Start the top-down planner at the aircraft, click the apron to add
numbered GPS waypoints, use Shift + middle-mouse drag on Windows or the on-screen
direction pad to pan and the mouse wheel to zoom, then test or save from the boxed
overlay toolbar. The CURVE control previews smoothing levels 0 to 4. After
a test finishes, SSA returns to the same planning view. `steering_multiplier` reverses
an OBJ whose front-wheel animation faces the wrong direction. Route testing uses
`smoothing_iterations` (0 to 4) to round waypoint corners. SSA writes
`ssa_routes.json` beside the scenery's `ssa.json`.

## Folder layout

Place `ssa.json` in the top folder of the airport scenery:

```
Custom Scenery/My Airport/ssa.json
Custom Scenery/My Airport/objects/...
```

Use `examples/ssa.json` as the starting point. Every `id` and `dataref` must be
unique. Prefer this naming convention:

`boldstudio31/ssa/animation/scenery_id/object_id`

Use a short unique scenery ID rather than an airport ICAO. Example:

`boldstudio31/ssa/animation/test_ssa/hangar_door_01`

## Blender / XPlane2Blender

Animate every service over a normalized value from **0.0 (parked/closed)** to
**1.0 (deployed/open)**. Enter the matching SSA dataref in XPlane2Blender.

### Hangar recommendation

- `base`: static parent
- `door_left`: animated from 0 to 1
- `door_right`: animated from 0 to 1 when used
- Put the origin/pivot on the real rail or hinge.
- Use one SSA dataref for both doors; reverse the right-side keyframes when
  necessary.

### Jetway recommendation

- `base_pedestal`: static
- `rotunda`: yaw
- `tunnel_1` and `tunnel_2`: extension
- `cabin_head`: final yaw/height
- `wheel_bogie`: follows extension and height

Jetways use separate normalized channels from 0.0 (parked) to 1.0 (connected):

- `rotunda_ratio`: rotunda yaw.
- `extension_ratio`: tunnel extension.
- `height_ratio`: tunnel/cabin height.
- `cabin_yaw_ratio`: cabin alignment.
- `wheel_steer_ratio`: horizontal bogie steering.
- `wheel_rotation_ratio`: visible bogie wheel movement.

See `examples/ssa.json` for complete channel dataref names and speeds.

The optional `kinematics` block enables closed-loop door targeting. Its values
describe the exact transform hierarchy exported in the OBJ:

- `door_forward_m`: L1 door offset forward from the aircraft reference point.
- `door_right_m`: lateral door offset; negative values are on the aircraft's left.
- `door_sill_height_m`: door-sill height above the ground.
- `object_heading_deg`: heading assigned to the OBJ in WED.
- `root_height_m` and `height_pivot_*_m`: root and pitch-pivot translations.
- `tunnel_parked_x_m`: combined tunnel/cabin translation at ratio 0.
- `extension_x_m`: combined extension added at ratio 1.
- `head_*_m`: contact point on the cabin head in cabin-local coordinates.
- `rotunda_degrees`, `height_degrees`, `cabin_degrees`: rotation at ratio 1.
- `connect_tolerance_m`: maximum head-to-door distance for `CONNECTED`.
- `max_solution_error_m`: reject docking as `OUT OF RANGE` when no valid
  kinematic solution is closer than this value.

Automatic mode starts OFF. Test manual Connect first. The tablet displays the
live head-to-door error in centimetres while docking.

### Moving car recommendation

- `base_path`: follows the route
- `body`: child of base path
- `wheel_fl`, `wheel_fr`, `wheel_rl`, `wheel_rr`: children with rotation

The production vehicle module will provide separate path-progress and
wheel-rotation datarefs.

## Automatic jetway rules

- Turboprop: no jetway.
- Narrow-body: one jetway at the forward passenger door.
- Wide-body: one jetway per supported forward door before the wing, limited by
  the parking stand's available jetways.
- Default activation radius: 35 m.

Version 0.7.3 includes an initial default-B738 profile and several common
narrow-body profiles. Aircraft without a profile use a conservative fallback;
their door position may require later calibration.

## SAM and AutoGate

Do not animate the same object with SSA and SAM/AutoGate simultaneously. SSA is
standalone; this prevents two plugins from writing competing animation states.
