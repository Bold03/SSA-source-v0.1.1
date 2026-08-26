# SSA scenery authoring guide

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

The optional `kinematics` block enables door targeting. Its values describe
the real movement represented by dataref values 0 and 1:

- `parked_heading_deg`: world heading used as the rotunda's zero reference.
- `rotunda_min_deg` / `rotunda_max_deg`: yaw angles relative to that reference.
- `parked_length_m`: base-to-cabin distance at `extension_ratio = 0`.
- `extension_travel_m`: extra reach represented by `extension_ratio = 1`.
- `deck_min_m` / `deck_max_m`: cabin floor heights represented by 0 and 1.
- `cabin_yaw_min_deg` / `cabin_yaw_max_deg`: cabin alignment angles represented
  by 0 and 1.

Automatic mode starts OFF. Test manual Connect first and adjust these limits if
the model stops short, overshoots, or rotates in the opposite direction.

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

Version 0.5 includes an initial default-B738 profile and several common
narrow-body profiles. Aircraft without a profile use a conservative fallback;
their door position may require later calibration.

## SAM and AutoGate

Do not animate the same object with SSA and SAM/AutoGate simultaneously. SSA is
standalone; this prevents two plugins from writing competing animation states.
