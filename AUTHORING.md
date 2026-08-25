# SSA scenery authoring guide

## Folder layout

Place `ssa.json` in the top folder of the airport scenery:

```
Custom Scenery/My Airport/ssa.json
Custom Scenery/My Airport/objects/...
```

Use `examples/ssa.json` as the starting point. Every `id` and `dataref` must be
unique. Prefer this naming convention:

`boldstudio31/ssa/ICAO/object_id`

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

For the first build, keyframe all parts against one normalized deployment
dataref. Later SSA versions will add separate yaw, extension and height channels
for geometry-based docking.

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

Accurate docking requires an aircraft profile containing ICAO type and door
coordinates. Aircraft without a profile will use a safe one-jetway fallback.

## SAM and AutoGate

Do not animate the same object with SSA and SAM/AutoGate simultaneously. SSA is
standalone; this prevents two plugins from writing competing animation states.

