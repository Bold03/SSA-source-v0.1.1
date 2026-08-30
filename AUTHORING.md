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
    "body_lookahead_m": 6.0,
    "body_heading_response": 1.8,
    "rear_axle_to_origin_m": 3.8,
    "wheelbase_m": 7.6,
    "max_steering_deg": 35.0,
    "speed_mps": 4.0,
    "max_speed_mps": 9.0,
    "adaptive_speed_start_m": 80.0,
    "adaptive_speed_full_m": 300.0,
    "turn_preview_seconds": 3.0,
    "turn_preview_min_m": 15.0,
    "corner_min_speed_mps": 2.2,
    "corner_full_slowdown_deg": 35.0,
    "acceleration_mps2": 1.5,
    "braking_mps2": 2.5,
    "collision_enabled": true,
    "collision_refresh_s": 0.10
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
numbered automatic Bezier anchors. Right-click and hold an anchor, then drag to
create an aligned custom Bezier handle like a graphics Pen Tool. Use Shift +
middle-mouse drag on Windows or
the on-screen direction pad to pan and the mouse wheel to zoom, then test or save
from the boxed overlay toolbar. SSA calculates cubic Bezier handles from the
neighbouring anchors until a custom handle is dragged. LOOP ON joins
the final anchor to the first and requires at least three anchors. STOP TEST
returns to the same planning view. `steering_multiplier` reverses an OBJ whose
front-wheel animation faces the wrong direction. SSA writes `ssa_routes.json`
beside the scenery's `ssa.json`.

The planner's right panel also configures the Traffic Manager for that route:

- `BUS COUNT`: one to five independent buses.
- `SPAWN`: delay between buses from 5 to 300 seconds.
- `SPEED`: base route speed from 5 to 50 km/h; normal adaptive long-route
  speed and corner braking still apply.
- `BUS`: choose one fixed OBJ or `RANDOM MODELS` to distribute the available
  configured models across the route.

These values are written to `ssa_routes.json` schema 4 as `bus_count`,
`spawn_interval_s`, `speed_mps`, and `model`. Older route schemas load with
one bus and a 45-second interval, so existing scenery routes do not need to be
rebuilt.

After SAVE, reload SSA or restart X-Plane. SSA reads `ssa_routes.json`, places
the background bus at the first anchor and starts saved routes automatically.
The TRAFFIC tab beside DEV provides manual START TRAFFIC and STOP TRAFFIC
controls and is visible only in Developer Mode. Traffic continues when
Developer Mode is disabled. A closed route continues running when LOOP is ON;
an open route brakes at its final anchor and then leaves the background traffic
system so later buses can clear the same endpoint. Future RealOps passenger
buses are intentionally separate from this background traffic system.

SSA supports up to sixteen saved background routes per scenery with the current
configured vehicle models. Each new PLAN
ROUTE session receives the next free `bus_route_XX` identifier, and SAVE appends
it to `ssa_routes.json` without replacing earlier routes. The TRAFFIC tab can
start or stop each of the first five routes separately and provides START ALL
and STOP ALL for the complete collection.

`body_lookahead_m` controls how far ahead the bus looks when choosing its body
heading; a longer distance suppresses small left/right corrections. The
`body_heading_response` value controls how quickly the body rotates toward that
heading. The Gapura bus defaults to 6 metres and 1.8 respectively.

The route itself is the rear-axle path, not the bus-body origin path. Set
`rear_axle_to_origin_m` to the forward distance from the rear axle to the OBJ
origin. `wheelbase_m` and `max_steering_deg` drive the front-wheel steering from
the route curvature, so the rear wheels stay on the line while the nose swings
naturally through a corner.

Vehicle instances update every simulator frame. `acceleration_mps2` and
`braking_mps2` control traffic-style speed changes; the default bus accelerates
at 1.5 m/s2 and brakes at 2.5 m/s2 instead of instantly changing speed.

Adaptive cruise speed keeps short apron routes at `speed_mps` and gradually
raises longer routes toward `max_speed_mps`. The default ramp starts at 80 m
and reaches full speed at 300 m. Curve-speed reduction and final braking remain
active, so increasing long-route speed does not remove corner control.

Turn-preview braking scans several seconds ahead on the sampled Bezier path.
With the defaults, SSA looks at least 15 m ahead, begins braking before a bend,
and reaches about 2.2 m/s for turns of 35 degrees or sharper. It accelerates
again only after the upcoming path straightens, reducing body and rear-axle
drift at waypoint transitions.

`collision_refresh_s` controls how often each active bus rescans nearby SSA
traffic. The recommended `0.10` seconds keeps collision response fast while
avoiding a full all-bus scan on every rendered frame. Visible vehicle movement,
wheel spin and steering still update every frame.

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

## VDGS parking display

Export the VDGS OBJ with these exact animation datarefs:

```text
boldstudio31/ssa/vdgs/active
boldstudio31/ssa/vdgs/left
boldstudio31/ssa/vdgs/right
boldstudio31/ssa/vdgs/center
boldstudio31/ssa/vdgs/slow
boldstudio31/ssa/vdgs/stop
boldstudio31/ssa/vdgs/lateral
boldstudio31/ssa/vdgs/distance_ratio
```

Add one `parking_display` object to `ssa.json`. Its latitude and longitude are
the VDGS pole position. `object_heading_deg` points from the display toward the
approaching aircraft. `stop_distance_m` is the desired aircraft reference-point
distance in front of the display.

```json
{
  "id": "vdgs_gate_a1",
  "label": "Gate A1 VDGS",
  "type": "parking_display",
  "latitude": -6.26645,
  "longitude": 106.89102,
  "radius_m": 90,
  "vdgs": {
    "object_heading_deg": 0.0,
    "stop_distance_m": 18.0,
    "use_aircraft_length": true,
    "nose_clearance_m": 2.5,
    "acquisition_distance_m": 80.0,
    "corridor_half_width_m": 8.0,
    "slow_distance_m": 12.0,
    "stop_tolerance_m": 0.5,
    "lateral_full_scale_m": 3.0,
    "lateral_deadband_m": 0.15,
    "lateral_stop_tolerance_m": 0.35,
    "lateral_multiplier": -1.0
  }
}
```

This VDGS OBJ uses `distance_ratio = 1` at the upper/far position and `0` at
the lower/stop position. If its marker or arrows appear mirrored in X-Plane,
change only `object_heading_deg` or `lateral_multiplier`; the Blender animation
does not need to be rebuilt.

When `use_aircraft_length` is enabled, `stop_distance_m` is a minimum/fallback.
SSA normally stops the aircraft reference point at half the loaded aircraft
length plus `nose_clearance_m`, preventing different aircraft noses from
reaching the panel. Increase `nose_clearance_m` if a particular model still
stops too close.

`acquisition_distance_m` and `corridor_half_width_m` form a straight approach
corridor in front of the display. In AUTO CORRIDOR mode SSA activates only the
first VDGS whose corridor contains the aircraft. A player can instead select a
gate in the tablet VDGS tab before entering the apron. The selected display is
ARMED, every other VDGS stays dark, and acquisition begins only after the
aircraft enters that gate's corridor.

### Place VDGS inside X-Plane

Put the OBJ and its textures in either of these conventional locations:

```text
Custom Scenery/My Airport/object/VDGS.obj
Custom Scenery/My Airport/objects/VDGS.obj
```

Open SSA Settings, enable Developer Mode, open DEV and press `PLACE VDGS`.
The live preview initially appears 20 metres in front of the aircraft. Use the
buttons to move it left/right/forward/back, change altitude by 0.1 metres, and
rotate it by five degrees. `SAVE VDGS` appends the placement to `ssa.json` and
stores `latitude`, `longitude`, `altitude_m`, `heading`, the relative OBJ path,
and its default guidance calibration.

Do not also place the same saved VDGS in WED. SSA restores the runtime instance
from `ssa.json` whenever the scenery configuration loads.

For a nonstandard filename, define the authoring model at the top level:

```json
"vdgs_models": [
  {
    "id": "default_vdgs",
    "label": "SSA VDGS",
    "object": "object/My_VDGS.obj"
  }
]
```

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

## 3D announcements

Place announcement audio in the scenery package as a mono PCM 16-bit WAV file.
Stereo files are rejected because a spatial source must have one position.

```json
"announcements": [
  {
    "id": "terminal_pa_01",
    "label": "Terminal Public Address 01",
    "audio": "audio/terminal_pa_01.wav",
    "latitude": -0.1479,
    "longitude": 109.4044,
    "altitude_m": 4.0,
    "gain": 0.8,
    "radius_m": 150.0,
    "autoplay": true,
    "loop": false,
    "start_delay_s": 5.0,
    "repeat_interval_s": 90.0
  }
]
```

### Modular flight Announcement Composer

SSA can assemble one announcement from small reusable WAV clips. Clips are
joined in this order: airline introduction, each flight-number digit, optional
origin, optional destination, event phrase, and each optional gate digit.

Add a reusable library and flight job to the scenery's `ssa.json`:

```json
"announcement_library": {
  "base_folder": "audio/announcements",
  "gap_ms": 100,
  "airlines": { "lion_air": "airlines/lion_air_jt.wav" },
  "digits": {
    "0": "digits/nol.wav", "1": "digits/satu.wav",
    "2": "digits/dua.wav", "3": "digits/tiga.wav",
    "4": "digits/empat.wav", "5": "digits/lima.wav",
    "6": "digits/enam.wav", "7": "digits/tujuh.wav",
    "8": "digits/delapan.wav", "9": "digits/sembilan.wav"
  },
  "origins": { "jakarta": "origins/dari_jakarta.wav" },
  "destinations": { "makassar": "destinations/tujuan_makassar.wav" },
  "events": {
    "landed": "events/telah_mendarat.wav",
    "boarding": "events/dipersilakan_naik_melalui_pintu.wav"
  }
},
"flight_announcements": [
  {
    "id": "lion_jt684_boarding",
    "airline": "lion_air",
    "flight_number": "684",
    "destination": "makassar",
    "event": "boarding",
    "gate": "2",
    "latitude": -6.2662,
    "longitude": 106.8906,
    "altitude_m": 4.0,
    "radius_m": 150.0,
    "repeat_interval_s": 120.0
  }
]
```

`flight_number` and `gate` must be strings so leading zeroes are preserved.
Every referenced clip must be mono PCM 16-bit WAV. Clips may use different
sample rates; SSA resamples them to the first clip's rate. `gap_ms` controls
the silence between clips and may be overridden for one flight job.

For an arrival from Jakarta, omit `destination` and `gate`, then set
`"origin": "jakarta"` and `"event": "landed"`. A missing key or audio
file rejects only that composed job; legacy full-file announcements still load.

### Place a composed announcement in X-Plane

Compile and install SSA, open Settings, enable Developer Mode and select the
`ANN` tab. Choose the airline and event with PREV/NEXT, set the numeric flight
number and gate, then choose the origin or destination shown for that event.
GAIN changes source loudness and RADIUS changes its maximum audible distance.

Press `PLACE SPEAKER`. A green volume symbol appears 20 metres ahead of the
aircraft with draggable X, Y and Z world-axis handles. It is an editor overlay,
not a scenery OBJ. Use the tablet buttons for precise one-metre horizontal and
0.1-metre altitude adjustments. `SAVE SPEAKER` appends the completed job and
coordinates to `flight_announcements` in `ssa.json`, reloads the scenery
configuration and removes the volume symbol. `CANCEL` changes nothing.

- `radius_m`: maximum audible distance from the active camera.
- `gain`: source volume from 0.0 to 2.0; begin testing at 0.8.
- `start_delay_s`: delay after entering the radius before the first playback.
- `repeat_interval_s`: silence after a WAV finishes before it plays again.
- `loop`: continuous playback; use it for ambience, not ordinary PA messages.
- Multiple speaker points may use the same WAV file.

Version 0.7.3 includes an initial default-B738 profile and several common
narrow-body profiles. Aircraft without a profile use a conservative fallback;
their door position may require later calibration.

## SAM and AutoGate

Do not animate the same object with SSA and SAM/AutoGate simultaneously. SSA is
standalone; this prevents two plugins from writing competing animation states.
