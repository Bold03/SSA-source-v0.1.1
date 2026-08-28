# SSA — Scenery Service Animation

**Developer:** BoldStudio31  
**Status:** saved-route apron bus runtime beta 0.12.0  
**Targets:** X-Plane 11.50+ and X-Plane 12; Windows, Linux and macOS

SSA is a standalone animation controller for scenery objects. It provides custom
datarefs for hangars, jetways, moving vehicles, ground staff and parking
displays. It does not depend on SAM or AutoGate.

## Implemented through 0.12.0

- Dynamic custom datarefs loaded from each scenery package's `ssa.json`.
- Smooth open/close animation with configurable speed.
- Tablet-style native X-Plane window with Hangar, Jetway and Bus tabs.
- Player Mode is the safe default and hides diagnostics and scenery-authoring tools.
- The tablet Settings tab contains a clear Developer Mode ON/OFF control.
  Normal players leave it OFF; enabling it adds a DEV tab with aircraft
  coordinates, object diagnostics and the in-game Moving Car Route Editor area.
- The optional `boldstudio31/ssa/developer/toggle` command remains available
  for scenery authors who want a keyboard shortcut.
- Developer Mode includes a Moving Car Route Editor that loads the configured
  OBJ as an X-Plane instance without requiring a Blender curve.
- Float datarefs support configurable ranges, allowing vehicle steering to use
  the natural `-1` (left), `0` (center), `1` (right) convention while existing
  hangar and jetway animations remain normalized to `0..1`.
- Route editing includes top-down anchor placement plus legacy 2 m movement and
  15 degree turn controls, undo, test, cancel and JSON save.
- Vehicle instances follow terrain height with a configurable model ground
  offset. Route tests drive wheel spin and front steering automatically.
- Saved Bezier anchors use latitude/longitude rather than transient X-Plane local
  coordinates and are written to the scenery's `ssa_routes.json`.
- BetterPushback-style top-down planning mode captures anchor clicks over the apron,
  draws numbered markers and a dotted automatic Bezier route, supports wheel zoom, and
  exposes UNDO, TEST, SAVE and EXIT controls in the overlay.
- The on-screen direction pad pans the top-down camera while planning. Toolbar
  controls have visible, exact button rectangles so TEST cannot be mistaken for UNDO.
- Configurable steering inversion fixes models whose front wheels turn in the
  opposite direction. Steering input follows the sampled Bezier curvature.
- A completed or manually stopped overlay test automatically returns to the
  same top-down planning view.
- Bezier routes are sampled about every 0.35 metres; the bus slows for sharp turns
  and the tablet DEV controls use separated visible buttons with exact hitboxes.
- Windows route planning supports Shift + middle-mouse dragging without taking
  keyboard focus. The dotted preview displays the generated cubic Bezier route.
- Front-wheel steering follows route curvature with look-ahead and a straight-line
  dead zone, preventing left/right oscillation. Stopping a planner test returns
  directly to the retained top-down Create Route view.
- Bus body heading uses a multi-metre forward target and centred path tangents,
  then applies damped rotation plus a heading dead zone. This filters small
  alternating segment angles without removing genuine turns.
- Each click is an automatic Bezier anchor. LOOP can close routes with three or
  more anchors, continuously replay the bus animation, and is stored in
  `ssa_routes.json`; stopping the test returns to the retained planner.
- Vehicle instances now update on every simulator frame instead of at 20 Hz.
  A distance-budget walker crosses multiple Bezier samples without inserting
  empty frames, eliminating the repeated micro-pause at each sample.
- Traffic-style acceleration and braking provide gradual starts, corner speed
  control and smooth stops. Frame delta is capped after simulator stalls so a
  temporary low-FPS spike cannot teleport the vehicle down the route.
- Per-model heading offsets correct OBJ forward-axis differences without
  re-exporting Blender; the Gapura bus profile uses 180 degrees.
- Right-click-and-hold dragging on an anchor creates aligned manual Bezier
  handles, allowing scenery authors to keep routes clear of buildings while
  retaining automatic handles on untouched anchors.
- Vehicle motion is referenced to the rear axle. A configurable axle-to-origin
  offset places the OBJ body ahead of the route, while wheelbase-based bicycle
  steering lets the front of the bus swing through turns without rear drift.
- Saved `ssa_routes.json` files are loaded with their automatic or custom
  Bezier handles when the scenery starts. The bus is placed at the first route
  point and remains available without Developer Mode.
- The Player Mode BUS tab shows the saved route, selected vehicle model, loop
  setting and runtime status, with separate START BUS and STOP BUS controls.
- Nearby hangar list (2 km) and small jetway activation radius (35 m).
- Per-channel automatic jetway targets calculated from aircraft position,
  heading, door profile and scenery-configured movement limits.
- Initial forward-left-door profiles for the default B738 plus B737/B739 and
  A319/A320/A321; unknown aircraft receive a conservative generic profile.
- GitHub Actions builds for Windows, Linux and universal macOS.
- Windows `NOMINMAX` protection so MSVC does not replace `std::min/std::max`.
- Automatic RealOps detection through its plugin signature and visible tablet status.
- RealOps safety suppression for future SSA vehicle and ground-staff objects.
- Configuration reload from the tablet, plugin menu, command, or scenery reload.
- Loaded-object count in the tablet and `Log.txt`.
- Hangar state labels: CLOSED, OPENING, OPEN and CLOSING.
- Open, close and toggle commands for the nearest hangar within 2 km.
- Animation position and target survive configuration reloads.
- Duplicate datarefs and unsupported object types are rejected safely.
- Six independent normalized jetway animation channels, including bogie steering.
- Manual connect, disconnect and toggle commands for the nearest jetway.
- Automatic jetway only operates while the aircraft is on the ground and nearly stopped.
- Common turboprops are rejected; narrow-body aircraft receive one jetway and
  recognized wide-bodies may receive two nearby jetways.
- Corrected the Jetway tab mouse hitbox.
- Automatic mode starts OFF so a new model can be calibrated safely.
- Per-scenery door-offset overrides allow aircraft targeting to be calibrated
  in `ssa.json` without recompiling the plugin.
- Forward kinematics extracted from the exported OBJ calculate the cabin-head
  position for the current rotunda, extension, height and cabin-yaw values.
- Numerical inverse kinematics finds independent channel targets before motion.
- SAM-style staged motion: WHEEL ALIGNING, HEAD 45 DEG, ALIGNING,
  APPROACHING, SEALING and CONNECTED.
- The bogie steers before bridge motion to prevent a sideways drifting appearance.
- The cabin head turns to 45 degrees and remains there. The bridge approaches
  to a configurable one-metre clearance, then performs the final extension to
  the contact tolerance without rotating the head back.
- Tablet clicks, commands and Automatic mode all use the same safe docking path.
- Live head-to-door distance is checked every frame; `CONNECTED` is shown only
  within the configured tolerance, otherwise the tablet shows `DOCKING` or
  `OUT OF RANGE`.
- The default B738 L1 door-state dataref is detected at startup; its XYZ target
  comes from the aircraft profile because X-Plane door datarefs contain state,
  not world coordinates.

## Planned before a production release

- Expanded, aircraft-specific door profile database and per-aircraft overrides.
- Wide-body multi-jetway assignment and collision/occupancy checks.
- SimBrief Pilot ID login, OFP state machine and departure/arrival triggers.
- Cross-platform positional announcement audio with distance attenuation.
- Vehicle paths/wheel rotation, ground-staff sequences, parking display text
  and a polished textured tablet UI.
- Deeper RealOps integration if its developers publish a supported inter-plugin API.

## Build without Visual Studio

Create a GitHub repository, upload this folder, then open **Actions → Build SSA →
Run workflow**. GitHub performs the compilation. Download the three artifacts
and merge their `SSA` folders. Install the merged folder at:

`X-Plane 11/Resources/plugins/SSA/` or `X-Plane 12/Resources/plugins/SSA/`

Open the tablet from **Plugins → SSA → Open tablet**. The command is
`boldstudio31/ssa/tablet/toggle`.

## Important

This is a beta, not yet a finished public plugin. Test it in a copy
of a scenery first. Jetway movement limits must match the transforms authored
in Blender. Additional aircraft profiles and licensed announcement recordings
are still required.
