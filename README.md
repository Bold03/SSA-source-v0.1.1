# SSA — Scenery Service Animation

**Developer:** BoldStudio31  
**Status:** staged jetway docking and developer UI beta 0.8.0  
**Targets:** X-Plane 11.50+ and X-Plane 12; Windows, Linux and macOS

SSA is a standalone animation controller for scenery objects. It provides custom
datarefs for hangars, jetways, moving vehicles, ground staff and parking
displays. It does not depend on SAM or AutoGate.

## Implemented through 0.8.0

- Dynamic custom datarefs loaded from each scenery package's `ssa.json`.
- Smooth open/close animation with configurable speed.
- Tablet-style native X-Plane window with Hangar, Jetway and Bus tabs.
- Player Mode is the safe default and hides diagnostics and scenery-authoring tools.
- Developer Mode is enabled explicitly from the SSA plugin menu or the
  `boldstudio31/ssa/developer/toggle` command. It adds a DEV tab with aircraft
  coordinates, object diagnostics and the in-game Moving Car Route Editor area.
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
