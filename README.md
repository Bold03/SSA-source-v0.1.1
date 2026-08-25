# SSA — Scenery Service Animation

**Developer:** BoldStudio31  
**Status:** foundation build 0.1.1  
**Targets:** X-Plane 11.50+ and X-Plane 12; Windows, Linux and macOS

SSA is a standalone animation controller for scenery objects. It provides custom
datarefs for hangars, jetways, moving vehicles, ground staff, parking displays
and digital billboards. It does not depend on SAM or AutoGate.

## Implemented in 0.1.1

- Dynamic custom datarefs loaded from each scenery package's `ssa.json`.
- Smooth open/close animation with configurable speed.
- Tablet-style native X-Plane window with Hangar and Jetway tabs.
- Nearby hangar list (2 km) and small jetway activation radius (35 m).
- First automatic jetway safety rule: turboprops are rejected; other aircraft
  receive one front jetway.
- GitHub Actions builds for Windows, Linux and universal macOS.
- Windows `NOMINMAX` protection so MSVC does not replace `std::min/std::max`.

## Planned before a production release

- Aircraft profile database and accurate forward-door geometry.
- Wide-body multi-jetway assignment and collision/occupancy checks.
- SimBrief Pilot ID login, OFP state machine and departure/arrival triggers.
- Cross-platform positional announcement audio with distance attenuation.
- Vehicle paths/wheel rotation, ground-staff sequences, parking display text,
  billboard content and a polished textured tablet UI.
- An explicit compatibility adapter after the exact “Plugin Ops” product/API is
  identified.

## Build without Visual Studio

Create a GitHub repository, upload this folder, then open **Actions → Build SSA →
Run workflow**. GitHub performs the compilation. Download the three artifacts
and merge their `SSA` folders. Install the merged folder at:

`X-Plane 11/Resources/plugins/SSA/` or `X-Plane 12/Resources/plugins/SSA/`

Open the tablet from **Plugins → SSA → Open tablet**. The command is
`boldstudio31/ssa/tablet/toggle`.

## Important

This is a source foundation, not yet a finished public plugin. Test it in a copy
of a scenery first. The model assets, airport coordinates, aircraft door
profiles and licensed announcement recordings are still required.
