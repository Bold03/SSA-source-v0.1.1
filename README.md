# SSA 0.30.2 - Vehicle Test Movement Fix

# SSA 0.30.1 - Vehicle Test Fix

# SSA 0.30.0 - Mockup UI Refresh

# SSA 0.29.0 — Scenery Service Animation

**Developer:** BoldStudio31  
**Targets:** X-Plane 11.50+ and X-Plane 12; Windows, Linux and macOS

SSA is a standalone scenery-animation controller for hangars, jetways, airport vehicles, ground staff and VDGS/parking displays.

## 0.29.0 UI

This version replaces the large tablet/dashboard layout with a compact classic simulator utility interface based on the SSA mock-up.

Player screens:
- SSA Main Menu
- Hangar Menu
- Jetway Menu
- Parking / VDGS Menu
- Settings

Developer Mode is OFF by default. Enabling it in Settings unlocks:
- Vehicle Developer Menu
- Parking / VDGS Developer Menu
- Route authoring controls
- 3D VDGS placement controls
- Manual wheel-spin and steering tests

## Hangar

- Shows nearby configured hangars.
- Select a hangar row, then use OPEN HANGAR, CLOSE HANGAR or TOGGLE DOOR.
- Live state: CLOSED, OPENING, OPEN or CLOSING.

## Jetway

- Shows nearby jetways and their live docking state.
- Select a jetway, then CONNECT or DISCONNECT it.
- Automatic Jetway Mode can be enabled or disabled from the Jetway or Settings menu.
- Existing staged docking logic and aircraft-door targeting are retained.

## Parking / VDGS

- Lists nearby VDGS parking positions.
- Select one stand manually or return to AUTO CORRIDOR selection.
- Live guidance includes acquisition, lateral correction, SLOW, STOP and overshoot state.
- Nose-wheel offset is used when available, with aircraft length as fallback.

## Developer — Vehicles

- Create and edit vehicle routes without Blender curves.
- Top-down planner supports route anchors, Bezier smoothing, test, save and undo.
- Vehicle traffic supports loop routes, multiple vehicles, spawn intervals, adaptive speed, corner slowing and collision avoidance.
- Vehicle visibility is limited to the configured airport presence radius. Default: 8000 m.

## Developer — Parking / VDGS

- Place VDGS models directly in X-Plane.
- Move left/right/forward/back and altitude.
- Rotate heading.
- Adjust acquisition range and corridor width.
- Save placement back to the scenery configuration.

## Datarefs and configuration

- Dynamic scenery datarefs are loaded from `ssa.json`.
- Vehicle routes are stored in `ssa_routes.json`.
- `vehicle_presence_radius_m` controls how far from the airport SSA background traffic remains active.
- `tools/SSA-Dataref-Generator.html` helps generate sanitized SSA datarefs.
- `tools/SSA-JSON-Builder.html` is an offline editor for `ssa.json`.

## Build without Visual Studio

1. Create a GitHub repository.
2. Upload the contents of this source archive.
3. Open **Actions → Build SSA → Run workflow**.
4. Download the Windows build artifact.
5. Put the built plugin in:
   - `X-Plane 11/Resources/plugins/SSA/`
   - or `X-Plane 12/Resources/plugins/SSA/`

Open SSA from **Plugins → SSA → Open tablet**.

Command: `boldstudio31/ssa/tablet/toggle`

## Important

SSA is still a development build. Test new hangar, jetway and VDGS animation limits in a test copy of the scenery before distributing it.
