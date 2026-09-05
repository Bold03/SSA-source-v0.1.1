# SSA 0.29.0 - Classic Simulator UI

## UI redesign
- Replaced the Airbus/tablet dashboard look with a compact classic simulator utility window based on the user's SSA Figma mock-up.
- Added a radial-style SSA main menu.
- Added classic menu screens for Hangar, Jetway, Parking/VDGS and Settings.
- Developer Mode remains hidden until enabled from Settings.
- Developer workspace is split into Vehicle Developer and Parking/VDGS Developer menus.
- Added clear labels to previously ambiguous controls.
- Removed obsolete player-service UI so only current SSA scenery tools are shown.

## Functional controls
- Hangar selection plus OPEN / CLOSE / TOGGLE controls.
- Jetway selection plus CONNECT / DISCONNECT and automatic mode toggle.
- Parking/VDGS manual selection and return-to-auto control.
- Vehicle route editor controls retained and exposed through Developer Mode.
- VDGS 3D placement controls retained and reorganized into a dedicated developer menu.
- Manual wheel spin and steering test controls retained.

## Notes
- This archive is source code. Build it with the existing GitHub Actions workflow or CMake using the X-Plane SDK.
