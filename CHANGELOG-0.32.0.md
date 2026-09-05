# SSA 0.32.0 - Classic UI Refinement

## UI
- Reduced the default SSA tablet window size so it feels closer to the compact Figma mock-up.
- Rebuilt the main menu as a real circular bubble/radial layout using X-Plane-safe drawing primitives.
- Player mode shows five service bubbles: Hangar, Jetway, Parking, VDGS, and Settings.
- Developer bubble appears only while Developer Mode is enabled.
- Added a dedicated VDGS status screen instead of routing VDGS to the Parking page.
- Cleaned the internal title bar and shortened the native floating-window title to `SSA`.
- Improved spacing on Hangar, Jetway, Parking, Settings, and Developer pages.

## Animation
- Smoother tablet open animation with eased scaling/slide.
- Navigation now triggers a short page pop transition.
- Radial buttons retain the bubble pulse interaction feedback.

## Preserved systems
- Keeps the v0.31.0 altitude-based vehicle visibility system.
- Keeps Developer Test override, Stop Test button, and saved traffic behavior.
