# SSA 0.26.0

## Airbus-style tablet UI integration

- Reworked the native X-Plane SSA window into a 1024x720 Airbus-inspired tablet layout.
- Added an SSA Home dashboard as the default screen.
- Added persistent bottom navigation: Home, Hangar, Jetway, VDGS, Announce, Settings, Developer.
- Added X-Plane connection status and listener/audio status presentation.
- Added service cards matching the approved Figma direction.
- Kept existing service, route, VDGS, speaker, announcement and scenery logic wired to the same C++ systems.
- Announcement page is now reachable from the player tablet navigation; SimBrief gating remains planned for the next logic pass.
- Developer entry from the tablet navigation automatically enables Developer Mode.

### Rendering note
The Figma design is translated to X-Plane native drawing primitives rather than embedded as HTML, so it remains compatible with the existing XPLM floating-window architecture.
