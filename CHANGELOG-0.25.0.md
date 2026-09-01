# SSA 0.25.0

## Implemented in this iteration

- Speaker deletion is no longer "delete nearest" by default in the tablet workflow.
- The Announcement developer page loads saved speakers, lets the developer select one, and requires a second confirmation click before deleting it from `ssa.json`.
- Added listener environment states for 3D announcements:
  - cockpit: 0.35x source gain
  - outside: 1.0x source gain
  - terminal zone: configurable gain multiplier (default example 1.35x)
- Added `audio_zones` JSON support for terminal listener areas.
- Added `sim/graphics/view/view_is_external` detection for cockpit/external switching.

## Next implementation block

- User-level SimBrief settings and HTTP OFP fetch/cache.
- Announcement feature gate until SimBrief Pilot ID or username is configured.
- Flight phase trigger state machine using the SimBrief OFP.
- Unified Developer workspace shortcuts (Undo/Redo/Move/Rotate/Delete/Focus/Escape).
- Explicit VDGS capture/stop checkpoints and jetway pivot/door-target checkpoints in the authoring workflow.
