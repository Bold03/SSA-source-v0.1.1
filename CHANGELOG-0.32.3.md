# SSA 0.32.3 - Multi-Airport Vehicle Loader

## Fixed
- Vehicle route editor no longer uses the first `ssa.json` found in `Custom Scenery`.
- SSA now indexes every scenery containing `vehicle_models` and selects the scenery nearest to the aircraft.
- A valid object latitude/longitude is used as the airport reference; if none exists, SSA falls back to the first valid waypoint in that scenery's `ssa_routes.json`.
- Placeholder coordinates `0,0` are ignored when choosing an airport.
- If the nearest scenery has a broken/missing vehicle OBJ path, SSA tries the next matching scenery instead of disabling vehicles globally.
- On `XPLM_MSG_SCENERY_LOADED` / config reload, vehicle scenery selection runs again using the aircraft's current position, so moving to another airport switches vehicle models/routes automatically.
- Startup and reload logs now show the selected airport/scenery folder.

## Result
`Custom Scenery/Test_SSA/ssa.json` and `Custom Scenery/WIDN Tanjung Pinang/ssa.json` can coexist. When the aircraft is at WIDN, the WIDN vehicle models and `ssa_routes.json` are selected instead of whichever folder happens to be scanned first.
