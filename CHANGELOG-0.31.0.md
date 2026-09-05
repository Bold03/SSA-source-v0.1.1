# SSA 0.31.0 - Altitude Vehicle Visibility

- Replaced the old airport-distance vehicle visibility cutoff with an aircraft AGL altitude gate.
- Saved/autostart SSA vehicles remain active below `vehicle_hide_agl_ft`.
- Default hide altitude is 5,000 ft AGL. Set `vehicle_hide_agl_ft` to 10000 in `ssa.json` if 10,000 ft is preferred.
- Vehicles are hidden immediately when the aircraft climbs above the configured altitude and become available again when descending below it.
- Developer route TEST remains active regardless of altitude.
- JSON Builder updated with the new altitude setting.
