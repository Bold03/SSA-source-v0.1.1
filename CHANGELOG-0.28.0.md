# SSA 0.28.0

- Added offline `tools/SSA-JSON-Builder.html` for creating/importing/validating/downloading `ssa.json`.
- Added top-level `vehicle_presence_radius_m` (default 8000 m).
- Background SSA apron vehicles are automatically hidden when the aircraft leaves the configured airport area.
- Vehicle visibility uses geographic latitude/longitude distance so long flights and X-Plane local-origin shifts do not make airport vehicles follow the aircraft.
- Vehicles resume rendering when returning inside the airport presence radius.
