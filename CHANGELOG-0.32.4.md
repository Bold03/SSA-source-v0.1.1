# SSA 0.32.4 - Route Save Path Fix

- Vehicle Developer SAVE now works while the top-down planner is still active (`Planning` state).
- Saving automatically closes the planner before serializing the route.
- `ssa_routes.json` is created directly in the selected SSA scenery folder.
- Route writer now creates the parent directory if needed, flushes the file, and logs the exact saved path.
- Failed saves now write an explicit `[SSA] Route save FAILED:` message to `Log.txt`.
