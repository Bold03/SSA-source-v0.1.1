# SSA 0.32.2 - Saved Route Manager

- Added a saved route list to Vehicle Developer Menu.
- Added SELECT, EDIT, START, STOP, and DELETE ROUTE actions.
- DELETE ROUTE requires a second confirmation click.
- Deleting a route removes only that route from `ssa_routes.json`.
- Remaining autostart routes are reloaded and restarted automatically.
- Replaced the old misleading Vehicle Developer `DELETE` action that only cancelled editing.
