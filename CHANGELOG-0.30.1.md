# SSA 0.30.1 - Vehicle Test Fix

- Fixed Vehicle Developer `TEST` button doing nothing while the route planner was still in `Planning` state.
- TEST now accepts both `Editing` and `Planning` route-editor states.
- Starting a test from the top-down planner now closes the planner overlay/camera control while the vehicle test runs.
- The planner is restored automatically after the route test completes, preserving the previous workflow.
