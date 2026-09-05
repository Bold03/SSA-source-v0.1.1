# SSA 0.30.3 - Stop Test Button

- Vehicle TEST button now changes to red STOP TEST while a route test is running.
- STOP TEST calls RouteEditor::stop_test().
- If the test was launched from the top-down planner, stopping restores the planner automatically.
- Startup log version bumped to 0.30.3.
