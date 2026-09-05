# SSA 0.32.5 - Windows Build Fix

- Fixed MSVC build error `C3861: XPLMDebugString identifier not found`.
- Added the required `#include <XPLMUtilities.h>` to `src/route_editor.cpp`.
- Keeps all v0.32.4 route-save and multi-airport changes unchanged.
- Startup log now reports `SSA 0.32.5 started`.
