#include "ssa/tablet.hpp"
#include <XPLMGraphics.h>
#include <algorithm>
#include <cstdio>

namespace ssa {
namespace {
void label(int x, int y, const char* text, float r = 0.90f, float g = 0.95f, float b = 1.0f) {
  float color[] = {r, g, b};
  XPLMDrawString(color, x, y, const_cast<char*>(text), nullptr, xplmFont_Proportional);
}
}

Tablet::Tablet(SceneryManager& scenery, std::function<void()> toggle_auto,
               std::function<void()> reload_config,
               std::function<void(ServiceObject&)> toggle_object)
    : scenery_(scenery), toggle_auto_(std::move(toggle_auto)),
      reload_config_(std::move(reload_config)), toggle_object_(std::move(toggle_object)) {
  XPLMCreateWindow_t params{};
  params.structSize = sizeof(params);
  params.left = 100; params.top = 700; params.right = 470; params.bottom = 220;
  params.visible = 0;
  params.drawWindowFunc = draw;
  params.handleMouseClickFunc = mouse;
  params.refcon = this;
  params.layer = xplm_WindowLayerFloatingWindows;
  params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
  window_ = XPLMCreateWindowEx(&params);
  XPLMSetWindowTitle(window_, "SSA - Scenery Service Animation");
}

Tablet::~Tablet() { if (window_) XPLMDestroyWindow(window_); }
void Tablet::toggle() { XPLMSetWindowIsVisible(window_, visible() ? 0 : 1); }
bool Tablet::visible() const { return window_ && XPLMGetWindowIsVisible(window_) != 0; }
void Tablet::toggle_developer_mode() {
  developer_mode_ = !developer_mode_;
  if (!developer_mode_ && tab_ == 3) tab_ = 0;
  XPLMSetWindowTitle(window_, developer_mode_
                                 ? "SSA - Scenery Service Animation [DEVELOPER]"
                                 : "SSA - Scenery Service Animation");
}
void Tablet::set_position(double latitude, double longitude) { latitude_ = latitude; longitude_ = longitude; }
void Tablet::draw(XPLMWindowID, void* refcon) { static_cast<Tablet*>(refcon)->draw_impl(); }
int Tablet::mouse(XPLMWindowID, int x, int y, XPLMMouseStatus status, void* refcon) {
  return static_cast<Tablet*>(refcon)->mouse_impl(x, y, status);
}

void Tablet::draw_impl() {
  int l, t, r, b;
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
  label(l + 22, t - 35, "BOLDSTUDIO31  |  SSA", 0.25f, 0.95f, 0.65f);
  const char* tabs = "[ HANGAR ]   JETWAY   BUS";
  if (tab_ == 1) tabs = "HANGAR   [ JETWAY ]   BUS";
  else if (tab_ == 2) tabs = "HANGAR   JETWAY   [ BUS ]";
  else if (tab_ == 3) tabs = "HANGAR   JETWAY   BUS   [ DEV ]";
  label(l + 22, t - 70, tabs);

  if (tab_ == 3 && developer_mode_) {
    label(l + 22, t - 105, "DEVELOPER TOOLS  |  Moving Car Route Editor", 1.0f, 0.72f, 0.20f);
    char coords[180];
    std::snprintf(coords, sizeof(coords), "Aircraft position: %.8f, %.8f", latitude_, longitude_);
    label(l + 22, t - 140, coords);
    char objects[120];
    std::snprintf(objects, sizeof(objects), "Loaded SSA objects: %zu", scenery_.objects().size());
    label(l + 22, t - 175, objects);
    label(l + 22, t - 215, "Route editor will use in-game waypoints (no Blender curve).",
          0.75f, 0.85f, 0.95f);
    label(l + 22, t - 250, "Model stage: prepare bus_root, body, wheels and doors.",
          0.75f, 0.85f, 0.95f);
    label(l + 22, t - 300, "[ RELOAD CONFIG ]", 0.25f, 0.95f, 0.65f);
    label(l + 22, b + 42,
          realops_detected_ ? "Developer Mode | RealOps compatibility active"
                            : "Developer Mode | RealOps not detected",
          1.0f, 0.72f, 0.20f);
    label(r - 96, b + 20, "[ EXIT DEV ]", 1.0f, 0.72f, 0.20f);
    return;
  }
  if (tab_ == 1) {
    label(l + 22, t - 105, automatic_ ? "Automatic jetway: ON" : "Automatic jetway: OFF");
    label(l + 22, t - 130, "Turboprop: 0 | Narrow: 1 | Wide: by forward doors");
  } else if (tab_ == 2) {
    label(l + 22, t - 105, "Apron buses nearby:");
    label(l + 22, t - 130, "Select a bus to start or stop its assigned route.");
  } else {
    label(l + 22, t - 105, "Nearby hangars (maximum 2 km):");
  }
  const ServiceType list_type = tab_ == 0 ? ServiceType::Hangar
                                           : tab_ == 1 ? ServiceType::Jetway
                                                       : ServiceType::Vehicle;
  auto list = scenery_.nearby(list_type, latitude_, longitude_,
                              tab_ == 0 ? 2000.0 : tab_ == 1 ? 35.0 : 2000.0);
  int y = t - 165;
  for (size_t i = 0; i < std::min<size_t>(list.size(), 8); ++i, y -= 34) {
    const auto* object = list[i];
    const char* state = "CLOSED";
    if (tab_ == 0) {
      if (object->progress >= 0.999f) state = "OPEN";
      else if (object->target > object->progress) state = "OPENING";
      else if (object->target < object->progress) state = "CLOSING";
    } else if (tab_ == 2) {
      state = "STOPPED";
      if (object->progress >= 0.999f) state = "RUNNING";
      else if (object->target > object->progress) state = "STARTING";
      else if (object->target < object->progress) state = "STOPPING";
    } else {
      switch (object->jetway_state) {
        case JetwayState::WheelAligning: state = "WHEEL ALIGNING"; break;
        case JetwayState::HeadPreAligning: state = "HEAD 45 DEG"; break;
        case JetwayState::Aligning: state = "ALIGNING"; break;
        case JetwayState::Approaching: state = "APPROACHING"; break;
        case JetwayState::Sealing: state = "SEALING"; break;
        case JetwayState::Connected: state = "CONNECTED"; break;
        case JetwayState::OutOfRange: state = "OUT OF RANGE"; break;
        case JetwayState::Parking: state = "PARKING"; break;
        case JetwayState::Parked: state = "PARKED"; break;
      }
    }
    char line[180];
    if (tab_ == 1 && object->head_error_m >= 0.0f)
      std::snprintf(line, sizeof(line), "%zu. %s  %s  %.0f cm", i + 1,
                    object->label.c_str(), state, object->head_error_m * 100.0f);
    else if (tab_ == 2)
      std::snprintf(line, sizeof(line), "%zu. %s   %s", i + 1,
                    object->label.c_str(), state);
    else
      std::snprintf(line, sizeof(line), "%zu. %s   %s  %3.0f%%", i + 1,
                    object->label.c_str(), state, object->progress * 100.0f);
    label(l + 26, y, line);
  }
  if (list.empty()) label(l + 26, y, "No SSA object found in range.", 1.0f, 0.65f, 0.35f);
  label(l + 22, b + 42, "PLAYER MODE  |  Developer tools hidden", 0.55f, 0.85f, 0.75f);
}

int Tablet::mouse_impl(int x, int y, XPLMMouseStatus status) {
  if (status != xplm_MouseDown) return 1;
  int l, t, r, b;
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
  if (developer_mode_ && tab_ == 3 && y > b + 5 && y < b + 38 && x > r - 130) {
    toggle_developer_mode();
    return 1;
  }
  if (developer_mode_ && tab_ == 3 && y < t - 275 && y > t - 325) {
    reload_config_();
    return 1;
  }
  if (y < t - 45 && y > t - 85) {
    if (developer_mode_ && x > l + 275) tab_ = 3;
    else if (x < l + 105) tab_ = 0;
    else if (x < l + 225) tab_ = 1;
    else tab_ = 2;
    return 1;
  }
  if (tab_ == 3) return 1;
  if (tab_ == 1 && y < t - 85 && y > t - 145) { toggle_auto_(); return 1; }
  const int index = (t - 145 - y) / 34;
  const ServiceType list_type = tab_ == 0 ? ServiceType::Hangar
                                           : tab_ == 1 ? ServiceType::Jetway
                                                       : ServiceType::Vehicle;
  auto list = scenery_.nearby(list_type, latitude_, longitude_,
                              tab_ == 0 ? 2000.0 : tab_ == 1 ? 35.0 : 2000.0);
  if (index >= 0 && static_cast<size_t>(index) < list.size()) toggle_object_(*list[index]);
  return 1;
}

} // namespace ssa
