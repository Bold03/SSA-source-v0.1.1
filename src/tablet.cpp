#include "ssa/tablet.hpp"
#include <XPLMGraphics.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ssa {
namespace {
void label(int x, int y, const char* text, float r = 0.90f, float g = 0.95f, float b = 1.0f) {
  float color[] = {r, g, b};
  XPLMDrawString(color, x, y, const_cast<char*>(text), nullptr, xplmFont_Proportional);
}
void button(int left, int top, int right, int bottom, const char* text,
            float red = 0.25f, float green = 0.95f, float blue = 0.65f) {
  XPLMDrawTranslucentDarkBox(left, top, right, bottom);
  label(left + 10, bottom + 10, text, red, green, blue);
}
}

Tablet::Tablet(SceneryManager& scenery, RouteEditor& route_editor,
               VdgsEditor& vdgs_editor,
               std::function<void()> toggle_auto,
               std::function<void()> reload_config,
               std::function<void(ServiceObject&)> toggle_object,
               std::function<void()> toggle_vehicle_spin,
               std::function<void(float)> set_vehicle_steering)
    : scenery_(scenery), route_editor_(route_editor), vdgs_editor_(vdgs_editor),
      toggle_auto_(std::move(toggle_auto)),
      reload_config_(std::move(reload_config)), toggle_object_(std::move(toggle_object)),
      toggle_vehicle_spin_(std::move(toggle_vehicle_spin)),
      set_vehicle_steering_(std::move(set_vehicle_steering)) {
  XPLMCreateWindow_t params{};
  params.structSize = sizeof(params);
  params.left = 100; params.top = 700; params.right = 620; params.bottom = 220;
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
  if (!developer_mode_ && vdgs_editor_.state() == VdgsEditorState::Placing)
    vdgs_editor_.cancel();
  if (!developer_mode_ && (tab_ == 4 || tab_ == 2)) tab_ = 3;
  XPLMSetWindowTitle(window_, developer_mode_
                                 ? "SSA - Scenery Service Animation [DEVELOPER]"
                                 : "SSA - Scenery Service Animation");
}
void Tablet::set_position(double latitude, double longitude, float heading) {
  latitude_ = latitude;
  longitude_ = longitude;
  heading_ = heading;
}
void Tablet::draw(XPLMWindowID, void* refcon) { static_cast<Tablet*>(refcon)->draw_impl(); }
int Tablet::mouse(XPLMWindowID, int x, int y, XPLMMouseStatus status, void* refcon) {
  return static_cast<Tablet*>(refcon)->mouse_impl(x, y, status);
}

void Tablet::draw_impl() {
  int l, t, r, b;
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
  label(l + 22, t - 35, "BOLDSTUDIO31  |  SSA", 0.25f, 0.95f, 0.65f);
  const char* tabs = developer_mode_
      ? "[HANGAR]  JETWAY  VDGS  SET  DEV  BUS"
      : "[ HANGAR ]   JETWAY   VDGS   SETTINGS";
  if (tab_ == 1) tabs = developer_mode_
      ? "HANGAR  [JETWAY]  VDGS  SET  DEV  BUS"
      : "HANGAR   [ JETWAY ]   VDGS   SETTINGS";
  else if (tab_ == 5) tabs = developer_mode_
      ? "HANGAR  JETWAY  [VDGS]  SET  DEV  BUS"
      : "HANGAR   JETWAY   [ VDGS ]   SETTINGS";
  else if (tab_ == 2) tabs = "HANGAR  JETWAY  VDGS  SET  DEV  [BUS]";
  else if (tab_ == 3) tabs = developer_mode_
      ? "HANGAR  JETWAY  VDGS  [SET]  DEV  BUS"
      : "HANGAR   JETWAY   VDGS   [ SETTINGS ]";
  else if (tab_ == 4) tabs = "HANGAR  JETWAY  VDGS  SET  [DEV]  BUS";
  label(l + 22, t - 70, tabs);

  if (tab_ == 3) {
    label(l + 22, t - 105, "SETTINGS", 0.25f, 0.95f, 0.65f);
    label(l + 22, t - 145,
          automatic_ ? "Automatic jetway: ON   [ TOGGLE ]"
                     : "Automatic jetway: OFF  [ TOGGLE ]");
    label(l + 22, t - 185,
          developer_mode_ ? "Developer Mode: ON   [ DISABLE ]"
                          : "Developer Mode: OFF  [ ENABLE ]",
          developer_mode_ ? 1.0f : 0.75f, developer_mode_ ? 0.72f : 0.85f,
          developer_mode_ ? 0.20f : 0.95f);
    label(l + 22, t - 220,
          "Developer Mode shows scenery-authoring tools and the Route Editor.",
          0.75f, 0.85f, 0.95f);
    label(l + 22, t - 265, "[ RELOAD CONFIG ]", 0.25f, 0.95f, 0.65f);
    label(l + 22, b + 42, "SSA SETTINGS", 0.55f, 0.85f, 0.75f);
    return;
  }

  if (tab_ == 4 && developer_mode_) {
    if (vdgs_editor_.state() == VdgsEditorState::Placing) {
      label(l + 22, t - 105, "DEVELOPER TOOLS  |  VDGS PLACEMENT", 1.0f, 0.72f, 0.20f);
      char placement[220];
      std::snprintf(placement, sizeof(placement),
                    "Lat %.8f  Lon %.8f  Alt %.2f m  Heading %.1f",
                    vdgs_editor_.latitude(), vdgs_editor_.longitude(),
                    vdgs_editor_.altitude_m(), vdgs_editor_.heading());
      label(l + 22, t - 135, placement, 0.75f, 0.85f, 0.95f);
      button(l + 22, t - 148, l + 152, t - 180, "LEFT 1 M");
      button(l + 172, t - 148, l + 342, t - 180, "FORWARD 1 M");
      button(l + 362, t - 148, r - 22, t - 180, "RIGHT 1 M");
      button(l + 22, t - 190, l + 152, t - 222, "ALT -0.1 M");
      button(l + 172, t - 190, l + 342, t - 222, "BACK 1 M");
      button(l + 362, t - 190, r - 22, t - 222, "ALT +0.1 M");
      button(l + 22, t - 232, l + 152, t - 264, "ROTATE -5");
      button(l + 172, t - 232, l + 342, t - 264, "SAVE VDGS", 1.0f, 0.72f, 0.20f);
      button(l + 362, t - 232, r - 22, t - 264, "ROTATE +5");
      button(l + 22, t - 274, l + 152, t - 306, "CANCEL", 1.0f, 0.72f, 0.20f);
      label(l + 172, t - 292, vdgs_editor_.status().c_str(), 0.75f, 0.85f, 0.95f);
      label(l + 22, b + 42, "VDGS is a live X-Plane instance; SAVE writes ssa.json.",
            1.0f, 0.72f, 0.20f);
      return;
    }
    label(l + 22, t - 105, "DEVELOPER TOOLS  |  ROUTES AND VDGS", 1.0f, 0.72f, 0.20f);
    const auto editor_state = route_editor_.state();
    if (editor_state == RouteEditorState::Unavailable) {
      label(l + 22, t - 145, route_editor_.status().c_str(), 1.0f, 0.45f, 0.35f);
      label(l + 22, t - 190, "Add vehicle_models to ssa.json, then reload.");
      if (vdgs_editor_.state() == VdgsEditorState::Idle) {
        button(l + 265, t - 165, r - 22, t - 205, "PLACE VDGS");
        label(l + 22, t - 230, vdgs_editor_.status().c_str(), 0.55f, 0.85f, 0.75f);
      }
    } else if (editor_state == RouteEditorState::Idle) {
      label(l + 22, t - 145, route_editor_.status().c_str());
      button(l + 22, t - 165, l + 245, t - 205, "PLAN BUS ROUTE");
      button(l + 265, t - 165, r - 22, t - 205, "PLACE VDGS");
      label(l + 22, t - 230, "Click the apron to add automatic Bezier anchors.", 0.75f, 0.85f, 0.95f);
      label(l + 22, t - 252, vdgs_editor_.status().c_str(), 0.55f, 0.85f, 0.75f);
    } else if (editor_state == RouteEditorState::Editing) {
      char route_info[180];
      std::snprintf(route_info, sizeof(route_info),
                    "Bezier anchors: %zu  |  Loop: %s  |  Heading: %.0f deg",
                    route_editor_.point_count(), route_editor_.loop_enabled() ? "ON" : "OFF",
                    route_editor_.heading());
      label(l + 22, t - 135, route_info);
      button(l + 22, t - 148, l + 152, t - 180, "LEFT 15");
      button(l + 172, t - 148, l + 342, t - 180, "FORWARD 2 M");
      button(l + 362, t - 148, r - 22, t - 180, "RIGHT 15");
      button(l + 22, t - 190, l + 152, t - 222, "BACK 2 M");
      button(l + 172, t - 190, l + 342, t - 222, "ADD ANCHOR");
      button(l + 362, t - 190, r - 22, t - 222, "UNDO");
      button(l + 22, t - 232, l + 152, t - 264, "TEST ROUTE", 1.0f, 0.72f, 0.20f);
      button(l + 172, t - 232, l + 342, t - 264, "SAVE ROUTE", 1.0f, 0.72f, 0.20f);
      button(l + 362, t - 232, r - 22, t - 264, "CANCEL", 1.0f, 0.72f, 0.20f);
      label(l + 22, t - 290, route_editor_.status().c_str(), 0.75f, 0.85f, 0.95f);
    } else if (editor_state == RouteEditorState::Planning) {
      label(l + 22, t - 150, "TOP-DOWN PLANNER ACTIVE", 1.0f, 0.72f, 0.20f);
      label(l + 22, t - 195, "Use the overlay to add automatic Bezier anchors.");
    } else {
      char testing[160];
      std::snprintf(testing, sizeof(testing), "TESTING ROUTE  |  Anchors: %zu",
                    route_editor_.point_count());
      label(l + 22, t - 150, testing, 1.0f, 0.72f, 0.20f);
      button(l + 22, t - 170, l + 160, t - 205, "STOP TEST");
      label(l + 22, t - 235, route_editor_.status().c_str());
    }
    label(l + 340, t - 300, "[ RELOAD CONFIG ]", 0.25f, 0.95f, 0.65f);
    label(l + 22, b + 42,
          realops_detected_ ? "Developer Mode | RealOps compatibility active"
                            : "Developer Mode | RealOps not detected",
          1.0f, 0.72f, 0.20f);
    label(r - 96, b + 20, "[ EXIT DEV ]", 1.0f, 0.72f, 0.20f);
    return;
  }
  if (tab_ == 5) {
    label(l + 22, t - 105, "VISUAL DOCKING GUIDANCE SYSTEM", 0.25f, 0.95f, 0.65f);
    auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 250.0);
    int vdgs_y = t - 150;
    for (size_t i = 0; i < std::min<size_t>(displays.size(), 6); ++i, vdgs_y -= 52) {
      const auto* display = displays[i];
      const char* state = "IDLE";
      switch (display->vdgs_state) {
        case VdgsState::Acquired: state = "ACQUIRED"; break;
        case VdgsState::Guiding: state = "GUIDING"; break;
        case VdgsState::Slow: state = "SLOW"; break;
        case VdgsState::Stop: state = "STOP"; break;
        case VdgsState::Overshoot: state = "OVERSHOOT"; break;
        case VdgsState::Idle: state = "IDLE"; break;
      }
      char line[190];
      if (display->vdgs_selected) {
        std::snprintf(line, sizeof(line), "%zu. %s  |  %s", i + 1,
                      display->label.c_str(), state);
        label(l + 22, vdgs_y, line, display->vdgs_state == VdgsState::Stop ? 1.0f : 0.90f,
              display->vdgs_state == VdgsState::Stop ? 0.30f : 0.95f,
              display->vdgs_state == VdgsState::Stop ? 0.20f : 1.0f);
        std::snprintf(line, sizeof(line), "Distance: %+.1f m   Lateral: %+.2f m",
                      display->vdgs_distance_error_m, display->vdgs_lateral_error_m);
        label(l + 38, vdgs_y - 20, line, 0.55f, 0.85f, 0.75f);
      } else {
        std::snprintf(line, sizeof(line), "%zu. %s  |  IDLE", i + 1,
                      display->label.c_str());
        label(l + 22, vdgs_y, line, 0.70f, 0.78f, 0.85f);
      }
    }
    if (displays.empty())
      label(l + 22, vdgs_y, "No VDGS found within 250 m.", 1.0f, 0.65f, 0.35f);
    label(l + 22, b + 42, "VDGS activates automatically during stand approach.",
          0.55f, 0.85f, 0.75f);
    return;
  }
  if (tab_ == 1) {
    label(l + 22, t - 105, automatic_ ? "Automatic jetway: ON" : "Automatic jetway: OFF");
    label(l + 22, t - 130, "Turboprop: 0 | Narrow: 1 | Wide: by forward doors");
  } else if (tab_ == 2) {
    label(l + 22, t - 105, "BACKGROUND BUS TRAFFIC", 0.25f, 0.95f, 0.65f);
    if (!route_editor_.saved_route_available()) {
      label(l + 22, t - 150, "No saved bus route found.", 1.0f, 0.65f, 0.35f);
      label(l + 22, t - 185, "A scenery developer must create and save a route first.",
            0.75f, 0.85f, 0.95f);
    } else {
      button(l + 22, t - 118, l + 180, t - 150, "START ALL");
      button(l + 195, t - 118, l + 353, t - 150, "STOP ALL",
             1.0f, 0.72f, 0.20f);
      int route_y = t - 185;
      const size_t shown = std::min<size_t>(route_editor_.saved_route_count(), 5);
      for (size_t i = 0; i < shown; ++i, route_y -= 44) {
        const auto* route = route_editor_.saved_route(i);
        if (!route) continue;
        const char* traffic_state = route->traffic_blocked
                                        ? "WAITING"
                                        : route->running ? "RUNNING" : "STOPPED";
        char route_line[180];
        std::snprintf(route_line, sizeof(route_line), "%zu. %.13s | %s | x%d | %s | %.0f km/h",
                      i + 1, route->label.c_str(), traffic_state,
                      route->bus_count, route->loop ? "LOOP" : "ONE WAY",
                      route->cruise_speed_mps * 3.6f);
        label(l + 22, route_y, route_line);
        size_t active_count = 0;
        for (const auto& vehicle : route->vehicles)
          if (vehicle.active && vehicle.instance) ++active_count;
        char traffic_detail[120];
        if (route->running && route->spawned_count < route->vehicles.size()) {
          const float due = static_cast<float>(route->spawned_count) *
                            route->spawn_interval_s;
          const float remaining = std::max(0.0f, due - route->spawn_clock);
          if (remaining <= 0.0f) {
            std::snprintf(traffic_detail, sizeof(traffic_detail),
                          "ACTIVE %zu/%d | SPAWN WAIT",
                          active_count, route->bus_count);
          } else {
            std::snprintf(traffic_detail, sizeof(traffic_detail),
                          "ACTIVE %zu/%d | NEXT: %.0f SEC",
                          active_count, route->bus_count, std::ceil(remaining));
          }
        } else {
          std::snprintf(traffic_detail, sizeof(traffic_detail),
                        "ACTIVE %zu/%d | %s", active_count, route->bus_count,
                        route->running ? "ALL BUSES SPAWNED" : "TRAFFIC STOPPED");
        }
        label(l + 38, route_y - 18, traffic_detail, 0.55f, 0.85f, 0.75f);
        button(r - 206, route_y + 12, r - 150, route_y - 16, "EDIT",
               0.30f, 0.80f, 1.0f);
        button(r - 142, route_y + 12, r - 86, route_y - 16, "START");
        button(r - 78, route_y + 12, r - 22, route_y - 16, "STOP",
               1.0f, 0.72f, 0.20f);
      }
      if (route_editor_.saved_route_count() > shown)
        label(l + 22, route_y, "More routes are running; showing the first five.",
              0.75f, 0.85f, 0.95f);
      label(l + 22, b + 68, route_editor_.status().c_str(), 0.75f, 0.85f, 0.95f);
    }
    label(l + 22, b + 42, "DEVELOPER | BACKGROUND TRAFFIC", 0.55f, 0.85f, 0.75f);
    return;
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
  label(l + 22, b + 42, "SSA READY", 0.55f, 0.85f, 0.75f);
}

int Tablet::mouse_impl(int x, int y, XPLMMouseStatus status) {
  if (status != xplm_MouseDown) return 1;
  int l, t, r, b;
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
  if (developer_mode_ && tab_ == 4 && y > b + 5 && y < b + 38 && x > r - 130) {
    toggle_developer_mode();
    return 1;
  }
  if (developer_mode_ && tab_ == 4 && y < t - 280 && y > t - 325 && x > l + 315) {
    reload_config_();
    return 1;
  }
  if (y < t - 45 && y > t - 85) {
    if (developer_mode_) {
      if (x < l + 82) tab_ = 0;
      else if (x < l + 168) tab_ = 1;
      else if (x < l + 245) tab_ = 5;
      else if (x < l + 325) tab_ = 3;
      else if (x < l + 405) tab_ = 4;
      else tab_ = 2;
    } else {
      if (x < l + 112) tab_ = 0;
      else if (x < l + 220) tab_ = 1;
      else if (x < l + 318) tab_ = 5;
      else tab_ = 3;
    }
    return 1;
  }
  if (tab_ == 3) {
    if (y < t - 120 && y > t - 165) toggle_auto_();
    else if (y < t - 165 && y > t - 205) toggle_developer_mode();
    else if (y < t - 235 && y > t - 285) reload_config_();
    return 1;
  }
  if (tab_ == 4) {
    if (vdgs_editor_.state() == VdgsEditorState::Placing) {
      const bool column_left = x >= l + 22 && x <= l + 152;
      const bool column_middle = x >= l + 172 && x <= l + 342;
      const bool column_right = x >= l + 362 && x <= r - 22;
      if (y <= t - 148 && y >= t - 180) {
        if (column_left) vdgs_editor_.move_side(-1.0f);
        else if (column_middle) vdgs_editor_.move_forward(1.0f);
        else if (column_right) vdgs_editor_.move_side(1.0f);
      } else if (y <= t - 190 && y >= t - 222) {
        if (column_left) vdgs_editor_.adjust_altitude(-0.1f);
        else if (column_middle) vdgs_editor_.move_forward(-1.0f);
        else if (column_right) vdgs_editor_.adjust_altitude(0.1f);
      } else if (y <= t - 232 && y >= t - 264) {
        if (column_left) vdgs_editor_.turn(-5.0f);
        else if (column_middle) {
          if (vdgs_editor_.save()) reload_config_();
        } else if (column_right) vdgs_editor_.turn(5.0f);
      } else if (column_left && y <= t - 274 && y >= t - 306) {
        vdgs_editor_.cancel();
      }
      return 1;
    }
    const auto editor_state = route_editor_.state();
    if (editor_state == RouteEditorState::Idle && x >= l + 22 && x <= l + 245 &&
        y <= t - 165 && y >= t - 205) {
      route_editor_.begin_planner(latitude_, longitude_, heading_);
    } else if ((editor_state == RouteEditorState::Idle ||
                editor_state == RouteEditorState::Unavailable) &&
               vdgs_editor_.state() == VdgsEditorState::Idle &&
               x >= l + 265 && x <= r - 22 &&
               y <= t - 165 && y >= t - 205) {
      vdgs_editor_.begin(latitude_, longitude_, heading_);
    } else if (editor_state == RouteEditorState::Editing) {
      const bool column_left = x >= l + 22 && x <= l + 152;
      const bool column_middle = x >= l + 172 && x <= l + 342;
      const bool column_right = x >= l + 362 && x <= r - 22;
      if (y <= t - 148 && y >= t - 180) {
        if (column_left) route_editor_.turn(-15.0f);
        else if (column_middle) route_editor_.move(2.0f);
        else if (column_right) route_editor_.turn(15.0f);
      } else if (y <= t - 190 && y >= t - 222) {
        if (column_left) route_editor_.move(-2.0f);
        else if (column_middle) route_editor_.add_point();
        else if (column_right) route_editor_.undo_point();
      } else if (y <= t - 232 && y >= t - 264) {
        if (column_left) route_editor_.start_test();
        else if (column_middle) route_editor_.save();
        else if (column_right) route_editor_.cancel();
      }
    } else if (editor_state == RouteEditorState::Testing && x >= l + 22 &&
               x <= l + 160 && y <= t - 170 && y >= t - 205) {
      route_editor_.stop_test();
    }
    return 1;
  }
  if (tab_ == 1 && y < t - 85 && y > t - 145) { toggle_auto_(); return 1; }
  if (tab_ == 2) {
    if (y <= t - 118 && y >= t - 150) {
      if (x >= l + 22 && x <= l + 180) route_editor_.start_all_saved_routes();
      else if (x >= l + 195 && x <= l + 353) route_editor_.stop_all_saved_routes();
      return 1;
    }
    int route_y = t - 185;
    const size_t shown = std::min<size_t>(route_editor_.saved_route_count(), 5);
    for (size_t i = 0; i < shown; ++i, route_y -= 44) {
      if (y <= route_y + 12 && y >= route_y - 16) {
        if (x >= r - 206 && x <= r - 150) route_editor_.edit_saved_route(i);
        else if (x >= r - 142 && x <= r - 86) route_editor_.start_saved_route(i);
        else if (x >= r - 78 && x <= r - 22) route_editor_.stop_saved_route(i);
        return 1;
      }
    }
    return 1;
  }
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
