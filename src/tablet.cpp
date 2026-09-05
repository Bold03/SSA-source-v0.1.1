#include "ssa/tablet.hpp"
#include <XPLMGraphics.h>
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

namespace ssa {
namespace {
constexpr float kBlueR = 0.08f;
constexpr float kBlueG = 0.28f;
constexpr float kBlueB = 1.00f;
constexpr float kWhite = 0.92f;

void label(int x, int y, const char* text, float r = kWhite, float g = kWhite,
           float b = kWhite) {
  float color[] = {r, g, b};
  XPLMDrawString(color, x, y, const_cast<char*>(text), nullptr,
                 xplmFont_Proportional);
}

void panel(int left, int top, int right, int bottom) {
  XPLMDrawTranslucentDarkBox(left, top, right, bottom);
}

void classic_button(int left, int top, int right, int bottom, const char* text,
                    bool selected = false, bool danger = false,
                    bool disabled = false) {
  panel(left, top, right, bottom);
  if (disabled) {
    label(left + 8, bottom + 9, text, 0.42f, 0.42f, 0.42f);
  } else if (danger) {
    label(left + 8, bottom + 9, text, 1.0f, 0.22f, 0.18f);
  } else if (selected) {
    label(left + 8, bottom + 9, text, 0.25f, 0.65f, 1.0f);
  } else {
    label(left + 8, bottom + 9, text, kBlueR, kBlueG, kBlueB);
  }
}

void title_bar(int left, int top, int right, const char* title, bool dev_mode) {
  panel(left, top, right, top - 28);
  label(left + 8, top - 19, title, 0.92f, 0.92f, 0.92f);
  if (dev_mode) label(right - 116, top - 19, "DEV MODE", 1.0f, 0.68f, 0.12f);
  classic_button(right - 28, top - 4, right - 4, top - 24, "X", false, true);
}

void footer_back(int left, int bottom, int right, const char* text = "BACK TO SSA MAIN MENU") {
  classic_button(left + 10, bottom + 42, left + 190, bottom + 14, text);
  label(right - 112, bottom + 23, "SSA READY", 0.60f, 0.72f, 0.82f);
}

bool inside(int x, int y, int left, int top, int right, int bottom) {
  return x >= left && x <= right && y <= top && y >= bottom;
}

const char* jetway_state_name(JetwayState state) {
  switch (state) {
    case JetwayState::WheelAligning: return "WHEEL ALIGN";
    case JetwayState::HeadPreAligning: return "HEAD 45 DEG";
    case JetwayState::Aligning: return "ALIGNING";
    case JetwayState::Approaching: return "APPROACHING";
    case JetwayState::Sealing: return "SEALING";
    case JetwayState::Connected: return "CONNECTED";
    case JetwayState::OutOfRange: return "OUT OF RANGE";
    case JetwayState::Parking: return "PARKING";
    case JetwayState::Parked: return "PARKED";
  }
  return "PARKED";
}

const char* vdgs_state_name(VdgsState state) {
  switch (state) {
    case VdgsState::Acquired: return "ACQUIRED";
    case VdgsState::Guiding: return "GUIDING";
    case VdgsState::Slow: return "SLOW";
    case VdgsState::Stop: return "STOP";
    case VdgsState::Overshoot: return "OVERSHOOT";
    case VdgsState::Idle: return "IDLE";
  }
  return "IDLE";
}

ServiceObject* find_by_id(std::vector<ServiceObject*>& list, const std::string& id) {
  const auto it = std::find_if(list.begin(), list.end(), [&](const auto* object) {
    return object && object->id == id;
  });
  return it == list.end() ? nullptr : *it;
}
} // namespace

Tablet::Tablet(SceneryManager& scenery, RouteEditor& route_editor,
               VdgsEditor& vdgs_editor,
               std::function<void()> toggle_auto,
               std::function<void()> reload_config,
               std::function<void(ServiceObject&)> toggle_object,
               std::function<void(ServiceObject*)> select_vdgs,
               std::function<void()> toggle_vehicle_spin,
               std::function<void(float)> set_vehicle_steering)
    : scenery_(scenery), route_editor_(route_editor), vdgs_editor_(vdgs_editor),
      toggle_auto_(std::move(toggle_auto)),
      reload_config_(std::move(reload_config)),
      toggle_object_(std::move(toggle_object)),
      select_vdgs_(std::move(select_vdgs)),
      toggle_vehicle_spin_(std::move(toggle_vehicle_spin)),
      set_vehicle_steering_(std::move(set_vehicle_steering)) {
  XPLMCreateWindow_t params{};
  params.structSize = sizeof(params);
  params.left = 120;
  params.top = 800;
  params.right = 740;
  params.bottom = 240;
  params.visible = 0;
  params.drawWindowFunc = draw;
  params.handleMouseClickFunc = mouse;
  params.refcon = this;
  params.layer = xplm_WindowLayerFloatingWindows;
  params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
  window_ = XPLMCreateWindowEx(&params);
  XPLMSetWindowTitle(window_, "SSA - Scenery Service Animation");
}

Tablet::~Tablet() {
  if (window_) XPLMDestroyWindow(window_);
}

void Tablet::toggle() {
  XPLMSetWindowIsVisible(window_, visible() ? 0 : 1);
}

bool Tablet::visible() const {
  return window_ && XPLMGetWindowIsVisible(window_) != 0;
}

void Tablet::toggle_developer_mode() {
  developer_mode_ = !developer_mode_;
  developer_page_ = 0;
  if (!developer_mode_ && vdgs_editor_.state() == VdgsEditorState::Placing)
    vdgs_editor_.cancel();
  if (!developer_mode_ && tab_ == 4) tab_ = 3;
  XPLMSetWindowTitle(window_, developer_mode_
                                 ? "SSA - Scenery Service Animation [DEVELOPER]"
                                 : "SSA - Scenery Service Animation");
}

void Tablet::set_position(double latitude, double longitude, float heading) {
  latitude_ = latitude;
  longitude_ = longitude;
  heading_ = heading;
}

void Tablet::draw(XPLMWindowID, void* refcon) {
  static_cast<Tablet*>(refcon)->draw_impl();
}

int Tablet::mouse(XPLMWindowID, int x, int y, XPLMMouseStatus status,
                  void* refcon) {
  return static_cast<Tablet*>(refcon)->mouse_impl(x, y, status);
}

void Tablet::draw_impl() {
  int l{}, t{}, r{}, b{};
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
  panel(l + 2, t - 2, r - 2, b + 2);

  const char* title = "SSA MAIN MENU";
  if (tab_ == 0) title = "SSA HANGAR MENU";
  else if (tab_ == 1) title = "SSA JETWAY MENU";
  else if (tab_ == 3) title = "SSA SETTING MENU";
  else if (tab_ == 4) title = developer_page_ == 1 ? "SSA VEHICLE DEVELOPER MENU"
                                                    : developer_page_ == 2
                                                          ? "SSA PARKING DEVELOPER MENU"
                                                          : "SSA DEVELOPER MENU";
  else if (tab_ == 5) title = "SSA PARKING MENU";
  title_bar(l + 4, t - 4, r - 4, title, developer_mode_);

  if (tab_ == 7) {
    const int cx = (l + r) / 2;
    const int cy = (t + b) / 2 + 10;

    label(l + 22, t - 52, "SCENERY SERVICE ANIMATION", 0.62f, 0.62f, 0.62f);
    label(r - 150, t - 52, "X-PLANE CONNECTED", 0.35f, 0.95f, 0.55f);

    // Radial-style service menu inspired by the user's Figma mock-up.
    classic_button(cx - 82, cy + 155, cx + 82, cy + 119, "HANGAR");
    classic_button(cx - 215, cy + 86, cx - 65, cy + 50, "JETWAY");
    classic_button(cx + 65, cy + 86, cx + 215, cy + 50, "VDGS / PARKING");
    classic_button(cx - 215, cy - 14, cx - 65, cy - 50, "SETTINGS");
    if (developer_mode_)
      classic_button(cx + 65, cy - 14, cx + 215, cy - 50, "DEVELOPER", true);
    else
      classic_button(cx + 65, cy - 14, cx + 215, cy - 50,
                     "DEVELOPER LOCKED", false, false, true);
    classic_button(cx - 82, cy - 84, cx + 82, cy - 120, "CLOSE SSA", false, true);

    panel(cx - 58, cy + 36, cx + 58, cy - 36);
    label(cx - 17, cy + 4, "SSA", 0.72f, 0.72f, 0.72f);
    label(cx - 49, cy - 17, developer_mode_ ? "DEVELOPER" : "PLAYER MODE",
          developer_mode_ ? 1.0f : 0.55f,
          developer_mode_ ? 0.68f : 0.75f,
          developer_mode_ ? 0.12f : 0.90f);

    char nearby[160];
    const auto hangars = scenery_.nearby(ServiceType::Hangar, latitude_, longitude_, 2000.0);
    const auto jetways = scenery_.nearby(ServiceType::Jetway, latitude_, longitude_, 1000.0);
    const auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    std::snprintf(nearby, sizeof(nearby), "NEARBY: %zu HANGAR  |  %zu JETWAY  |  %zu VDGS",
                  hangars.size(), jetways.size(), displays.size());
    label(l + 22, b + 28, nearby, 0.60f, 0.72f, 0.82f);
    return;
  }

  if (tab_ == 0) {
    auto hangars = scenery_.nearby(ServiceType::Hangar, latitude_, longitude_, 2000.0);
    if (selected_hangar_id_.empty() && !hangars.empty()) selected_hangar_id_ = hangars.front()->id;
    ServiceObject* selected = find_by_id(hangars, selected_hangar_id_);

    label(l + 20, t - 54, "SELECT HANGAR", 0.72f, 0.72f, 0.72f);
    int y = t - 76;
    const size_t shown = std::min<size_t>(hangars.size(), 5);
    for (size_t i = 0; i < shown; ++i, y -= 42) {
      auto* object = hangars[i];
      char row[160];
      const char* state = "CLOSED";
      if (object->progress >= 0.999f) state = "OPEN";
      else if (object->target > object->progress) state = "OPENING";
      else if (object->target < object->progress) state = "CLOSING";
      std::snprintf(row, sizeof(row), "%zu  %.22s    %s  %.0f%%", i + 1,
                    object->label.c_str(), state, object->progress * 100.0f);
      classic_button(l + 20, y, r - 20, y - 32, row,
                     object->id == selected_hangar_id_);
    }
    if (hangars.empty()) label(l + 20, y - 8, "NO SSA HANGAR FOUND WITHIN 2 KM", 1.0f, 0.55f, 0.35f);

    const int action_top = b + 132;
    classic_button(l + 20, action_top, l + 170, action_top - 34, "OPEN HANGAR");
    classic_button(l + 184, action_top, l + 334, action_top - 34, "CLOSE HANGAR");
    classic_button(l + 348, action_top, l + 498, action_top - 34, "TOGGLE DOOR");
    if (selected) {
      char info[180];
      std::snprintf(info, sizeof(info), "SELECTED: %.28s  |  DATAREF: %.48s",
                    selected->label.c_str(), selected->dataref.c_str());
      label(l + 20, b + 84, info, 0.62f, 0.72f, 0.82f);
    }
    footer_back(l, b, r);
    return;
  }

  if (tab_ == 1) {
    auto jetways = scenery_.nearby(ServiceType::Jetway, latitude_, longitude_, 50.0);
    if (selected_jetway_id_.empty() && !jetways.empty()) selected_jetway_id_ = jetways.front()->id;
    ServiceObject* selected = find_by_id(jetways, selected_jetway_id_);

    label(l + 20, t - 54, automatic_ ? "AUTOMATIC JETWAY: ON" : "AUTOMATIC JETWAY: OFF",
          automatic_ ? 0.35f : 1.0f, automatic_ ? 0.95f : 0.68f,
          automatic_ ? 0.55f : 0.18f);
    int y = t - 78;
    const size_t shown = std::min<size_t>(jetways.size(), 6);
    for (size_t i = 0; i < shown; ++i, y -= 42) {
      auto* object = jetways[i];
      char row[180];
      if (object->head_error_m >= 0.0f)
        std::snprintf(row, sizeof(row), "%zu  %.22s    %s    %.0f CM", i + 1,
                      object->label.c_str(), jetway_state_name(object->jetway_state),
                      object->head_error_m * 100.0f);
      else
        std::snprintf(row, sizeof(row), "%zu  %.22s    %s", i + 1,
                      object->label.c_str(), jetway_state_name(object->jetway_state));
      classic_button(l + 20, y, r - 20, y - 32, row,
                     object->id == selected_jetway_id_);
    }
    if (jetways.empty()) label(l + 20, y - 8, "NO JETWAY FOUND WITHIN 50 M", 1.0f, 0.55f, 0.35f);

    const int action_top = b + 132;
    classic_button(l + 20, action_top, l + 160, action_top - 34, "CONNECT");
    classic_button(l + 174, action_top, l + 334, action_top - 34, "DISCONNECT");
    classic_button(l + 348, action_top, l + 498, action_top - 34,
                   automatic_ ? "AUTO: ON" : "AUTO: OFF", automatic_);
    if (selected) {
      char info[180];
      std::snprintf(info, sizeof(info), "TARGET DOOR: L1  |  STATE: %s",
                    jetway_state_name(selected->jetway_state));
      label(l + 20, b + 84, info, 0.62f, 0.72f, 0.82f);
    }
    footer_back(l, b, r);
    return;
  }

  if (tab_ == 5) {
    auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    const bool manual_gate = std::any_of(displays.begin(), displays.end(),
                                         [](const auto* display) { return display->vdgs_armed; });
    label(l + 20, t - 54, manual_gate ? "PARKING SELECTION: MANUAL" : "PARKING SELECTION: AUTO",
          manual_gate ? 1.0f : 0.35f, manual_gate ? 0.68f : 0.95f,
          manual_gate ? 0.12f : 0.55f);
    int y = t - 78;
    const size_t shown = std::min<size_t>(displays.size(), 6);
    for (size_t i = 0; i < shown; ++i, y -= 42) {
      auto* display = displays[i];
      char row[180];
      const char* state = display->vdgs_armed && !display->vdgs_selected
                              ? "ARMED"
                              : vdgs_state_name(display->vdgs_state);
      if (display->vdgs_selected)
        std::snprintf(row, sizeof(row), "%zu  %.22s    %s   D%+.1fM  L%+.2fM", i + 1,
                      display->label.c_str(), state, display->vdgs_distance_error_m,
                      display->vdgs_lateral_error_m);
      else
        std::snprintf(row, sizeof(row), "%zu  %.22s    %s", i + 1,
                      display->label.c_str(), state);
      classic_button(l + 20, y, r - 20, y - 32, row, display->vdgs_armed);
    }
    if (displays.empty()) label(l + 20, y - 8, "NO VDGS / PARKING FOUND WITHIN 2 KM", 1.0f, 0.55f, 0.35f);

    const int action_top = b + 132;
    classic_button(l + 20, action_top, l + 190, action_top - 34,
                   manual_gate ? "RETURN AUTO" : "AUTO CORRIDOR", !manual_gate);
    label(l + 214, action_top - 22,
          "SELECT A PARKING ROW TO ARM ITS VDGS", 0.62f, 0.72f, 0.82f);
    footer_back(l, b, r);
    return;
  }

  if (tab_ == 3) {
    label(l + 20, t - 58, "GENERAL", 0.72f, 0.72f, 0.72f);
    classic_button(l + 20, t - 78, l + 250, t - 112,
                   automatic_ ? "AUTOMATIC JETWAY: ON" : "AUTOMATIC JETWAY: OFF",
                   automatic_);
    classic_button(l + 20, t - 122, l + 250, t - 156,
                   developer_mode_ ? "DEVELOPER MODE: ON" : "DEVELOPER MODE: OFF",
                   developer_mode_);
    classic_button(l + 20, t - 166, l + 250, t - 200, "RELOAD SSA CONFIG");

    panel(l + 286, t - 78, r - 20, t - 238);
    label(l + 304, t - 102, "SSA SETTINGS", 0.82f, 0.82f, 0.82f);
    label(l + 304, t - 128, "Developer Mode unlocks:", 0.60f, 0.70f, 0.82f);
    label(l + 304, t - 150, "- Vehicle route authoring", 0.60f, 0.70f, 0.82f);
    label(l + 304, t - 172, "- VDGS / parking placement", 0.60f, 0.70f, 0.82f);
    label(l + 304, t - 194, "- Manual test controls", 0.60f, 0.70f, 0.82f);
    label(l + 304, t - 220, "Player UI stays focused on scenery tools.", 0.45f, 0.78f, 0.65f);

    footer_back(l, b, r);
    return;
  }

  if (tab_ == 4 && developer_mode_) {
    if (developer_page_ == 0) {
      label(l + 20, t - 58, "SELECT DEVELOPER TOOL", 1.0f, 0.68f, 0.12f);
      classic_button(l + 20, t - 86, l + 280, t - 128, "VEHICLE DEVELOPER MENU", true);
      classic_button(l + 20, t - 140, l + 280, t - 182, "PARKING / VDGS DEVELOPER", true);
      classic_button(l + 20, t - 194, l + 280, t - 236, "RELOAD SSA CONFIG");
      classic_button(l + 20, t - 248, l + 280, t - 290, "DISABLE DEVELOPER MODE", false, true);

      panel(l + 320, t - 86, r - 20, t - 290);
      label(l + 338, t - 110, "DEVELOPER STATUS", 0.82f, 0.82f, 0.82f);
      label(l + 338, t - 136, "Vehicle route editor", 0.58f, 0.72f, 0.82f);
      label(l + 338, t - 158, route_editor_.status().c_str(), 0.35f, 0.90f, 0.58f);
      label(l + 338, t - 196, "VDGS placement editor", 0.58f, 0.72f, 0.82f);
      label(l + 338, t - 218, vdgs_editor_.status().c_str(), 0.35f, 0.90f, 0.58f);
      label(l + 338, t - 258,
            realops_detected_ ? "RealOps: DETECTED" : "RealOps: NOT DETECTED",
            realops_detected_ ? 0.35f : 0.82f,
            realops_detected_ ? 0.90f : 0.62f,
            realops_detected_ ? 0.58f : 0.30f);
      footer_back(l, b, r);
      return;
    }

    if (developer_page_ == 1) {
      label(l + 20, t - 58, "VEHICLE ROUTE AUTHORING", 1.0f, 0.68f, 0.12f);
      const auto state = route_editor_.state();
      if (state == RouteEditorState::Unavailable) {
        label(l + 20, t - 94, route_editor_.status().c_str(), 1.0f, 0.45f, 0.30f);
        label(l + 20, t - 118, "Add vehicle_models to ssa.json, then reload.", 0.62f, 0.72f, 0.82f);
      } else if (state == RouteEditorState::Idle) {
        classic_button(l + 20, t - 86, l + 240, t - 126, "PLAN NEW VEHICLE ROUTE", true);
        if (route_editor_.saved_route_available()) {
          classic_button(l + 260, t - 86, l + 390, t - 126, "START ALL");
          classic_button(l + 410, t - 86, l + 540, t - 126, "STOP ALL", false, true);
        }
      } else if (state == RouteEditorState::Editing) {
        char route_info[180];
        std::snprintf(route_info, sizeof(route_info), "ANCHORS: %zu  |  LOOP: %s  |  HEADING: %.0f",
                      route_editor_.point_count(), route_editor_.loop_enabled() ? "ON" : "OFF",
                      route_editor_.heading());
        label(l + 20, t - 86, route_info, 0.62f, 0.72f, 0.82f);
        classic_button(l + 20, t - 106, l + 140, t - 140, "LEFT 15");
        classic_button(l + 152, t - 106, l + 284, t - 140, "FORWARD 2M");
        classic_button(l + 296, t - 106, l + 416, t - 140, "RIGHT 15");
        classic_button(l + 20, t - 150, l + 140, t - 184, "BACK 2M");
        classic_button(l + 152, t - 150, l + 284, t - 184, "ADD ANCHOR");
        classic_button(l + 296, t - 150, l + 416, t - 184, "UNDO");
        classic_button(l + 20, t - 194, l + 140, t - 228, "TEST ROUTE", true);
        classic_button(l + 152, t - 194, l + 284, t - 228, "SAVE ROUTE", true);
        classic_button(l + 296, t - 194, l + 416, t - 228, "CANCEL", false, true);
      } else if (state == RouteEditorState::Planning) {
        label(l + 20, t - 92, "TOP-DOWN PLANNER ACTIVE", 1.0f, 0.68f, 0.12f);
        label(l + 20, t - 116, "Add route anchors in the planner overlay.", 0.62f, 0.72f, 0.82f);
      } else if (state == RouteEditorState::Testing) {
        label(l + 20, t - 92, "ROUTE TEST ACTIVE", 1.0f, 0.68f, 0.12f);
        classic_button(l + 20, t - 112, l + 160, t - 148, "STOP TEST", false, true);
      }

      label(l + 20, b + 146, "MANUAL WHEEL TEST", 0.72f, 0.72f, 0.72f);
      classic_button(l + 20, b + 132, l + 160, b + 98,
                     vehicle_spinning_ ? "WHEEL SPIN: ON" : "WHEEL SPIN: OFF",
                     vehicle_spinning_);
      classic_button(l + 174, b + 132, l + 284, b + 98, "STEER LEFT");
      classic_button(l + 296, b + 132, l + 406, b + 98, "CENTER");
      classic_button(l + 418, b + 132, l + 528, b + 98, "STEER RIGHT");
      char steer[80];
      std::snprintf(steer, sizeof(steer), "STEERING DATAREF: %+.2f", vehicle_steering_);
      label(l + 20, b + 76, steer, 0.60f, 0.72f, 0.82f);
      classic_button(l + 20, b + 58, l + 198, b + 28, "BACK TO DEVELOPER");
      return;
    }

    if (developer_page_ == 2) {
      label(l + 20, t - 58, "PARKING / VDGS AUTHORING", 1.0f, 0.68f, 0.12f);
      if (vdgs_editor_.state() == VdgsEditorState::Placing) {
        char placement[220];
        std::snprintf(placement, sizeof(placement),
                      "LAT %.8f  LON %.8f  ALT %.2fM  HDG %.1f",
                      vdgs_editor_.latitude(), vdgs_editor_.longitude(),
                      vdgs_editor_.altitude_m(), vdgs_editor_.heading());
        label(l + 20, t - 86, placement, 0.62f, 0.72f, 0.82f);
        classic_button(l + 20, t - 106, l + 140, t - 140, "LEFT 1M");
        classic_button(l + 152, t - 106, l + 284, t - 140, "FORWARD 1M");
        classic_button(l + 296, t - 106, l + 416, t - 140, "RIGHT 1M");
        classic_button(l + 20, t - 150, l + 140, t - 184, "ALT -0.1M");
        classic_button(l + 152, t - 150, l + 284, t - 184, "BACK 1M");
        classic_button(l + 296, t - 150, l + 416, t - 184, "ALT +0.1M");
        classic_button(l + 20, t - 194, l + 140, t - 228, "ROTATE -5");
        classic_button(l + 152, t - 194, l + 284, t - 228, "SAVE PARKING", true);
        classic_button(l + 296, t - 194, l + 416, t - 228, "ROTATE +5");
        classic_button(l + 20, t - 238, l + 140, t - 272, "RANGE -10");
        classic_button(l + 152, t - 238, l + 284, t - 272, "WIDTH -1");
        classic_button(l + 296, t - 238, l + 416, t - 272, "WIDTH +1");
        classic_button(l + 428, t - 238, l + 548, t - 272, "RANGE +10");
        classic_button(l + 20, t - 282, l + 140, t - 316, "CANCEL", false, true);
        char detection[140];
        std::snprintf(detection, sizeof(detection), "CAPTURE %.0fM  |  CORRIDOR %.0fM",
                      vdgs_editor_.acquisition_distance_m(),
                      vdgs_editor_.corridor_half_width_m() * 2.0f);
        label(l + 162, t - 303, detection, 0.60f, 0.72f, 0.82f);
      } else {
        classic_button(l + 20, t - 86, l + 250, t - 128, "PLACE NEW PARKING / VDGS", true);
        label(l + 20, t - 160, vdgs_editor_.status().c_str(), 0.60f, 0.72f, 0.82f);
        label(l + 20, t - 188, "Capture Point -> Centerline -> Stop Point", 0.60f, 0.72f, 0.82f);
      }
      classic_button(l + 20, b + 58, l + 198, b + 28, "BACK TO DEVELOPER");
      return;
    }
  }

  // Fallback: never expose an inaccessible developer screen.
  tab_ = 7;
}

int Tablet::mouse_impl(int x, int y, XPLMMouseStatus status) {
  if (status != xplm_MouseDown) return 1;
  int l{}, t{}, r{}, b{};
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);

  if (inside(x, y, r - 32, t - 4, r - 4, t - 28)) {
    XPLMSetWindowIsVisible(window_, 0);
    return 1;
  }

  if (tab_ == 7) {
    const int cx = (l + r) / 2;
    const int cy = (t + b) / 2 + 10;
    if (inside(x, y, cx - 82, cy + 155, cx + 82, cy + 119)) tab_ = 0;
    else if (inside(x, y, cx - 215, cy + 86, cx - 65, cy + 50)) tab_ = 1;
    else if (inside(x, y, cx + 65, cy + 86, cx + 215, cy + 50)) tab_ = 5;
    else if (inside(x, y, cx - 215, cy - 14, cx - 65, cy - 50)) tab_ = 3;
    else if (developer_mode_ && inside(x, y, cx + 65, cy - 14, cx + 215, cy - 50)) {
      developer_page_ = 0;
      tab_ = 4;
    } else if (inside(x, y, cx - 82, cy - 84, cx + 82, cy - 120)) {
      XPLMSetWindowIsVisible(window_, 0);
    }
    return 1;
  }

  if (tab_ == 0) {
    auto hangars = scenery_.nearby(ServiceType::Hangar, latitude_, longitude_, 2000.0);
    int row_top = t - 76;
    const size_t shown = std::min<size_t>(hangars.size(), 5);
    for (size_t i = 0; i < shown; ++i, row_top -= 42) {
      if (inside(x, y, l + 20, row_top, r - 20, row_top - 32)) {
        selected_hangar_id_ = hangars[i]->id;
        return 1;
      }
    }
    ServiceObject* selected = find_by_id(hangars, selected_hangar_id_);
    const int action_top = b + 132;
    if (selected && inside(x, y, l + 20, action_top, l + 170, action_top - 34))
      scenery_.set_uniform_target(*selected, 1.0f);
    else if (selected && inside(x, y, l + 184, action_top, l + 334, action_top - 34))
      scenery_.set_uniform_target(*selected, 0.0f);
    else if (selected && inside(x, y, l + 348, action_top, l + 498, action_top - 34))
      toggle_object_(*selected);
    else if (inside(x, y, l + 10, b + 42, l + 190, b + 14))
      tab_ = 7;
    return 1;
  }

  if (tab_ == 1) {
    auto jetways = scenery_.nearby(ServiceType::Jetway, latitude_, longitude_, 50.0);
    int row_top = t - 78;
    const size_t shown = std::min<size_t>(jetways.size(), 6);
    for (size_t i = 0; i < shown; ++i, row_top -= 42) {
      if (inside(x, y, l + 20, row_top, r - 20, row_top - 32)) {
        selected_jetway_id_ = jetways[i]->id;
        return 1;
      }
    }
    ServiceObject* selected = find_by_id(jetways, selected_jetway_id_);
    const int action_top = b + 132;
    if (selected && inside(x, y, l + 20, action_top, l + 160, action_top - 34))
      scenery_.set_uniform_target(*selected, 1.0f);
    else if (selected && inside(x, y, l + 174, action_top, l + 334, action_top - 34))
      scenery_.set_uniform_target(*selected, 0.0f);
    else if (inside(x, y, l + 348, action_top, l + 498, action_top - 34))
      toggle_auto_();
    else if (inside(x, y, l + 10, b + 42, l + 190, b + 14))
      tab_ = 7;
    return 1;
  }

  if (tab_ == 5) {
    auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    int row_top = t - 78;
    const size_t shown = std::min<size_t>(displays.size(), 6);
    for (size_t i = 0; i < shown; ++i, row_top -= 42) {
      if (inside(x, y, l + 20, row_top, r - 20, row_top - 32)) {
        select_vdgs_(displays[i]);
        return 1;
      }
    }
    const int action_top = b + 132;
    if (inside(x, y, l + 20, action_top, l + 190, action_top - 34))
      select_vdgs_(nullptr);
    else if (inside(x, y, l + 10, b + 42, l + 190, b + 14))
      tab_ = 7;
    return 1;
  }

  if (tab_ == 3) {
    if (inside(x, y, l + 20, t - 78, l + 250, t - 112))
      toggle_auto_();
    else if (inside(x, y, l + 20, t - 122, l + 250, t - 156))
      toggle_developer_mode();
    else if (inside(x, y, l + 20, t - 166, l + 250, t - 200))
      reload_config_();
    else if (inside(x, y, l + 10, b + 42, l + 190, b + 14))
      tab_ = 7;
    return 1;
  }

  if (tab_ == 4 && developer_mode_) {
    if (developer_page_ == 0) {
      if (inside(x, y, l + 20, t - 86, l + 280, t - 128)) developer_page_ = 1;
      else if (inside(x, y, l + 20, t - 140, l + 280, t - 182)) developer_page_ = 2;
      else if (inside(x, y, l + 20, t - 194, l + 280, t - 236)) reload_config_();
      else if (inside(x, y, l + 20, t - 248, l + 280, t - 290)) toggle_developer_mode();
      else if (inside(x, y, l + 10, b + 42, l + 190, b + 14)) tab_ = 7;
      return 1;
    }

    if (developer_page_ == 1) {
      const auto state = route_editor_.state();
      if (state == RouteEditorState::Idle) {
        if (inside(x, y, l + 20, t - 86, l + 240, t - 126))
          route_editor_.begin_planner(latitude_, longitude_, heading_);
        else if (route_editor_.saved_route_available() && inside(x, y, l + 260, t - 86, l + 390, t - 126))
          route_editor_.start_all_saved_routes();
        else if (route_editor_.saved_route_available() && inside(x, y, l + 410, t - 86, l + 540, t - 126))
          route_editor_.stop_all_saved_routes();
      } else if (state == RouteEditorState::Editing) {
        if (inside(x, y, l + 20, t - 106, l + 140, t - 140)) route_editor_.turn(-15.0f);
        else if (inside(x, y, l + 152, t - 106, l + 284, t - 140)) route_editor_.move(2.0f);
        else if (inside(x, y, l + 296, t - 106, l + 416, t - 140)) route_editor_.turn(15.0f);
        else if (inside(x, y, l + 20, t - 150, l + 140, t - 184)) route_editor_.move(-2.0f);
        else if (inside(x, y, l + 152, t - 150, l + 284, t - 184)) route_editor_.add_point();
        else if (inside(x, y, l + 296, t - 150, l + 416, t - 184)) route_editor_.undo_point();
        else if (inside(x, y, l + 20, t - 194, l + 140, t - 228)) route_editor_.start_test();
        else if (inside(x, y, l + 152, t - 194, l + 284, t - 228)) route_editor_.save();
        else if (inside(x, y, l + 296, t - 194, l + 416, t - 228)) route_editor_.cancel();
      } else if (state == RouteEditorState::Testing &&
                 inside(x, y, l + 20, t - 112, l + 160, t - 148)) {
        route_editor_.stop_test();
      }

      if (inside(x, y, l + 20, b + 132, l + 160, b + 98)) toggle_vehicle_spin_();
      else if (inside(x, y, l + 174, b + 132, l + 284, b + 98)) {
        vehicle_steering_ = std::max(-1.0f, vehicle_steering_ - 0.25f);
        set_vehicle_steering_(vehicle_steering_);
      } else if (inside(x, y, l + 296, b + 132, l + 406, b + 98)) {
        vehicle_steering_ = 0.0f;
        set_vehicle_steering_(vehicle_steering_);
      } else if (inside(x, y, l + 418, b + 132, l + 528, b + 98)) {
        vehicle_steering_ = std::min(1.0f, vehicle_steering_ + 0.25f);
        set_vehicle_steering_(vehicle_steering_);
      } else if (inside(x, y, l + 20, b + 58, l + 198, b + 28)) {
        developer_page_ = 0;
      }
      return 1;
    }

    if (developer_page_ == 2) {
      if (vdgs_editor_.state() == VdgsEditorState::Placing) {
        if (inside(x, y, l + 20, t - 106, l + 140, t - 140)) vdgs_editor_.move_side(-1.0f);
        else if (inside(x, y, l + 152, t - 106, l + 284, t - 140)) vdgs_editor_.move_forward(1.0f);
        else if (inside(x, y, l + 296, t - 106, l + 416, t - 140)) vdgs_editor_.move_side(1.0f);
        else if (inside(x, y, l + 20, t - 150, l + 140, t - 184)) vdgs_editor_.adjust_altitude(-0.1f);
        else if (inside(x, y, l + 152, t - 150, l + 284, t - 184)) vdgs_editor_.move_forward(-1.0f);
        else if (inside(x, y, l + 296, t - 150, l + 416, t - 184)) vdgs_editor_.adjust_altitude(0.1f);
        else if (inside(x, y, l + 20, t - 194, l + 140, t - 228)) vdgs_editor_.turn(-5.0f);
        else if (inside(x, y, l + 152, t - 194, l + 284, t - 228)) {
          if (vdgs_editor_.save()) reload_config_();
        } else if (inside(x, y, l + 296, t - 194, l + 416, t - 228)) vdgs_editor_.turn(5.0f);
        else if (inside(x, y, l + 20, t - 238, l + 140, t - 272)) vdgs_editor_.adjust_acquisition_distance(-10.0f);
        else if (inside(x, y, l + 152, t - 238, l + 284, t - 272)) vdgs_editor_.adjust_corridor_half_width(-1.0f);
        else if (inside(x, y, l + 296, t - 238, l + 416, t - 272)) vdgs_editor_.adjust_corridor_half_width(1.0f);
        else if (inside(x, y, l + 428, t - 238, l + 548, t - 272)) vdgs_editor_.adjust_acquisition_distance(10.0f);
        else if (inside(x, y, l + 20, t - 282, l + 140, t - 316)) vdgs_editor_.cancel();
      } else if (inside(x, y, l + 20, t - 86, l + 250, t - 128)) {
        vdgs_editor_.begin(latitude_, longitude_, heading_);
      }
      if (inside(x, y, l + 20, b + 58, l + 198, b + 28)) developer_page_ = 0;
      return 1;
    }
  }

  return 1;
}

} // namespace ssa
