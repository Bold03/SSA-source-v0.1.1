#include "ssa/tablet.hpp"
#include <XPLMGraphics.h>
#include <XPLMProcessing.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace ssa {
namespace {
constexpr float kBlueR = 0.10f;
constexpr float kBlueG = 0.30f;
constexpr float kBlueB = 1.00f;
constexpr float kText = 0.93f;
constexpr float kSoft = 0.70f;

float now_sec() { return XPLMGetElapsedTime(); }

int lerpi(int a, int b, float p) {
  return static_cast<int>(std::lround(a + (b - a) * p));
}

void label(int x, int y, const char* text, float r = kText, float g = kText,
           float b = kText) {
  float color[] = {r, g, b};
  XPLMDrawString(color, x, y, const_cast<char*>(text), nullptr,
                 xplmFont_Proportional);
}

void panel(int left, int top, int right, int bottom) {
  XPLMDrawTranslucentDarkBox(left, top, right, bottom);
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
  const bool show = !visible();
  if (show) {
    open_anim_ = 0.0f;
    last_draw_time_ = now_sec();
  }
  XPLMSetWindowIsVisible(window_, show ? 1 : 0);
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

void Tablet::pulse(const char* id) {
  pulse_button_id_ = id ? id : "";
  pulse_start_time_ = now_sec();
}

void Tablet::draw(XPLMWindowID, void* refcon) {
  static_cast<Tablet*>(refcon)->draw_impl();
}

int Tablet::mouse(XPLMWindowID, int x, int y, XPLMMouseStatus status,
                  void* refcon) {
  return static_cast<Tablet*>(refcon)->mouse_impl(x, y, status);
}

void Tablet::draw_impl() {
  const float now = now_sec();
  if (last_draw_time_ < 0.0f) last_draw_time_ = now;
  const float dt = std::clamp(now - last_draw_time_, 0.0f, 0.10f);
  last_draw_time_ = now;
  open_anim_ = std::min(1.0f, open_anim_ + dt * 4.0f);
  const float appear = open_anim_;
  const float pulse_p = std::clamp((now - pulse_start_time_) / 0.24f, 0.0f, 1.0f);

  int l{}, t{}, r{}, b{};
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
  const int x_inset = static_cast<int>((1.0f - appear) * 32.0f);
  const int y_drop = static_cast<int>((1.0f - appear) * 18.0f);
  l += x_inset;
  r -= x_inset;
  t -= y_drop;
  b += static_cast<int>((1.0f - appear) * 10.0f);

  panel(l + 2, t - 2, r - 2, b + 2);
  panel(l + 10, t - 34, r - 10, b + 10);
  panel(l + 10, t - 4, r - 10, t - 28);
  label((l + r) / 2 - 128, t - 20, "SSA - Scenery Service Animation", 0.62f, 0.64f, 0.68f);
  label(l + 22, t - 51, "SSA MAIN MENU", 0.94f, 0.94f, 0.94f);
  panel(r - 34, t - 10, r - 12, t - 28);
  label(r - 28, t - 22, "X", 1.0f, 0.22f, 0.22f);

  auto button = [&](int left, int top, int right, int bottom, const char* text,
                    const char* id, bool blue = false, bool danger = false,
                    bool disabled = false) {
    int pad = 0;
    if (!pulse_button_id_.empty() && pulse_button_id_ == id && pulse_p < 1.0f) {
      const float s = std::sin(pulse_p * 3.1415926f);
      pad = static_cast<int>(std::lround(4.0f * s));
    }
    panel(left - pad, top + pad, right + pad, bottom - pad);
    if (disabled) label(left + 8, bottom + 9, text, 0.45f, 0.45f, 0.45f);
    else if (danger) label(left + 8, bottom + 9, text, 1.0f, 0.20f, 0.20f);
    else if (blue) label(left + 8, bottom + 9, text, kBlueR, kBlueG, kBlueB);
    else label(left + 8, bottom + 9, text, 0.78f, 0.78f, 0.78f);
  };

  auto footer_back = [&](const char* text = "BACK TO SSA MAIN MENU") {
    button(l + 18, b + 54, l + 148, b + 30, text, "back", true);
  };

  auto title_for_tab = [&]() -> const char* {
    if (tab_ == 0) return "SSA HANGAR MENU";
    if (tab_ == 1) return "SSA JETWAY MENU";
    if (tab_ == 3) return "SSA SETTING MENU";
    if (tab_ == 4) return developer_page_ == 1 ? "SSA VEHICLE DEVELOPER MENU"
                          : developer_page_ == 2 ? "SSA PARKING DEVELOPER MENU"
                                                : "SSA DEVELOPER MENU";
    if (tab_ == 5) return "SSA PARKING MENU";
    return "SSA MAIN MENU";
  };
  label(l + 22, t - 20, title_for_tab(), 0.94f, 0.94f, 0.94f);

  // MAIN / RADIAL MENU
  if (tab_ == 7) {
    label(l + 34, t - 80, "SCENERY SERVICE ANIMATION", 0.68f, 0.68f, 0.68f);
    label(r - 160, t - 80, "X-PLANE CONNECTED", 0.35f, 0.95f, 0.55f);

    const int cx = (l + r) / 2 + 2;
    const int cy = (t + b) / 2 + 22;

    panel(cx - 64, cy + 64, cx + 64, cy - 64);
    label(cx - 18, cy + 6, "SSA", 0.70f, 0.70f, 0.70f);
    label(cx - 24, cy - 16, "CLOSE", 0.70f, 0.70f, 0.70f);

    button(cx - 40, cy + 112, cx + 40, cy + 88, "HANGAR", "main_hangar", true);
    button(cx - 132, cy + 62, cx - 60, cy + 38, "JETWAY", "main_jetway", true);
    button(cx + 60, cy + 62, cx + 144, cy + 38, "PARKING", "main_parking", true);
    button(cx - 132, cy - 14, cx - 60, cy - 38, "SETTINGS", "main_settings", true);
    button(cx + 60, cy - 14, cx + 144, cy - 38,
           developer_mode_ ? "DEVELOPER" : "LOCKED",
           "main_dev", developer_mode_, false, !developer_mode_);
    button(cx - 40, cy - 64, cx + 40, cy - 88, "VDGS", "main_vdgs", true);

    char nearby[160];
    const auto hangars = scenery_.nearby(ServiceType::Hangar, latitude_, longitude_, 2000.0);
    const auto jetways = scenery_.nearby(ServiceType::Jetway, latitude_, longitude_, 1000.0);
    const auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    std::snprintf(nearby, sizeof(nearby), "NEARBY: %zu HANGAR  |  %zu JETWAY  |  %zu VDGS",
                  hangars.size(), jetways.size(), displays.size());
    label(l + 34, b + 28, nearby, 0.60f, 0.72f, 0.82f);
    label(l + 34, b + 12, developer_mode_ ? "DEVELOPER MODE" : "PLAYER MODE",
          developer_mode_ ? 1.0f : 0.60f,
          developer_mode_ ? 0.68f : 0.72f,
          developer_mode_ ? 0.12f : 0.82f);
    return;
  }

  // HANGAR MENU
  if (tab_ == 0) {
    auto hangars = scenery_.nearby(ServiceType::Hangar, latitude_, longitude_, 2000.0);
    if (selected_hangar_id_.empty() && !hangars.empty()) selected_hangar_id_ = hangars.front()->id;
    ServiceObject* selected = find_by_id(hangars, selected_hangar_id_);

    int y = t - 74;
    const size_t shown = std::min<size_t>(hangars.size(), 5);
    for (size_t i = 0; i < shown; ++i, y -= 30) {
      auto* object = hangars[i];
      char row[96];
      std::snprintf(row, sizeof(row), "%zu. %s", i + 1, object->label.c_str());
      button(l + 36, y, l + 176, y - 24, row,
             object->id.c_str(), true);
    }
    if (hangars.empty()) label(l + 36, y - 8, "NO SSA HANGAR FOUND WITHIN 2 KM", 1.0f, 0.55f, 0.35f);

    button(l + 38, b + 122, l + 98, b + 98, "OPEN", "hang_open");
    button(l + 38, b + 92, l + 98, b + 68, "CLOSE", "hang_close");
    button(l + 38, b + 62, l + 98, b + 38, "TOGGLE", "hang_toggle");
    footer_back();

    if (selected) {
      const char* state = "CLOSED";
      if (selected->progress >= 0.999f) state = "OPEN";
      else if (selected->target > selected->progress) state = "OPENING";
      else if (selected->target < selected->progress) state = "CLOSING";
      char info1[128];
      char info2[128];
      std::snprintf(info1, sizeof(info1), "SELECTED: %s", selected->label.c_str());
      std::snprintf(info2, sizeof(info2), "STATUS: %s  %.0f%%", state, selected->progress * 100.0f);
      label(l + 220, b + 110, info1, 0.78f, 0.78f, 0.78f);
      label(l + 220, b + 88, info2, kBlueR, kBlueG, kBlueB);
    }
    return;
  }

  // JETWAY MENU
  if (tab_ == 1) {
    auto jetways = scenery_.nearby(ServiceType::Jetway, latitude_, longitude_, 50.0);
    if (selected_jetway_id_.empty() && !jetways.empty()) selected_jetway_id_ = jetways.front()->id;
    ServiceObject* selected = find_by_id(jetways, selected_jetway_id_);

    int y = t - 74;
    const size_t shown = std::min<size_t>(jetways.size(), 5);
    for (size_t i = 0; i < shown; ++i, y -= 30) {
      auto* object = jetways[i];
      char row[96];
      std::snprintf(row, sizeof(row), "%zu. %s", i + 1, object->label.c_str());
      button(l + 36, y, l + 176, y - 24, row, object->id.c_str(), true);
    }
    button(l + 38, b + 122, l + 112, b + 98, "CONNECT", "jet_connect");
    button(l + 38, b + 92, l + 112, b + 68, "DISCONNECT", "jet_disconnect");
    button(l + 38, b + 62, l + 112, b + 38, automatic_ ? "AUTO ON" : "AUTO OFF", "jet_auto", true);
    footer_back();
    if (selected) {
      char info1[128];
      char info2[128];
      std::snprintf(info1, sizeof(info1), "SELECTED: %s", selected->label.c_str());
      std::snprintf(info2, sizeof(info2), "STATE: %s", jetway_state_name(selected->jetway_state));
      label(l + 220, b + 110, info1, 0.78f, 0.78f, 0.78f);
      label(l + 220, b + 88, info2, kBlueR, kBlueG, kBlueB);
    }
    return;
  }

  // PARKING / VDGS MENU
  if (tab_ == 5) {
    auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    int y = t - 74;
    const size_t shown = std::min<size_t>(displays.size(), 5);
    for (size_t i = 0; i < shown; ++i, y -= 30) {
      auto* display = displays[i];
      char row[96];
      std::snprintf(row, sizeof(row), "%zu. %s", i + 1, display->label.c_str());
      button(l + 36, y, l + 176, y - 24, row, display->id.c_str(), true);
    }
    button(l + 38, b + 122, l + 128, b + 98, "ARM VDGS", "park_arm");
    button(l + 38, b + 92, l + 128, b + 68, "AUTO MODE", "park_auto");
    button(l + 38, b + 62, l + 128, b + 38, "CLEAR", "park_clear");
    footer_back();
    if (!displays.empty()) {
      auto* sel = displays.front();
      char info1[128];
      char info2[128];
      std::snprintf(info1, sizeof(info1), "PARKING: %s", sel->label.c_str());
      std::snprintf(info2, sizeof(info2), "STATUS: %s", vdgs_state_name(sel->vdgs_state));
      label(l + 220, b + 110, info1, 0.78f, 0.78f, 0.78f);
      label(l + 220, b + 88, info2, kBlueR, kBlueG, kBlueB);
    }
    return;
  }

  // SETTINGS
  if (tab_ == 3) {
    button(l + 36, t - 74, l + 106, t - 98, "GENERAL", "set_general", true);
    button(l + 36, t - 104, l + 106, t - 128, "VEHICLE", "set_vehicle", true);
    button(l + 36, t - 134, l + 106, t - 158, "HANGAR", "set_hangar", true);
    button(l + 36, t - 164, l + 106, t - 188, "PARKING", "set_parking", true);
    button(l + 36, t - 194, l + 106, t - 218, "VDGS", "set_vdgs", true);
    button(l + 36, b + 108, l + 106, b + 84,
           automatic_ ? "AUTO ON" : "AUTO OFF", "set_auto");
    button(l + 36, b + 78, l + 106, b + 54,
           developer_mode_ ? "DEV ON" : "DEV OFF", "set_dev");
    button(l + 36, b + 48, l + 106, b + 24, "RELOAD", "set_reload", true);
    footer_back("BACK TO SSA SETTINGS");
    label(l + 180, t - 76, "Developer Mode unlocks scenery authoring tools.", 0.74f, 0.74f, 0.74f);
    label(l + 180, t - 98, "Player mode keeps the tablet simple.", 0.74f, 0.74f, 0.74f);
    return;
  }

  // DEVELOPER
  if (tab_ == 4 && developer_mode_) {
    if (developer_page_ == 0) {
      button(l + 36, t - 74, l + 160, t - 98, "VEHICLE MENU", "dev_vehicle", true);
      button(l + 36, t - 104, l + 160, t - 128, "PARKING MENU", "dev_parking", true);
      button(l + 36, t - 134, l + 160, t - 158, "RELOAD CONFIG", "dev_reload", true);
      button(l + 36, t - 164, l + 160, t - 188, "EXIT DEV MODE", "dev_exit");
      label(l + 220, t - 76, "DEVELOPER STATUS", 0.84f, 0.84f, 0.84f);
      label(l + 220, t - 100, route_editor_.status().c_str(), 0.68f, 0.68f, 0.68f);
      label(l + 220, t - 122, vdgs_editor_.status().c_str(), 0.68f, 0.68f, 0.68f);
      footer_back("BACK TO SSA SETTINGS");
      return;
    }

    if (developer_page_ == 1) {
      button(l + 36, t - 74, l + 132, t - 98, "PLAN NEW ROUTE", "veh_plan", true);
      button(l + 36, t - 104, l + 102, t - 128, "TEST", "veh_test");
      button(l + 36, t - 134, l + 102, t - 158, "SAVE", "veh_save");
      button(l + 36, t - 164, l + 132, t - 188, "BACK TO DEV", "veh_back", true);
      int fy = t - 74;
      const char* fields[] = {"ROUTE", "MODEL", "SPEED", "LOOP", "HEADING", "STATUS"};
      for (int i = 0; i < 6; ++i, fy -= 30) button(l + 220, fy, l + 318, fy - 24, fields[i], fields[i]);
      button(l + 220, b + 82, l + 278, b + 58, "TEST SPIN", "veh_spin");
      button(l + 220, b + 52, l + 278, b + 28, "DELETE", "veh_delete", false, true);
      return;
    }

    if (developer_page_ == 2) {
      button(l + 36, t - 74, l + 132, t - 98, "PLACE NEW VDGS", "park_new", true);
      button(l + 36, t - 104, l + 112, t - 128, "RELOAD SNAP", "park_snap");
      button(l + 36, b + 82, l + 104, b + 58, "DELETE", "park_delete", false, true);
      button(l + 36, b + 52, l + 132, b + 28, "BACK TO DEV", "park_back", true);
      int fy = t - 74;
      const char* fields[] = {"PARK ID", "STAND", "LAT", "LON", "HEADING", "RANGE"};
      for (int i = 0; i < 6; ++i, fy -= 30) button(l + 220, fy, l + 318, fy - 24, fields[i], fields[i]);
      return;
    }
  }

  tab_ = 7;
}

int Tablet::mouse_impl(int x, int y, XPLMMouseStatus status) {
  if (status != xplm_MouseDown) return 1;
  if (open_anim_ < 0.95f) return 1;

  int l{}, t{}, r{}, b{};
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);

  if (inside(x, y, r - 34, t - 10, r - 12, t - 28)) {
    pulse("close");
    XPLMSetWindowIsVisible(window_, 0);
    return 1;
  }

  if (tab_ == 7) {
    const int cx = (l + r) / 2 + 2;
    const int cy = (t + b) / 2 + 22;
    if (inside(x, y, cx - 40, cy + 112, cx + 40, cy + 88)) { pulse("main_hangar"); tab_ = 0; }
    else if (inside(x, y, cx - 132, cy + 62, cx - 60, cy + 38)) { pulse("main_jetway"); tab_ = 1; }
    else if (inside(x, y, cx + 60, cy + 62, cx + 144, cy + 38)) { pulse("main_parking"); tab_ = 5; }
    else if (inside(x, y, cx - 132, cy - 14, cx - 60, cy - 38)) { pulse("main_settings"); tab_ = 3; }
    else if (inside(x, y, cx - 40, cy - 64, cx + 40, cy - 88)) { pulse("main_vdgs"); tab_ = 5; }
    else if (developer_mode_ && inside(x, y, cx + 60, cy - 14, cx + 144, cy - 38)) {
      pulse("main_dev"); developer_page_ = 0; tab_ = 4;
    } else if (inside(x, y, cx - 64, cy + 64, cx + 64, cy - 64)) {
      XPLMSetWindowIsVisible(window_, 0);
    }
    return 1;
  }

  if (tab_ == 0) {
    auto hangars = scenery_.nearby(ServiceType::Hangar, latitude_, longitude_, 2000.0);
    int row_top = t - 74;
    const size_t shown = std::min<size_t>(hangars.size(), 5);
    for (size_t i = 0; i < shown; ++i, row_top -= 30) {
      if (inside(x, y, l + 36, row_top, l + 176, row_top - 24)) {
        selected_hangar_id_ = hangars[i]->id;
        pulse(hangars[i]->id.c_str());
        return 1;
      }
    }
    ServiceObject* selected = find_by_id(hangars, selected_hangar_id_);
    if (selected && inside(x, y, l + 38, b + 122, l + 98, b + 98)) { pulse("hang_open"); scenery_.set_uniform_target(*selected, 1.0f); }
    else if (selected && inside(x, y, l + 38, b + 92, l + 98, b + 68)) { pulse("hang_close"); scenery_.set_uniform_target(*selected, 0.0f); }
    else if (selected && inside(x, y, l + 38, b + 62, l + 98, b + 38)) { pulse("hang_toggle"); toggle_object_(*selected); }
    else if (inside(x, y, l + 18, b + 54, l + 148, b + 30)) { pulse("back"); tab_ = 7; }
    return 1;
  }

  if (tab_ == 1) {
    auto jetways = scenery_.nearby(ServiceType::Jetway, latitude_, longitude_, 50.0);
    int row_top = t - 74;
    const size_t shown = std::min<size_t>(jetways.size(), 5);
    for (size_t i = 0; i < shown; ++i, row_top -= 30) {
      if (inside(x, y, l + 36, row_top, l + 176, row_top - 24)) {
        selected_jetway_id_ = jetways[i]->id;
        pulse(jetways[i]->id.c_str());
        return 1;
      }
    }
    ServiceObject* selected = find_by_id(jetways, selected_jetway_id_);
    if (selected && inside(x, y, l + 38, b + 122, l + 112, b + 98)) { pulse("jet_connect"); scenery_.set_uniform_target(*selected, 1.0f); }
    else if (selected && inside(x, y, l + 38, b + 92, l + 112, b + 68)) { pulse("jet_disconnect"); scenery_.set_uniform_target(*selected, 0.0f); }
    else if (inside(x, y, l + 38, b + 62, l + 112, b + 38)) { pulse("jet_auto"); toggle_auto_(); }
    else if (inside(x, y, l + 18, b + 54, l + 148, b + 30)) { pulse("back"); tab_ = 7; }
    return 1;
  }

  if (tab_ == 5) {
    auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    int row_top = t - 74;
    const size_t shown = std::min<size_t>(displays.size(), 5);
    for (size_t i = 0; i < shown; ++i, row_top -= 30) {
      if (inside(x, y, l + 36, row_top, l + 176, row_top - 24)) {
        pulse(displays[i]->id.c_str());
        select_vdgs_(displays[i]);
        return 1;
      }
    }
    if (inside(x, y, l + 38, b + 122, l + 128, b + 98)) { pulse("park_arm"); if (!displays.empty()) select_vdgs_(displays.front()); }
    else if (inside(x, y, l + 38, b + 92, l + 128, b + 68)) { pulse("park_auto"); select_vdgs_(nullptr); }
    else if (inside(x, y, l + 38, b + 62, l + 128, b + 38)) { pulse("park_clear"); select_vdgs_(nullptr); }
    else if (inside(x, y, l + 18, b + 54, l + 148, b + 30)) { pulse("back"); tab_ = 7; }
    return 1;
  }

  if (tab_ == 3) {
    if (inside(x, y, l + 36, b + 108, l + 106, b + 84)) { pulse("set_auto"); toggle_auto_(); }
    else if (inside(x, y, l + 36, b + 78, l + 106, b + 54)) { pulse("set_dev"); toggle_developer_mode(); }
    else if (inside(x, y, l + 36, b + 48, l + 106, b + 24)) { pulse("set_reload"); reload_config_(); }
    else if (inside(x, y, l + 18, b + 54, l + 148, b + 30)) { pulse("back"); tab_ = 7; }
    return 1;
  }

  if (tab_ == 4 && developer_mode_) {
    if (developer_page_ == 0) {
      if (inside(x, y, l + 36, t - 74, l + 160, t - 98)) { pulse("dev_vehicle"); developer_page_ = 1; }
      else if (inside(x, y, l + 36, t - 104, l + 160, t - 128)) { pulse("dev_parking"); developer_page_ = 2; }
      else if (inside(x, y, l + 36, t - 134, l + 160, t - 158)) { pulse("dev_reload"); reload_config_(); }
      else if (inside(x, y, l + 36, t - 164, l + 160, t - 188)) { pulse("dev_exit"); toggle_developer_mode(); tab_ = 3; }
      else if (inside(x, y, l + 18, b + 54, l + 148, b + 30)) { pulse("back"); tab_ = 3; }
      return 1;
    }

    if (developer_page_ == 1) {
      const auto state = route_editor_.state();
      if (inside(x, y, l + 36, t - 74, l + 132, t - 98)) { pulse("veh_plan"); if (state == RouteEditorState::Idle) route_editor_.begin_planner(latitude_, longitude_, heading_); }
      else if (inside(x, y, l + 36, t - 104, l + 102, t - 128)) { pulse("veh_test"); if (state == RouteEditorState::Editing) route_editor_.start_test(); }
      else if (inside(x, y, l + 36, t - 134, l + 102, t - 158)) { pulse("veh_save"); if (state == RouteEditorState::Editing) route_editor_.save(); }
      else if (inside(x, y, l + 36, t - 164, l + 132, t - 188)) { pulse("veh_back"); developer_page_ = 0; }
      else if (inside(x, y, l + 220, b + 82, l + 278, b + 58)) { pulse("veh_spin"); toggle_vehicle_spin_(); }
      else if (inside(x, y, l + 220, b + 52, l + 278, b + 28)) { pulse("veh_delete"); route_editor_.cancel(); }
      return 1;
    }

    if (developer_page_ == 2) {
      if (inside(x, y, l + 36, t - 74, l + 132, t - 98)) { pulse("park_new"); if (vdgs_editor_.state() != VdgsEditorState::Placing) vdgs_editor_.begin(latitude_, longitude_, heading_); }
      else if (inside(x, y, l + 36, t - 104, l + 112, t - 128)) { pulse("park_snap"); reload_config_(); }
      else if (inside(x, y, l + 36, b + 82, l + 104, b + 58)) { pulse("park_delete"); vdgs_editor_.cancel(); }
      else if (inside(x, y, l + 36, b + 52, l + 132, b + 28)) { pulse("park_back"); developer_page_ = 0; }
      return 1;
    }
  }

  return 1;
}

} // namespace ssa
