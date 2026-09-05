#include "ssa/tablet.hpp"
#include <XPLMGraphics.h>
#include <XPLMProcessing.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
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

void disc(int cx, int cy, int radius) {
  const int step = 5;
  for (int y = -radius; y < radius; y += step) {
    const float ym = static_cast<float>(y) + step * 0.5f;
    const float rr = static_cast<float>(radius * radius);
    const float w = std::sqrt(std::max(0.0f, rr - ym * ym));
    panel(cx - static_cast<int>(w), cy - y,
          cx + static_cast<int>(w), cy - y - step);
  }
}

bool inside_circle(int x, int y, int cx, int cy, int radius) {
  const int dx = x - cx;
  const int dy = y - cy;
  return dx * dx + dy * dy <= radius * radius;
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
  params.left = 140;
  params.top = 790;
  params.right = 640;
  params.bottom = 330;
  params.visible = 0;
  params.drawWindowFunc = draw;
  params.handleMouseClickFunc = mouse;
  params.refcon = this;
  params.layer = xplm_WindowLayerFloatingWindows;
  params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
  window_ = XPLMCreateWindowEx(&params);
  XPLMSetWindowTitle(window_, "SSA");
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
  XPLMSetWindowTitle(window_, developer_mode_ ? "SSA [DEVELOPER]" : "SSA");
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

void Tablet::transition() {
  open_anim_ = std::min(open_anim_, 0.72f);
  last_draw_time_ = now_sec();
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
  open_anim_ = std::min(1.0f, open_anim_ + dt * 3.6f);
  const float p = std::clamp(open_anim_, 0.0f, 1.0f);
  const float appear = p * p * (3.0f - 2.0f * p);
  const float pulse_p = std::clamp((now - pulse_start_time_) / 0.24f, 0.0f, 1.0f);

  int l{}, t{}, r{}, b{};
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
  const int x_inset = static_cast<int>((1.0f - appear) * 44.0f);
  const int y_drop = static_cast<int>((1.0f - appear) * 24.0f);
  l += x_inset;
  r -= x_inset;
  t -= y_drop;
  b += static_cast<int>((1.0f - appear) * 10.0f);

  panel(l + 2, t - 2, r - 2, b + 2);
  panel(l + 8, t - 32, r - 8, b + 8);
  panel(l + 8, t - 4, r - 8, t - 28);
  panel(r - 32, t - 9, r - 12, t - 27);
  label(r - 27, t - 21, "X", 1.0f, 0.20f, 0.20f);

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
    // Dedicated footer zone: never share vertical space with action buttons.
    panel(l + 8, b + 42, r - 8, b + 8);
    button(l + 18, b + 34, l + 212, b + 12, text, "back", true);
  };

  auto title_for_tab = [&]() -> const char* {
    if (tab_ == 0) return "SSA HANGAR MENU";
    if (tab_ == 1) return "SSA JETWAY MENU";
    if (tab_ == 3) return "SSA SETTING MENU";
    if (tab_ == 4) return developer_page_ == 1 ? "SSA VEHICLE DEVELOPER MENU"
                          : developer_page_ == 2 ? "SSA PARKING DEVELOPER MENU"
                                                : "SSA DEVELOPER MENU";
    if (tab_ == 5) return "SSA PARKING MENU";
    if (tab_ == 6) return "SSA VDGS MENU";
    return "SSA MAIN MENU";
  };
  label(l + 18, t - 19, title_for_tab(), 0.94f, 0.94f, 0.94f);
  if (developer_mode_) label(r - 108, t - 19, "DEV", 1.0f, 0.68f, 0.12f);
  label(r - 62, b + 17, "v0.33", 0.42f, 0.48f, 0.54f);

  auto empty_state = [&](int left, int top, int right, int bottom,
                         const char* title, const char* hint) {
    panel(left, top, right, bottom);
    const int cx = (left + right) / 2;
    const int cy = (top + bottom) / 2;
    label(cx - static_cast<int>(std::strlen(title)) * 3, cy + 8, title,
          1.0f, 0.55f, 0.35f);
    label(cx - static_cast<int>(std::strlen(hint)) * 3, cy - 16, hint,
          0.58f, 0.64f, 0.70f);
  };

  // MAIN / RADIAL MENU
  if (tab_ == 7) {
    label(l + 24, t - 50, "SCENERY SERVICE ANIMATION", 0.66f, 0.66f, 0.66f);
    label(r - 146, t - 50, "X-PLANE CONNECTED", 0.35f, 0.95f, 0.55f);

    const int cx = (l + r) / 2;
    const int cy = (t + b) / 2 + 12;
    const int count = developer_mode_ ? 6 : 5;
    const int orbit = 88;
    const int bubble_r = 28;

    disc(cx, cy, 42);
    label(cx - 13, cy + 5, "SSA", 0.76f, 0.76f, 0.76f);
    label(cx - 20, cy - 14, "CLOSE", 0.62f, 0.62f, 0.62f);

    const char* labels_dev[] = {"HGR", "JET", "PARK", "VDGS", "SET", "DEV"};
    const char* labels_player[] = {"HGR", "JET", "PARK", "VDGS", "SET"};
    const char** labels = developer_mode_ ? labels_dev : labels_player;
    for (int i = 0; i < count; ++i) {
      const float a = -1.5707963f + i * (6.2831853f / static_cast<float>(count));
      const int bx = cx + static_cast<int>(std::cos(a) * orbit);
      const int by = cy + static_cast<int>(std::sin(a) * orbit);
      int extra = 0;
      char id[24];
      std::snprintf(id, sizeof(id), "main_%d", i);
      if (pulse_button_id_ == id && pulse_p < 1.0f)
        extra = static_cast<int>(std::sin(pulse_p * 3.1415926f) * 6.0f);
      disc(bx, by, bubble_r + extra);
      const std::string txt = labels[i];
      label(bx - static_cast<int>(txt.size()) * 4, by - 4, txt.c_str(),
            i == count - 1 && developer_mode_ ? 1.0f : kBlueR,
            i == count - 1 && developer_mode_ ? 0.68f : kBlueG,
            i == count - 1 && developer_mode_ ? 0.12f : kBlueB);
    }

    label(cx - 118, cy - 126, "HGR  JET  PARK  VDGS  SETTINGS", 0.58f, 0.66f, 0.74f);
    if (developer_mode_) label(cx + 78, cy - 126, "DEV", 1.0f, 0.68f, 0.12f);

    char nearby[160];
    const auto hangars = scenery_.nearby(ServiceType::Hangar, latitude_, longitude_, 2000.0);
    const auto jetways = scenery_.nearby(ServiceType::Jetway, latitude_, longitude_, 1000.0);
    const auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    std::snprintf(nearby, sizeof(nearby), "NEARBY %zu HGR  |  %zu JET  |  %zu VDGS",
                  hangars.size(), jetways.size(), displays.size());
    label(l + 24, b + 26, nearby, 0.58f, 0.70f, 0.80f);
    return;
  }

  // HANGAR MENU
  if (tab_ == 0) {
    auto hangars = scenery_.nearby(ServiceType::Hangar, latitude_, longitude_, 2000.0);
    if (selected_hangar_id_.empty() && !hangars.empty()) selected_hangar_id_ = hangars.front()->id;
    ServiceObject* selected = find_by_id(hangars, selected_hangar_id_);

    panel(l + 24, t - 56, l + 190, b + 164);
    label(l + 36, t - 72, "NEARBY HANGARS", 0.62f, 0.68f, 0.74f);
    int y = t - 88;
    const size_t shown = std::min<size_t>(hangars.size(), 5);
    for (size_t i = 0; i < shown; ++i, y -= 30) {
      auto* object = hangars[i];
      char row[96];
      std::snprintf(row, sizeof(row), "%zu. %.22s", i + 1, object->label.c_str());
      button(l + 36, y, l + 178, y - 24, row, object->id.c_str(),
             object->id == selected_hangar_id_);
    }
    if (hangars.empty())
      empty_state(l + 36, t - 92, l + 178, t - 190, "NO HANGAR", "MOVE CLOSER / RELOAD");

    panel(l + 210, t - 56, r - 24, b + 164);
    label(l + 226, t - 72, "HANGAR STATUS", 0.62f, 0.68f, 0.74f);
    if (selected) {
      const char* state = "CLOSED";
      if (selected->progress >= 0.999f) state = "OPEN";
      else if (selected->target > selected->progress) state = "OPENING";
      else if (selected->target < selected->progress) state = "CLOSING";
      char info1[128];
      char info2[128];
      char info3[128];
      std::snprintf(info1, sizeof(info1), "SELECTED  %.26s", selected->label.c_str());
      std::snprintf(info2, sizeof(info2), "STATE     %s", state);
      std::snprintf(info3, sizeof(info3), "POSITION  %.0f%%", selected->progress * 100.0f);
      label(l + 226, t - 104, info1, 0.78f, 0.78f, 0.78f);
      label(l + 226, t - 132, info2, kBlueR, kBlueG, kBlueB);
      label(l + 226, t - 158, info3, 0.72f, 0.76f, 0.80f);
      label(l + 226, t - 196, "DOOR CONTROL", 0.58f, 0.64f, 0.70f);
      button(l + 226, t - 212, l + 294, t - 236, "OPEN", "hang_open",
             selected->target > 0.5f);
      button(l + 302, t - 212, l + 378, t - 236, "CLOSE", "hang_close",
             selected->target <= 0.5f);
      button(l + 386, t - 212, r - 36, t - 236, "TOGGLE", "hang_toggle");
    } else {
      empty_state(l + 226, t - 100, r - 36, t - 222,
                  "NOTHING SELECTED", "SELECT A HANGAR ON THE LEFT");
    }
    footer_back();
    return;
  }

  // JETWAY MENU
  if (tab_ == 1) {
    auto jetways = scenery_.nearby(ServiceType::Jetway, latitude_, longitude_, 50.0);
    if (selected_jetway_id_.empty() && !jetways.empty()) selected_jetway_id_ = jetways.front()->id;
    ServiceObject* selected = find_by_id(jetways, selected_jetway_id_);

    panel(l + 24, t - 56, l + 190, b + 164);
    label(l + 36, t - 72, "NEARBY JETWAYS", 0.62f, 0.68f, 0.74f);
    int y = t - 88;
    const size_t shown = std::min<size_t>(jetways.size(), 5);
    for (size_t i = 0; i < shown; ++i, y -= 30) {
      auto* object = jetways[i];
      char row[96];
      std::snprintf(row, sizeof(row), "%zu. %.22s", i + 1, object->label.c_str());
      button(l + 36, y, l + 178, y - 24, row, object->id.c_str(),
             object->id == selected_jetway_id_);
    }
    if (jetways.empty())
      empty_state(l + 36, t - 92, l + 178, t - 190, "NO JETWAY", "MOVE CLOSER TO A GATE");

    panel(l + 210, t - 56, r - 24, b + 164);
    label(l + 226, t - 72, "JETWAY STATUS", 0.62f, 0.68f, 0.74f);
    if (selected) {
      char info1[128];
      char info2[128];
      char info3[128];
      std::snprintf(info1, sizeof(info1), "SELECTED  %.26s", selected->label.c_str());
      std::snprintf(info2, sizeof(info2), "STATE     %s", jetway_state_name(selected->jetway_state));
      if (selected->head_error_m >= 0.0f)
        std::snprintf(info3, sizeof(info3), "DOOR L1   %.0f CM", selected->head_error_m * 100.0f);
      else
        std::snprintf(info3, sizeof(info3), "DOOR L1   WAITING");
      label(l + 226, t - 104, info1, 0.78f, 0.78f, 0.78f);
      label(l + 226, t - 132, info2, kBlueR, kBlueG, kBlueB);
      label(l + 226, t - 158, info3, 0.72f, 0.76f, 0.80f);
      label(l + 226, t - 196, automatic_ ? "AUTOMATIC MODE ON" : "MANUAL MODE",
            automatic_ ? 0.35f : 0.72f, automatic_ ? 0.95f : 0.76f, automatic_ ? 0.55f : 0.80f);
      button(l + 226, t - 212, l + 310, t - 236, "CONNECT", "jet_connect");
      button(l + 318, t - 212, l + 414, t - 236, "DISCONNECT", "jet_disconnect");
      button(l + 422, t - 212, r - 36, t - 236, automatic_ ? "AUTO ON" : "AUTO OFF",
             "jet_auto", automatic_);
    } else {
      empty_state(l + 226, t - 100, r - 36, t - 222,
                  "NOTHING SELECTED", "SELECT A JETWAY ON THE LEFT");
    }
    footer_back();
    return;
  }

  // PARKING MENU
  if (tab_ == 5) {
    auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);

    panel(l + 24, t - 56, l + 190, b + 164);
    label(l + 36, t - 72, "PARKING STANDS", 0.62f, 0.68f, 0.74f);
    int y = t - 88;
    const size_t shown = std::min<size_t>(displays.size(), 5);
    ServiceObject* selected = nullptr;
    for (size_t i = 0; i < shown; ++i, y -= 30) {
      auto* display = displays[i];
      if (display->vdgs_selected || display->vdgs_armed) selected = display;
      char row[96];
      std::snprintf(row, sizeof(row), "%zu. %.22s", i + 1, display->label.c_str());
      button(l + 36, y, l + 178, y - 24, row, display->id.c_str(),
             display->vdgs_selected || display->vdgs_armed);
    }
    if (!selected && !displays.empty()) selected = displays.front();
    if (displays.empty())
      empty_state(l + 36, t - 92, l + 178, t - 190, "NO PARKING", "NO SSA STAND NEARBY");

    panel(l + 210, t - 56, r - 24, b + 164);
    label(l + 226, t - 72, "PARKING STATUS", 0.62f, 0.68f, 0.74f);
    if (selected) {
      char info1[128], info2[128], info3[128];
      std::snprintf(info1, sizeof(info1), "STAND     %.26s", selected->label.c_str());
      std::snprintf(info2, sizeof(info2), "VDGS      %s", vdgs_state_name(selected->vdgs_state));
      std::snprintf(info3, sizeof(info3), "MODE      %s", selected->vdgs_armed ? "MANUAL" : "AUTO");
      label(l + 226, t - 104, info1, 0.78f, 0.78f, 0.78f);
      label(l + 226, t - 132, info2, kBlueR, kBlueG, kBlueB);
      label(l + 226, t - 158, info3, 0.72f, 0.76f, 0.80f);
      button(l + 226, t - 212, l + 310, t - 236, "ARM VDGS", "park_arm", selected->vdgs_armed);
      button(l + 318, t - 212, l + 402, t - 236, "AUTO MODE", "park_auto", !selected->vdgs_armed);
      button(l + 410, t - 212, r - 36, t - 236, "CLEAR", "park_clear");
    } else {
      empty_state(l + 226, t - 100, r - 36, t - 222,
                  "NO STAND ACTIVE", "SELECT A PARKING STAND");
    }
    footer_back();
    return;
  }

  // VDGS STATUS MENU
  if (tab_ == 6) {
    auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    int y = t - 88;
    const size_t shown = std::min<size_t>(displays.size(), 5);
    ServiceObject* active = nullptr;

    panel(l + 24, t - 56, l + 190, b + 164);
    label(l + 36, t - 72, "VDGS UNITS", 0.62f, 0.68f, 0.74f);
    for (size_t i = 0; i < shown; ++i, y -= 30) {
      auto* display = displays[i];
      if (display->vdgs_selected || display->vdgs_armed) active = display;
      char row[96];
      std::snprintf(row, sizeof(row), "%zu. %.22s", i + 1, display->label.c_str());
      button(l + 36, y, l + 178, y - 24, row, display->id.c_str(),
             display->vdgs_selected || display->vdgs_armed);
    }
    if (!active && !displays.empty()) active = displays.front();
    if (displays.empty())
      empty_state(l + 36, t - 92, l + 178, t - 190, "NO VDGS", "NO SSA VDGS NEARBY");

    panel(l + 210, t - 56, r - 24, b + 164);
    label(l + 226, t - 72, "GUIDANCE DISPLAY", 0.62f, 0.68f, 0.74f);
    if (active) {
      char state[96], dist[96], lat[96];
      std::snprintf(state, sizeof(state), "STATE     %s", vdgs_state_name(active->vdgs_state));
      std::snprintf(dist, sizeof(dist), "DISTANCE  %+.1f M", active->vdgs_distance_error_m);
      std::snprintf(lat, sizeof(lat), "LATERAL   %+.2f M", active->vdgs_lateral_error_m);
      label(l + 226, t - 104, state, kBlueR, kBlueG, kBlueB);
      label(l + 226, t - 132, dist, 0.74f, 0.78f, 0.82f);
      label(l + 226, t - 156, lat, 0.74f, 0.78f, 0.82f);

      label(l + 258, t - 194, "CENTERLINE", 0.54f, 0.60f, 0.66f);
      label(l + 244, t - 214, "<----- | ----->", 0.66f, 0.72f, 0.78f);
      if (active->vdgs_state == VdgsState::Stop)
        label(l + 282, t - 242, "STOP", 1.0f, 0.22f, 0.18f);
      else if (active->vdgs_state == VdgsState::Slow)
        label(l + 278, t - 242, "SLOW", 1.0f, 0.68f, 0.12f);
      else
        label(l + 256, t - 242, "FOLLOW GUIDANCE", 0.35f, 0.95f, 0.55f);

      button(l + 226, b + 190, l + 318, b + 166, "AUTO MODE", "vdgs_auto", !active->vdgs_armed);
      button(l + 326, b + 190, l + 402, b + 166, "CLEAR", "vdgs_clear");
    } else {
      empty_state(l + 226, t - 100, r - 36, t - 222,
                  "NO GUIDANCE", "SELECT A VDGS ON THE LEFT");
    }
    footer_back();
    return;
  }

  // SETTINGS
  if (tab_ == 3) {
    const char* setting_names[] = {"GENERAL", "VEHICLE", "HANGAR", "PARKING", "VDGS"};
    int sy = t - 74;
    for (int i = 0; i < 5; ++i, sy -= 30) {
      char id[24];
      std::snprintf(id, sizeof(id), "set_page_%d", i);
      button(l + 36, sy, l + 126, sy - 24, setting_names[i], id, settings_page_ == i);
    }

    panel(l + 154, t - 56, r - 24, b + 164);
    if (settings_page_ == 0) {
      label(l + 172, t - 76, "GENERAL", 0.82f, 0.82f, 0.82f);
      label(l + 172, t - 106, automatic_ ? "AUTOMATIC JETWAY     ON" : "AUTOMATIC JETWAY     OFF",
            automatic_ ? 0.35f : 0.74f, automatic_ ? 0.95f : 0.76f, automatic_ ? 0.55f : 0.80f);
      label(l + 172, t - 132, developer_mode_ ? "DEVELOPER MODE       ON" : "DEVELOPER MODE       OFF",
            developer_mode_ ? 1.0f : 0.74f, developer_mode_ ? 0.68f : 0.76f, developer_mode_ ? 0.12f : 0.80f);
      label(l + 172, t - 174, "Developer tools stay hidden in Player Mode.", 0.58f, 0.64f, 0.70f);
    } else if (settings_page_ == 1) {
      label(l + 172, t - 76, "VEHICLE", 0.82f, 0.82f, 0.82f);
      char agl[128];
      char current[128];
      std::snprintf(agl, sizeof(agl), "HIDE TRAFFIC ABOVE   %.0f FT AGL", route_editor_.vehicle_hide_agl_ft());
      std::snprintf(current, sizeof(current), "AIRCRAFT HEIGHT      %.0f FT AGL", route_editor_.aircraft_agl_ft());
      label(l + 172, t - 106, agl, 0.72f, 0.76f, 0.80f);
      label(l + 172, t - 132, current, 0.72f, 0.76f, 0.80f);
      label(l + 172, t - 158, route_editor_.traffic_visible() ? "SAVED TRAFFIC        ACTIVE" : "SAVED TRAFFIC        HIDDEN",
            route_editor_.traffic_visible() ? 0.35f : 1.0f,
            route_editor_.traffic_visible() ? 0.95f : 0.55f,
            route_editor_.traffic_visible() ? 0.55f : 0.35f);
      char routes[96];
      std::snprintf(routes, sizeof(routes), "SAVED ROUTES         %zu", route_editor_.saved_route_count());
      label(l + 172, t - 184, routes, 0.72f, 0.76f, 0.80f);
    } else if (settings_page_ == 2) {
      label(l + 172, t - 76, "HANGAR", 0.82f, 0.82f, 0.82f);
      label(l + 172, t - 108, "Nearby hangars are searched within 2 KM.", 0.66f, 0.70f, 0.74f);
      label(l + 172, t - 134, "OPEN / CLOSE controls the selected SSA door.", 0.66f, 0.70f, 0.74f);
    } else if (settings_page_ == 3) {
      label(l + 172, t - 76, "PARKING", 0.82f, 0.82f, 0.82f);
      label(l + 172, t - 108, "Select a stand to arm its parking guidance.", 0.66f, 0.70f, 0.74f);
      label(l + 172, t - 134, "AUTO MODE lets SSA choose the active stand.", 0.66f, 0.70f, 0.74f);
    } else {
      label(l + 172, t - 76, "VDGS", 0.82f, 0.82f, 0.82f);
      label(l + 172, t - 108, "Guidance shows distance and lateral error.", 0.66f, 0.70f, 0.74f);
      label(l + 172, t - 134, "Developer Mode unlocks placement tools.", 0.66f, 0.70f, 0.74f);
    }

    label(l + 36, b + 156, "QUICK CONTROLS", 0.58f, 0.64f, 0.70f);
    button(l + 36, b + 138, l + 126, b + 114,
           automatic_ ? "AUTO ON" : "AUTO OFF", "set_auto", automatic_);
    button(l + 36, b + 108, l + 126, b + 84,
           developer_mode_ ? "DEV ON" : "DEV OFF", "set_dev", developer_mode_);
    button(l + 36, b + 78, l + 126, b + 54, "RELOAD", "set_reload", true);
    footer_back("BACK TO MAIN MENU");
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
      footer_back("BACK TO SETTINGS");
      return;
    }

    if (developer_page_ == 1) {
      const auto route_state = route_editor_.state();
      const size_t route_count = route_editor_.saved_route_count();
      if (route_count == 0) {
        selected_saved_route_index_ = 0;
        delete_route_confirm_ = false;
      } else if (selected_saved_route_index_ >= route_count) {
        selected_saved_route_index_ = route_count - 1;
        delete_route_confirm_ = false;
      }

      button(l + 28, t - 74, l + 140, t - 98, "PLAN NEW ROUTE", "veh_plan", true);
      if (route_state == RouteEditorState::Testing)
        button(l + 28, t - 104, l + 140, t - 128, "STOP TEST", "veh_test", false, true);
      else
        button(l + 28, t - 104, l + 140, t - 128, "TEST ROUTE", "veh_test", true);
      button(l + 28, t - 134, l + 92, t - 158, "SAVE", "veh_save");
      button(l + 28, t - 164, l + 140, t - 188, "BACK TO DEV", "veh_back", true);
      button(l + 28, b + 104, l + 100, b + 80, "TEST SPIN", "veh_spin");

      label(l + 174, t - 62, "SAVED ROUTES", 0.82f, 0.82f, 0.82f);
      int route_y = t - 76;
      const size_t shown_routes = std::min<size_t>(route_count, 5);
      for (size_t i = 0; i < shown_routes; ++i, route_y -= 30) {
        const auto* route = route_editor_.saved_route(i);
        if (!route) continue;
        char row[128];
        std::snprintf(row, sizeof(row), "%zu. %.24s%s", i + 1,
                      route->label.c_str(), route->running ? "  [RUN]" : "");
        char id[32];
        std::snprintf(id, sizeof(id), "route_%zu", i);
        button(l + 174, route_y, r - 24, route_y - 24, row, id,
               i == selected_saved_route_index_);
      }
      if (route_count == 0)
        label(l + 174, t - 88, "NO SAVED ROUTES", 1.0f, 0.55f, 0.35f);

      const bool route_actions_enabled = route_state == RouteEditorState::Idle && route_count > 0;
      const auto* selected_route = route_count > 0
                                       ? route_editor_.saved_route(selected_saved_route_index_)
                                       : nullptr;
      if (selected_route) {
        char selected_info[160];
        std::snprintf(selected_info, sizeof(selected_info), "SELECTED: %.28s", selected_route->label.c_str());
        label(l + 174, b + 136, selected_info, 0.70f, 0.76f, 0.82f);
      }
      button(l + 174, b + 118, l + 236, b + 94, "EDIT", "route_edit", true, false,
             !route_actions_enabled);
      button(l + 244, b + 118, l + 306, b + 94, "START", "route_start", true, false,
             !route_actions_enabled);
      button(l + 314, b + 118, l + 376, b + 94, "STOP", "route_stop", false, false,
             !route_actions_enabled);
      button(l + 174, b + 84, l + 318, b + 58,
             delete_route_confirm_ ? "CONFIRM DELETE" : "DELETE ROUTE",
             "route_delete", false, true, !route_actions_enabled);
      if (delete_route_confirm_)
        label(l + 326, b + 68, "CLICK AGAIN TO DELETE", 1.0f, 0.42f, 0.28f);

      char live_status[220];
      char area_status[96];
      if (route_state == RouteEditorState::Testing) {
        std::snprintf(area_status, sizeof(area_status), "TEST ACTIVE");
      } else if (route_editor_.traffic_visible()) {
        std::snprintf(area_status, sizeof(area_status), "TRAFFIC ACTIVE %.0f FT AGL",
                      route_editor_.aircraft_agl_ft());
      } else {
        std::snprintf(area_status, sizeof(area_status), "TRAFFIC HIDDEN %.0f FT AGL",
                      route_editor_.aircraft_agl_ft());
      }
      std::snprintf(live_status, sizeof(live_status),
                    "%s | %.1f KM/H | %s",
                    route_editor_.status().c_str(),
                    route_editor_.test_speed_mps() * 3.6f,
                    area_status);
      label(l + 174, b + 38, live_status, 0.72f, 0.78f, 0.84f);
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
    const int cx = (l + r) / 2;
    const int cy = (t + b) / 2 + 12;
    const int count = developer_mode_ ? 6 : 5;
    const int orbit = 88;
    const int bubble_r = 32;

    if (inside_circle(x, y, cx, cy, 42)) {
      XPLMSetWindowIsVisible(window_, 0);
      return 1;
    }

    for (int i = 0; i < count; ++i) {
      const float a = -1.5707963f + i * (6.2831853f / static_cast<float>(count));
      const int bx = cx + static_cast<int>(std::cos(a) * orbit);
      const int by = cy + static_cast<int>(std::sin(a) * orbit);
      if (!inside_circle(x, y, bx, by, bubble_r)) continue;
      char id[24];
      std::snprintf(id, sizeof(id), "main_%d", i);
      pulse(id);
      if (i == 0) tab_ = 0;
      else if (i == 1) tab_ = 1;
      else if (i == 2) tab_ = 5;
      else if (i == 3) tab_ = 6;
      else if (i == 4) tab_ = 3;
      else if (i == 5 && developer_mode_) { developer_page_ = 0; tab_ = 4; }
      transition();
      return 1;
    }
    return 1;
  }

  if (tab_ == 0) {
    auto hangars = scenery_.nearby(ServiceType::Hangar, latitude_, longitude_, 2000.0);
    int row_top = t - 88;
    const size_t shown = std::min<size_t>(hangars.size(), 5);
    for (size_t i = 0; i < shown; ++i, row_top -= 30) {
      if (inside(x, y, l + 36, row_top, l + 178, row_top - 24)) {
        selected_hangar_id_ = hangars[i]->id;
        pulse(hangars[i]->id.c_str());
        return 1;
      }
    }
    ServiceObject* selected = find_by_id(hangars, selected_hangar_id_);
    if (selected && inside(x, y, l + 226, t - 212, l + 294, t - 236)) { pulse("hang_open"); scenery_.set_uniform_target(*selected, 1.0f); }
    else if (selected && inside(x, y, l + 302, t - 212, l + 378, t - 236)) { pulse("hang_close"); scenery_.set_uniform_target(*selected, 0.0f); }
    else if (selected && inside(x, y, l + 386, t - 212, r - 36, t - 236)) { pulse("hang_toggle"); toggle_object_(*selected); }
    else if (inside(x, y, l + 18, b + 34, l + 212, b + 12)) { pulse("back"); tab_ = 7; transition(); }
    return 1;
  }

  if (tab_ == 1) {
    auto jetways = scenery_.nearby(ServiceType::Jetway, latitude_, longitude_, 50.0);
    int row_top = t - 88;
    const size_t shown = std::min<size_t>(jetways.size(), 5);
    for (size_t i = 0; i < shown; ++i, row_top -= 30) {
      if (inside(x, y, l + 36, row_top, l + 178, row_top - 24)) {
        selected_jetway_id_ = jetways[i]->id;
        pulse(jetways[i]->id.c_str());
        return 1;
      }
    }
    ServiceObject* selected = find_by_id(jetways, selected_jetway_id_);
    if (selected && inside(x, y, l + 226, t - 212, l + 310, t - 236)) { pulse("jet_connect"); scenery_.set_uniform_target(*selected, 1.0f); }
    else if (selected && inside(x, y, l + 318, t - 212, l + 414, t - 236)) { pulse("jet_disconnect"); scenery_.set_uniform_target(*selected, 0.0f); }
    else if (inside(x, y, l + 422, t - 212, r - 36, t - 236)) { pulse("jet_auto"); toggle_auto_(); }
    else if (inside(x, y, l + 18, b + 34, l + 212, b + 12)) { pulse("back"); tab_ = 7; transition(); }
    return 1;
  }

  if (tab_ == 5) {
    auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    int row_top = t - 88;
    const size_t shown = std::min<size_t>(displays.size(), 5);
    for (size_t i = 0; i < shown; ++i, row_top -= 30) {
      if (inside(x, y, l + 36, row_top, l + 178, row_top - 24)) {
        pulse(displays[i]->id.c_str());
        select_vdgs_(displays[i]);
        return 1;
      }
    }
    if (inside(x, y, l + 226, t - 212, l + 310, t - 236)) { pulse("park_arm"); if (!displays.empty()) select_vdgs_(displays.front()); }
    else if (inside(x, y, l + 318, t - 212, l + 402, t - 236)) { pulse("park_auto"); select_vdgs_(nullptr); }
    else if (inside(x, y, l + 410, t - 212, r - 36, t - 236)) { pulse("park_clear"); select_vdgs_(nullptr); }
    else if (inside(x, y, l + 18, b + 34, l + 212, b + 12)) { pulse("back"); tab_ = 7; transition(); }
    return 1;
  }

  if (tab_ == 6) {
    auto displays = scenery_.nearby(ServiceType::ParkingDisplay, latitude_, longitude_, 2000.0);
    int row_top = t - 88;
    const size_t shown = std::min<size_t>(displays.size(), 5);
    for (size_t i = 0; i < shown; ++i, row_top -= 30) {
      if (inside(x, y, l + 36, row_top, l + 178, row_top - 24)) {
        pulse(displays[i]->id.c_str());
        select_vdgs_(displays[i]);
        return 1;
      }
    }
    if (inside(x, y, l + 226, b + 190, l + 318, b + 166)) { pulse("vdgs_auto"); select_vdgs_(nullptr); }
    else if (inside(x, y, l + 326, b + 190, l + 402, b + 166)) { pulse("vdgs_clear"); select_vdgs_(nullptr); }
    else if (inside(x, y, l + 18, b + 34, l + 212, b + 12)) { pulse("back"); tab_ = 7; transition(); }
    return 1;
  }

  if (tab_ == 3) {
    int sy = t - 74;
    for (int i = 0; i < 5; ++i, sy -= 30) {
      if (inside(x, y, l + 36, sy, l + 126, sy - 24)) {
        settings_page_ = i;
        char id[24];
        std::snprintf(id, sizeof(id), "set_page_%d", i);
        pulse(id);
        return 1;
      }
    }
    if (inside(x, y, l + 36, b + 138, l + 126, b + 114)) { pulse("set_auto"); toggle_auto_(); }
    else if (inside(x, y, l + 36, b + 108, l + 126, b + 84)) { pulse("set_dev"); toggle_developer_mode(); }
    else if (inside(x, y, l + 36, b + 78, l + 126, b + 54)) { pulse("set_reload"); reload_config_(); }
    else if (inside(x, y, l + 18, b + 34, l + 212, b + 12)) { pulse("back"); tab_ = 7; transition(); }
    return 1;
  }

  if (tab_ == 4 && developer_mode_) {
    if (developer_page_ == 0) {
      if (inside(x, y, l + 36, t - 74, l + 160, t - 98)) { pulse("dev_vehicle"); developer_page_ = 1; transition(); }
      else if (inside(x, y, l + 36, t - 104, l + 160, t - 128)) { pulse("dev_parking"); developer_page_ = 2; transition(); }
      else if (inside(x, y, l + 36, t - 134, l + 160, t - 158)) { pulse("dev_reload"); reload_config_(); }
      else if (inside(x, y, l + 36, t - 164, l + 160, t - 188)) { pulse("dev_exit"); toggle_developer_mode(); tab_ = 3; transition(); }
      else if (inside(x, y, l + 18, b + 34, l + 212, b + 12)) { pulse("back"); tab_ = 3; transition(); }
      return 1;
    }

    if (developer_page_ == 1) {
      const auto state = route_editor_.state();
      const size_t route_count = route_editor_.saved_route_count();

      int route_y = t - 76;
      const size_t shown_routes = std::min<size_t>(route_count, 5);
      for (size_t i = 0; i < shown_routes; ++i, route_y -= 30) {
        if (inside(x, y, l + 174, route_y, r - 24, route_y - 24)) {
          selected_saved_route_index_ = i;
          delete_route_confirm_ = false;
          char id[32];
          std::snprintf(id, sizeof(id), "route_%zu", i);
          pulse(id);
          return 1;
        }
      }

      if (inside(x, y, l + 28, t - 74, l + 140, t - 98)) {
        pulse("veh_plan");
        delete_route_confirm_ = false;
        if (state == RouteEditorState::Idle)
          route_editor_.begin_planner(latitude_, longitude_, heading_);
      }
      else if (inside(x, y, l + 28, t - 104, l + 140, t - 128)) {
        pulse("veh_test");
        delete_route_confirm_ = false;
        if (state == RouteEditorState::Testing)
          route_editor_.stop_test();
        else if (state == RouteEditorState::Editing || state == RouteEditorState::Planning)
          route_editor_.start_test();
      }
      else if (inside(x, y, l + 28, t - 134, l + 92, t - 158)) {
        pulse("veh_save");
        delete_route_confirm_ = false;
        if (state == RouteEditorState::Editing || state == RouteEditorState::Planning)
          route_editor_.save();
      }
      else if (inside(x, y, l + 28, t - 164, l + 140, t - 188)) {
        pulse("veh_back");
        delete_route_confirm_ = false;
        developer_page_ = 0;
        transition();
      }
      else if (inside(x, y, l + 28, b + 104, l + 100, b + 80)) {
        pulse("veh_spin");
        toggle_vehicle_spin_();
      }
      else if (state == RouteEditorState::Idle && route_count > 0 &&
               inside(x, y, l + 174, b + 118, l + 236, b + 94)) {
        pulse("route_edit");
        delete_route_confirm_ = false;
        route_editor_.edit_saved_route(selected_saved_route_index_);
      }
      else if (state == RouteEditorState::Idle && route_count > 0 &&
               inside(x, y, l + 244, b + 118, l + 306, b + 94)) {
        pulse("route_start");
        delete_route_confirm_ = false;
        route_editor_.start_saved_route(selected_saved_route_index_);
      }
      else if (state == RouteEditorState::Idle && route_count > 0 &&
               inside(x, y, l + 314, b + 118, l + 376, b + 94)) {
        pulse("route_stop");
        delete_route_confirm_ = false;
        route_editor_.stop_saved_route(selected_saved_route_index_);
      }
      else if (state == RouteEditorState::Idle && route_count > 0 &&
               inside(x, y, l + 174, b + 84, l + 318, b + 58)) {
        pulse("route_delete");
        if (!delete_route_confirm_) {
          delete_route_confirm_ = true;
        } else {
          if (route_editor_.delete_saved_route(selected_saved_route_index_)) {
            const size_t new_count = route_editor_.saved_route_count();
            if (new_count == 0) selected_saved_route_index_ = 0;
            else if (selected_saved_route_index_ >= new_count)
              selected_saved_route_index_ = new_count - 1;
          }
          delete_route_confirm_ = false;
        }
      }
      return 1;
    }

    if (developer_page_ == 2) {
      if (inside(x, y, l + 36, t - 74, l + 132, t - 98)) { pulse("park_new"); if (vdgs_editor_.state() != VdgsEditorState::Placing) vdgs_editor_.begin(latitude_, longitude_, heading_); }
      else if (inside(x, y, l + 36, t - 104, l + 112, t - 128)) { pulse("park_snap"); reload_config_(); }
      else if (inside(x, y, l + 36, b + 82, l + 104, b + 58)) { pulse("park_delete"); vdgs_editor_.cancel(); }
      else if (inside(x, y, l + 36, b + 52, l + 132, b + 28)) { pulse("park_back"); developer_page_ = 0; transition(); }
      return 1;
    }
  }

  return 1;
}

} // namespace ssa
