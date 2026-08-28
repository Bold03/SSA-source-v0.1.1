#include "ssa/route_editor.hpp"
#include <XPLMGraphics.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace ssa {
namespace {
constexpr float pi = 3.14159265358979323846f;
float normalize_heading(float value) {
  while (value < 0.0f) value += 360.0f;
  while (value >= 360.0f) value -= 360.0f;
  return value;
}
}

RouteEditor::~RouteEditor() { unload(); }

void RouteEditor::unload() {
  if (planner_active_) close_planner();
  if (planner_window_) XPLMDestroyWindow(planner_window_);
  if (instance_) XPLMDestroyInstance(instance_);
  for (auto& route : traffic_routes_)
    if (route.instance) XPLMDestroyInstance(route.instance);
  if (object_) XPLMUnloadObject(object_);
  if (probe_) XPLMDestroyProbe(probe_);
  instance_ = nullptr;
  object_ = nullptr;
  probe_ = nullptr;
  planner_window_ = nullptr;
  planner_drag_active_ = false;
  route_load_pending_ = false;
  route_load_delay_seconds_ = 0.0f;
  handle_drag_anchor_ = -1;
  points_.clear();
  traffic_routes_.clear();
  test_points_.clear();
  test_distance_remaining_.clear();
  return_to_planner_after_test_ = false;
  loop_enabled_ = false;
  editing_route_id_.clear();
  editing_route_label_.clear();
  current_speed_mps_ = 0.0f;
  state_ = RouteEditorState::Unavailable;
}

bool RouteEditor::load(const std::string& xplane_root) {
  unload();
  field_of_view_ref_ = XPLMFindDataRef("sim/graphics/view/field_of_view_deg");
  const fs::path custom = fs::path(xplane_root) / "Custom Scenery";
  try {
    if (!fs::exists(custom)) {
      status_ = "Custom Scenery folder not found";
      return false;
    }
    for (const auto& entry : fs::directory_iterator(custom)) {
      const fs::path config_path = entry.path() / "ssa.json";
      if (!entry.is_directory() || !fs::exists(config_path)) continue;
      std::ifstream input(config_path);
      const json root = json::parse(input);
      if (!root.contains("vehicle_models") || !root.at("vehicle_models").is_array() ||
          root.at("vehicle_models").empty()) continue;
      const auto& model = root.at("vehicle_models").front();
      model_id_ = model.value("id", "gapura_bus");
      model_label_ = model.value("label", model_id_);
      ground_offset_m_ = model.value("ground_offset_m", 0.445f);
      speed_mps_ = std::clamp(model.value("speed_mps", 4.0f), 0.5f, 15.0f);
      acceleration_mps2_ =
          std::clamp(model.value("acceleration_mps2", 1.5f), 0.2f, 6.0f);
      braking_mps2_ = std::clamp(model.value("braking_mps2", 2.5f), 0.5f, 8.0f);
      heading_offset_deg_ = model.value("heading_offset_deg", 180.0f);
      steering_multiplier_ = model.value("steering_multiplier", -1.0f);
      body_lookahead_m_ = std::clamp(model.value("body_lookahead_m", 6.0f), 1.0f, 12.0f);
      body_heading_response_ =
          std::clamp(model.value("body_heading_response", 1.8f), 0.5f, 8.0f);
      rear_axle_to_origin_m_ =
          std::clamp(model.value("rear_axle_to_origin_m", 3.8f), -12.0f, 12.0f);
      wheelbase_m_ = std::clamp(model.value("wheelbase_m", 7.6f), 1.0f, 15.0f);
      max_steering_deg_ =
          std::clamp(model.value("max_steering_deg", 35.0f), 10.0f, 60.0f);
      scenery_directory_ = entry.path().string();
      route_path_ = (entry.path() / "ssa_routes.json").string();
      const fs::path object_path = entry.path() / model.at("object").get<std::string>();
      object_ = XPLMLoadObject(object_path.string().c_str());
      if (!object_) {
        status_ = "Cannot load " + object_path.filename().string();
        return false;
      }
      const char* datarefs[] = {"boldstudio31/ssa/vehicle/wheel_spin",
                                "boldstudio31/ssa/vehicle/steering", nullptr};
      instance_ = XPLMCreateInstance(object_, datarefs);
      probe_ = XPLMCreateProbe(xplm_ProbeY);
      if (!instance_ || !probe_) {
        status_ = "Cannot create vehicle instance";
        unload();
        return false;
      }
      state_ = RouteEditorState::Idle;
      status_ = "Ready: " + model_label_ + " | Waiting for scenery";
      return true;
    }
  } catch (const std::exception& e) {
    status_ = e.what();
    return false;
  }
  status_ = "Add vehicle_models to ssa.json";
  return false;
}

void RouteEditor::schedule_saved_route_load() {
  if (state_ == RouteEditorState::Unavailable || !instance_ || !probe_) return;
  route_load_pending_ = true;
  route_load_delay_seconds_ = 0.0f;
  status_ = "Scenery ready | Loading saved traffic route";
}

bool RouteEditor::load_saved_route() {
  for (auto& route : traffic_routes_)
    if (route.instance) XPLMDestroyInstance(route.instance);
  traffic_routes_.clear();
  if (route_path_.empty() || !fs::exists(route_path_)) return false;
  try {
    std::ifstream input(route_path_);
    const json root = json::parse(input);
    if (!root.contains("routes") || !root.at("routes").is_array() ||
        root.at("routes").empty()) return false;
    const char* datarefs[] = {"boldstudio31/ssa/vehicle/wheel_spin",
                              "boldstudio31/ssa/vehicle/steering", nullptr};
    size_t fallback_number = 1;
    for (const auto& route_json : root.at("routes")) {
      if (traffic_routes_.size() >= 16) break;
      if (route_json.value("model", model_id_) != model_id_ ||
          !route_json.contains("waypoints") ||
          !route_json.at("waypoints").is_array()) continue;
      TrafficRoute route;
      char fallback_id[32];
      std::snprintf(fallback_id, sizeof(fallback_id), "bus_route_%02zu", fallback_number);
      route.id = route_json.value("id", std::string(fallback_id));
      route.label = route_json.value(
          "label", std::string("Apron Bus Route ") + std::to_string(fallback_number));
      route.model = model_id_;
      route.loop = route_json.value("loop", false);
      route.autostart = route_json.value("autostart", true);
      route.speed_mps = std::clamp(route_json.value("speed_mps", speed_mps_), 0.5f, 15.0f);
      for (const auto& waypoint : route_json.at("waypoints")) {
        if (!waypoint.contains("latitude") || !waypoint.contains("longitude")) continue;
        double x{}, y{}, z{};
        XPLMWorldToLocal(waypoint.at("latitude").get<double>(),
                         waypoint.at("longitude").get<double>(), 0.0, &x, &y, &z);
        RoutePoint point{static_cast<float>(x),
                         terrain_y(static_cast<float>(x), static_cast<float>(z),
                                   static_cast<float>(y)),
                         static_cast<float>(z),
                         normalize_heading(waypoint.value("heading", 0.0f))};
        if (waypoint.value("handle_mode", std::string("auto")) == "aligned" &&
            waypoint.contains("handle_in_latitude") &&
            waypoint.contains("handle_in_longitude") &&
            waypoint.contains("handle_out_latitude") &&
            waypoint.contains("handle_out_longitude")) {
          double in_x{}, in_y{}, in_z{}, out_x{}, out_y{}, out_z{};
          XPLMWorldToLocal(waypoint.at("handle_in_latitude").get<double>(),
                           waypoint.at("handle_in_longitude").get<double>(), 0.0,
                           &in_x, &in_y, &in_z);
          XPLMWorldToLocal(waypoint.at("handle_out_latitude").get<double>(),
                           waypoint.at("handle_out_longitude").get<double>(), 0.0,
                           &out_x, &out_y, &out_z);
          point.handle_in_x = static_cast<float>(in_x - x);
          point.handle_in_z = static_cast<float>(in_z - z);
          point.handle_out_x = static_cast<float>(out_x - x);
          point.handle_out_z = static_cast<float>(out_z - z);
          point.custom_handles = true;
        }
        route.anchors.push_back(point);
      }
      if (route.anchors.size() < 2) continue;
      build_bezier_path(route.anchors, route.loop, route.path, route.distance_remaining);
      if (route.path.size() < 2) continue;
      route.current = route.path.front();
      route.instance = XPLMCreateInstance(object_, datarefs);
      if (!route.instance) continue;
      show_instance(route.instance, route.current, 0.0f, 0.0f);
      traffic_routes_.push_back(std::move(route));
      ++fallback_number;
    }
    status_ = traffic_routes_.empty()
                  ? "No valid saved traffic routes"
                  : "Loaded " + std::to_string(traffic_routes_.size()) + " traffic route(s)";
    return !traffic_routes_.empty();
  } catch (const std::exception& e) {
    status_ = std::string("Route load failed: ") + e.what();
    for (auto& route : traffic_routes_)
      if (route.instance) XPLMDestroyInstance(route.instance);
    traffic_routes_.clear();
    return false;
  }
}

float RouteEditor::terrain_y(float x, float z, float fallback) const {
  if (!probe_) return fallback;
  XPLMProbeInfo_t info{};
  info.structSize = sizeof(info);
  const auto result = XPLMProbeTerrainXYZ(probe_, x, fallback + 1000.0f, z, &info);
  return result == xplm_ProbeHitTerrain ? info.locationY + ground_offset_m_ : fallback;
}

void RouteEditor::show(float spin, float steering) {
  show_instance(instance_, current_, spin, steering);
}

void RouteEditor::show_instance(XPLMInstanceRef instance, const RoutePoint& point,
                                float spin, float steering) {
  if (!instance) return;
  XPLMDrawInfo_t draw{};
  draw.structSize = sizeof(draw);
  const float physical_heading = point.heading * pi / 180.0f;
  draw.x = point.x + std::sin(physical_heading) * rear_axle_to_origin_m_;
  draw.y = point.y;
  draw.z = point.z - std::cos(physical_heading) * rear_axle_to_origin_m_;
  draw.heading = normalize_heading(point.heading + heading_offset_deg_);
  const float data[] = {spin, std::clamp(steering * steering_multiplier_, -1.0f, 1.0f)};
  XPLMInstanceSetPosition(instance, &draw, data);
}

void RouteEditor::create_route(double latitude, double longitude, float heading) {
  if (state_ == RouteEditorState::Unavailable || !instance_) return;
  double x{}, y{}, z{};
  XPLMWorldToLocal(latitude, longitude, 0.0, &x, &y, &z);
  current_ = {static_cast<float>(x), terrain_y(static_cast<float>(x), static_cast<float>(z),
                                               static_cast<float>(y)),
              static_cast<float>(z), normalize_heading(heading)};
  points_.clear();
  points_.push_back(current_);
  size_t route_number = 1;
  for (;;) {
    char route_id[32];
    std::snprintf(route_id, sizeof(route_id), "bus_route_%02zu", route_number);
    const bool used = std::any_of(traffic_routes_.begin(), traffic_routes_.end(),
                                  [&](const TrafficRoute& route) {
                                    return route.id == route_id;
                                  });
    if (!used) {
      editing_route_id_ = route_id;
      editing_route_label_ = "Apron Bus Route " + std::to_string(route_number);
      break;
    }
    ++route_number;
  }
  spin_ = 0.0f;
  state_ = RouteEditorState::Editing;
  status_ = "Route created; first point added";
  show(spin_, 0.0f);
}

void RouteEditor::begin_planner(double latitude, double longitude, float heading) {
  create_route(latitude, longitude, heading);
  if (state_ != RouteEditorState::Editing) return;
  planner_center_x_ = current_.x;
  planner_center_y_ = current_.y;
  planner_center_z_ = current_.z;
  open_planner();
}

void RouteEditor::open_planner() {
  if (!instance_ || points_.empty()) return;
  build_bezier_path();
  int left{}, top{}, right{}, bottom{};
  XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
  if (!planner_window_) {
    XPLMCreateWindow_t params{};
    params.structSize = sizeof(params);
    params.left = left;
    params.top = top;
    params.right = right;
    params.bottom = bottom;
    params.visible = 1;
    params.drawWindowFunc = planner_draw;
    params.handleMouseClickFunc = planner_mouse_cb;
    params.handleRightClickFunc = planner_right_mouse_cb;
    params.handleKeyFunc = planner_key;
    params.handleCursorFunc = planner_cursor;
    params.handleMouseWheelFunc = planner_wheel_cb;
    params.refcon = this;
    params.layer = xplm_WindowLayerFlightOverlay;
    params.decorateAsFloatingWindow = xplm_WindowDecorationNone;
    planner_window_ = XPLMCreateWindowEx(&params);
  } else {
    XPLMSetWindowGeometry(planner_window_, left, top, right, bottom);
    XPLMSetWindowIsVisible(planner_window_, 1);
  }
  if (!planner_window_) {
    state_ = RouteEditorState::Editing;
    status_ = "Cannot create top-down planner overlay";
    return;
  }
  planner_active_ = true;
  state_ = RouteEditorState::Planning;
  status_ = "Planner active: click ground to add GPS markers";
  XPLMControlCamera(xplm_ControlCameraForever, camera_control, this);
}

void RouteEditor::close_planner() {
  if (!planner_active_) return;
  planner_active_ = false;
  planner_drag_active_ = false;
  handle_drag_anchor_ = -1;
  if (planner_window_) XPLMSetWindowIsVisible(planner_window_, 0);
  XPLMDontControlCamera();
  if (state_ == RouteEditorState::Planning) state_ = RouteEditorState::Editing;
}

int RouteEditor::camera_control(XPLMCameraPosition_t* position, int losing_control,
                                void* refcon) {
  auto* self = static_cast<RouteEditor*>(refcon);
  if (losing_control || !position || !self->planner_active_) {
    self->planner_active_ = false;
    if (self->planner_window_) XPLMSetWindowIsVisible(self->planner_window_, 0);
    if (self->state_ == RouteEditorState::Planning)
      self->state_ = RouteEditorState::Editing;
    return 0;
  }
  position->x = self->planner_center_x_;
  position->y = self->planner_center_y_ + self->planner_height_m_;
  position->z = self->planner_center_z_;
  position->pitch = -90.0f;
  position->heading = 0.0f;
  position->roll = 0.0f;
  position->zoom = 1.0f;
  return 1;
}

void RouteEditor::planner_draw(XPLMWindowID, void* refcon) {
  static_cast<RouteEditor*>(refcon)->draw_planner();
}

void RouteEditor::draw_planner() {
  if (!planner_active_ || !planner_window_) return;
  int left{}, top{}, right{}, bottom{};
  XPLMGetWindowGeometry(planner_window_, &left, &top, &right, &bottom);
  float title_color[] = {0.25f, 0.95f, 0.65f};
  float marker_color[] = {1.0f, 0.72f, 0.20f};
  float route_color[] = {0.55f, 1.0f, 0.70f};
  float handle_color[] = {0.30f, 0.80f, 1.0f};
  char title[] = "SSA BEZIER ROUTE | CLICK: ADD | RMB HOLD+DRAG ANCHOR: CURVE | SHIFT+MMB: PAN";
  XPLMDrawString(title_color, left + 24, top - 30, title, nullptr,
                 xplmFont_Proportional);
  auto draw_button = [&](int button_left, int button_top, int button_right,
                         int button_bottom, const char* label) {
    XPLMDrawTranslucentDarkBox(button_left, button_top, button_right, button_bottom);
    XPLMDrawString(title_color, button_left + 12, button_bottom + 10,
                   const_cast<char*>(label), nullptr, xplmFont_Proportional);
  };
  draw_button(left + 24, top - 42, left + 110, top - 74, "UNDO");
  draw_button(left + 124, top - 42, left + 210, top - 74, "TEST");
  draw_button(left + 224, top - 42, left + 310, top - 74, "SAVE");
  draw_button(right - 110, top - 42, right - 24, top - 74, "EXIT");
  draw_button(right - 100, top - 92, right - 64, top - 120, "^");
  draw_button(right - 140, top - 124, right - 104, top - 152, "<");
  draw_button(right - 100, top - 124, right - 64, top - 152, "v");
  draw_button(right - 60, top - 124, right - 24, top - 152, ">");
  const bool has_custom_handles = std::any_of(
      points_.begin(), points_.end(), [](const RoutePoint& point) {
        return point.custom_handles;
      });
  draw_button(right - 180, top - 180, right - 24, top - 212,
              has_custom_handles ? "PATH: BEZIER CUSTOM" : "PATH: BEZIER AUTO");
  draw_button(right - 180, top - 220, right - 24, top - 252,
              loop_enabled_ ? "LOOP: ON" : "LOOP: OFF");
  const float width = static_cast<float>(std::max(1, right - left));
  const float height = static_cast<float>(std::max(1, top - bottom));
  const float half_width_m = planner_half_width();
  const float half_height_m = half_width_m * height / width;
  auto screen = [&](const RoutePoint& point, int& sx, int& sy) {
    sx = static_cast<int>(left + width * 0.5f +
                          (point.x - planner_center_x_) / half_width_m * width * 0.5f);
    sy = static_cast<int>(bottom + height * 0.5f -
                          (point.z - planner_center_z_) / half_height_m * height * 0.5f);
  };
  const auto& route_line = test_points_.size() > 1 ? test_points_ : points_;
  if (!route_line.empty()) {
    int last_x{}, last_y{};
    screen(route_line.front(), last_x, last_y);
    for (size_t i = 1; i < route_line.size(); ++i) {
      int x{}, y{};
      screen(route_line[i], x, y);
      if (std::hypot(x - last_x, y - last_y) >= 10.0) {
        char point[] = ".";
        XPLMDrawString(route_color, x, y, point, nullptr, xplmFont_Proportional);
        last_x = x;
        last_y = y;
      }
    }
  }
  auto draw_dotted_handle = [&](const RoutePoint& a, const RoutePoint& b) {
    int ax{}, ay{}, bx{}, by{};
    screen(a, ax, ay);
    screen(b, bx, by);
    const float pixels = std::hypot(static_cast<float>(bx - ax),
                                    static_cast<float>(by - ay));
    const int steps = std::max(1, static_cast<int>(pixels / 9.0f));
    for (int step = 0; step <= steps; ++step) {
      if ((step & 1) != 0) continue;
      const float t = static_cast<float>(step) / static_cast<float>(steps);
      char dot[] = ".";
      XPLMDrawString(handle_color,
                     static_cast<int>(ax + (bx - ax) * t),
                     static_cast<int>(ay + (by - ay) * t), dot, nullptr,
                     xplmFont_Proportional);
    }
  };
  for (const auto& point : points_) {
    if (!point.custom_handles) continue;
    RoutePoint handle_in = point;
    RoutePoint handle_out = point;
    handle_in.x += point.handle_in_x;
    handle_in.z += point.handle_in_z;
    handle_out.x += point.handle_out_x;
    handle_out.z += point.handle_out_z;
    draw_dotted_handle(handle_in, handle_out);
    int in_x{}, in_y{}, out_x{}, out_y{};
    screen(handle_in, in_x, in_y);
    screen(handle_out, out_x, out_y);
    char handle[] = "o";
    XPLMDrawString(handle_color, in_x - 3, in_y, handle, nullptr,
                   xplmFont_Proportional);
    XPLMDrawString(handle_color, out_x - 3, out_y, handle, nullptr,
                   xplmFont_Proportional);
  }
  for (size_t i = 0; i < points_.size(); ++i) {
    int sx{}, sy{};
    screen(points_[i], sx, sy);
    char marker[24];
    std::snprintf(marker, sizeof(marker), "[ A%zu ]", i + 1);
    XPLMDrawString(marker_color, sx - 10, sy, marker, nullptr,
                   xplmFont_Proportional);
  }
}

int RouteEditor::planner_mouse_cb(XPLMWindowID, int x, int y, XPLMMouseStatus status,
                                  void* refcon) {
  return static_cast<RouteEditor*>(refcon)->planner_mouse(x, y, status);
}

int RouteEditor::planner_mouse(int x, int y, XPLMMouseStatus status) {
  if (!planner_active_ || status != xplm_MouseDown) return 1;
  int left{}, top{}, right{}, bottom{};
  XPLMGetWindowGeometry(planner_window_, &left, &top, &right, &bottom);
  const bool toolbar_y = y >= top - 74 && y <= top - 42;
  if (toolbar_y) {
    if (x >= left + 24 && x <= left + 110) undo_point();
    else if (x >= left + 124 && x <= left + 210) {
      start_test();
      if (state_ == RouteEditorState::Testing) close_planner();
    } else if (x >= left + 224 && x <= left + 310) {
      state_ = RouteEditorState::Editing;
      if (save()) close_planner();
      else state_ = RouteEditorState::Planning;
    } else if (x >= right - 110 && x <= right - 24) {
      close_planner();
    }
    return 1;
  }
  const float pan_step = std::max(5.0f, planner_height_m_ * 0.08f);
  if (x >= right - 100 && x <= right - 64 && y >= top - 120 && y <= top - 92) {
    planner_center_z_ -= pan_step;
    return 1;
  }
  if (x >= right - 140 && x <= right - 104 && y >= top - 152 && y <= top - 124) {
    planner_center_x_ -= pan_step;
    return 1;
  }
  if (x >= right - 100 && x <= right - 64 && y >= top - 152 && y <= top - 124) {
    planner_center_z_ += pan_step;
    return 1;
  }
  if (x >= right - 60 && x <= right - 24 && y >= top - 152 && y <= top - 124) {
    planner_center_x_ += pan_step;
    return 1;
  }
  if (y >= top - 252 && y <= top - 220) {
    if (x >= right - 180 && x <= right - 24) toggle_loop();
    return 1;
  }
  if (x >= right - 190 && y <= top - 170 && y >= top - 260) return 1;
  // Keep the title/toolbar band from accidentally creating an anchor.
  if (y > top - 90) return 1;
  const float width = static_cast<float>(std::max(1, right - left));
  const float height = static_cast<float>(std::max(1, top - bottom));
  const float half_width_m = planner_half_width();
  const float half_height_m = half_width_m * height / width;
  const float world_x = planner_center_x_ +
                        ((x - left) / width * 2.0f - 1.0f) * half_width_m;
  const float world_z = planner_center_z_ -
                        ((y - bottom) / height * 2.0f - 1.0f) * half_height_m;
  add_point_at(world_x, world_z);
  return 1;
}

int RouteEditor::planner_right_mouse_cb(XPLMWindowID, int x, int y,
                                        XPLMMouseStatus status, void* refcon) {
  return static_cast<RouteEditor*>(refcon)->planner_right_mouse(x, y, status);
}

int RouteEditor::planner_right_mouse(int x, int y, XPLMMouseStatus status) {
  if (!planner_active_ || !planner_window_) return 1;
  if (status == xplm_MouseUp) {
    if (handle_drag_anchor_ >= 0)
      status_ = "Bezier handle saved; right-drag again to adjust";
    handle_drag_anchor_ = -1;
    return 1;
  }
  int left{}, top{}, right{}, bottom{};
  XPLMGetWindowGeometry(planner_window_, &left, &top, &right, &bottom);
  const float width = static_cast<float>(std::max(1, right - left));
  const float height = static_cast<float>(std::max(1, top - bottom));
  const float half_width_m = planner_half_width();
  const float half_height_m = half_width_m * height / width;
  auto screen = [&](const RoutePoint& point, int& sx, int& sy) {
    sx = static_cast<int>(left + width * 0.5f +
                          (point.x - planner_center_x_) / half_width_m * width * 0.5f);
    sy = static_cast<int>(bottom + height * 0.5f -
                          (point.z - planner_center_z_) / half_height_m * height * 0.5f);
  };
  if (status == xplm_MouseDown) {
    handle_drag_anchor_ = -1;
    float nearest = 19.0f;
    for (size_t i = 0; i < points_.size(); ++i) {
      int sx{}, sy{};
      screen(points_[i], sx, sy);
      const float distance = std::hypot(static_cast<float>(x - sx),
                                        static_cast<float>(y - sy));
      if (distance < nearest) {
        nearest = distance;
        handle_drag_anchor_ = static_cast<int>(i);
      }
    }
    if (handle_drag_anchor_ < 0) status_ = "Right-click directly on an anchor";
    return 1;
  }
  if (status != xplm_MouseDrag || handle_drag_anchor_ < 0 ||
      static_cast<size_t>(handle_drag_anchor_) >= points_.size()) return 1;
  const float world_x = planner_center_x_ +
                        ((x - left) / width * 2.0f - 1.0f) * half_width_m;
  const float world_z = planner_center_z_ -
                        ((y - bottom) / height * 2.0f - 1.0f) * half_height_m;
  auto& point = points_[static_cast<size_t>(handle_drag_anchor_)];
  point.handle_out_x = world_x - point.x;
  point.handle_out_z = world_z - point.z;
  point.handle_in_x = -point.handle_out_x;
  point.handle_in_z = -point.handle_out_z;
  point.custom_handles = true;
  build_bezier_path();
  status_ = "Adjusting aligned Bezier handle";
  return 1;
}

void RouteEditor::planner_key(XPLMWindowID, char key, XPLMKeyFlags flags, char virtual_key,
                              void* refcon, int losing_focus) {
  auto* self = static_cast<RouteEditor*>(refcon);
  if (!self || losing_focus || !self->planner_active_ || (flags & xplm_UpFlag)) return;
  const float step = std::max(5.0f, self->planner_height_m_ * 0.08f);
  const auto code = static_cast<unsigned char>(virtual_key != 0 ? virtual_key : key);
  switch (code) {
    case XPLM_KEY_LEFT: self->planner_center_x_ -= step; break;
    case XPLM_KEY_RIGHT: self->planner_center_x_ += step; break;
    case XPLM_KEY_UP: self->planner_center_z_ -= step; break;
    case XPLM_KEY_DOWN: self->planner_center_z_ += step; break;
    default: break;
  }
}

XPLMCursorStatus RouteEditor::planner_cursor(XPLMWindowID, int, int, void*) {
  return xplm_CursorArrow;
}

int RouteEditor::planner_wheel_cb(XPLMWindowID, int x, int y, int wheel, int clicks,
                                  void* refcon) {
  return static_cast<RouteEditor*>(refcon)->planner_wheel(x, y, wheel, clicks);
}

int RouteEditor::planner_wheel(int, int, int wheel, int clicks) {
  if (!planner_active_ || wheel != 0) return 1;
  planner_height_m_ = std::clamp(planner_height_m_ - clicks * 12.0f, 50.0f, 500.0f);
  return 1;
}

float RouteEditor::planner_half_width() const {
  const float field_of_view = field_of_view_ref_
                                  ? std::clamp(XPLMGetDataf(field_of_view_ref_), 20.0f, 120.0f)
                                  : 60.0f;
  return planner_height_m_ * std::tan(field_of_view * pi / 360.0f);
}

void RouteEditor::update_planner_drag() {
#ifdef _WIN32
  if (!planner_active_) {
    planner_drag_active_ = false;
    return;
  }
  const bool dragging = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0 &&
                        (GetAsyncKeyState(VK_MBUTTON) & 0x8000) != 0;
  int mouse_x{}, mouse_y{};
  XPLMGetMouseLocationGlobal(&mouse_x, &mouse_y);
  if (!dragging) {
    planner_drag_active_ = false;
    return;
  }
  if (!planner_drag_active_) {
    planner_drag_active_ = true;
    planner_drag_x_ = mouse_x;
    planner_drag_y_ = mouse_y;
    return;
  }
  int left{}, top{}, right{}, bottom{};
  XPLMGetWindowGeometry(planner_window_, &left, &top, &right, &bottom);
  const float width = static_cast<float>(std::max(1, right - left));
  const float height = static_cast<float>(std::max(1, top - bottom));
  const float metres_per_x = planner_half_width() * 2.0f / width;
  const float metres_per_y = planner_half_width() * 2.0f * height / width / height;
  planner_center_x_ -= (mouse_x - planner_drag_x_) * metres_per_x;
  planner_center_z_ += (mouse_y - planner_drag_y_) * metres_per_y;
  planner_drag_x_ = mouse_x;
  planner_drag_y_ = mouse_y;
#else
  planner_drag_active_ = false;
#endif
}

void RouteEditor::add_point_at(float x, float z) {
  if (state_ != RouteEditorState::Planning) return;
  RoutePoint point{x, terrain_y(x, z, current_.y), z, current_.heading};
  if (!points_.empty()) {
    const auto& previous = points_.back();
    point.heading = normalize_heading(std::atan2(point.x - previous.x,
                                                  -(point.z - previous.z)) * 180.0f / pi);
  }
  points_.push_back(point);
  build_bezier_path();
  current_ = point;
  status_ = "Bezier anchor " + std::to_string(points_.size()) + " added";
  show(spin_, 0.0f);
}

void RouteEditor::move(float metres) {
  if (state_ != RouteEditorState::Editing) return;
  const float angle = current_.heading * pi / 180.0f;
  current_.x += std::sin(angle) * metres;
  current_.z -= std::cos(angle) * metres;
  current_.y = terrain_y(current_.x, current_.z, current_.y);
  spin_ += std::abs(metres) / 3.0f;
  spin_ -= std::floor(spin_);
  status_ = metres >= 0.0f ? "Bus moved forward" : "Bus moved backward";
  show(spin_, 0.0f);
}

void RouteEditor::turn(float degrees) {
  if (state_ != RouteEditorState::Editing) return;
  current_.heading = normalize_heading(current_.heading + degrees);
  status_ = degrees < 0.0f ? "Bus turned left" : "Bus turned right";
  show(spin_, degrees < 0.0f ? -1.0f : 1.0f);
}

void RouteEditor::add_point() {
  if (state_ != RouteEditorState::Editing) return;
  points_.push_back(current_);
  build_bezier_path();
  status_ = "Anchor " + std::to_string(points_.size()) + " added";
  show(spin_, 0.0f);
}

void RouteEditor::undo_point() {
  if ((state_ != RouteEditorState::Editing && state_ != RouteEditorState::Planning) ||
      points_.size() <= 1) return;
  points_.pop_back();
  build_bezier_path();
  current_ = points_.back();
  status_ = "Last anchor removed";
  show(spin_, 0.0f);
}

float RouteEditor::heading_delta(float from, float to) {
  float result = normalize_heading(to) - normalize_heading(from);
  if (result > 180.0f) result -= 360.0f;
  if (result < -180.0f) result += 360.0f;
  return result;
}

void RouteEditor::build_bezier_path() {
  build_bezier_path(points_, loop_enabled_, test_points_, test_distance_remaining_);
}

void RouteEditor::build_bezier_path(const std::vector<RoutePoint>& anchors, bool loop,
                                    std::vector<RoutePoint>& path,
                                    std::vector<float>& distance_remaining) {
  path.clear();
  distance_remaining.clear();
  if (anchors.empty()) return;
  if (anchors.size() == 1) {
    path = anchors;
    return;
  }
  const bool closed = loop && anchors.size() >= 3;
  const size_t segment_count = closed ? anchors.size() : anchors.size() - 1;
  auto anchor = [&](long long index) -> const RoutePoint& {
    const long long count = static_cast<long long>(anchors.size());
    if (closed) {
      index %= count;
      if (index < 0) index += count;
      return anchors[static_cast<size_t>(index)];
    }
    return anchors[static_cast<size_t>(std::clamp<long long>(index, 0, count - 1))];
  };
  path.push_back(anchors.front());
  for (size_t segment = 0; segment < segment_count; ++segment) {
    const auto& p0 = anchor(static_cast<long long>(segment) - 1);
    const auto& p1 = anchor(static_cast<long long>(segment));
    const auto& p2 = anchor(static_cast<long long>(segment) + 1);
    const auto& p3 = anchor(static_cast<long long>(segment) + 2);
    const float c1x = p1.custom_handles ? p1.x + p1.handle_out_x
                                        : p1.x + (p2.x - p0.x) / 6.0f;
    const float c1y = p1.y + (p2.y - p0.y) / 6.0f;
    const float c1z = p1.custom_handles ? p1.z + p1.handle_out_z
                                        : p1.z + (p2.z - p0.z) / 6.0f;
    const float c2x = p2.custom_handles ? p2.x + p2.handle_in_x
                                        : p2.x - (p3.x - p1.x) / 6.0f;
    const float c2y = p2.y - (p3.y - p1.y) / 6.0f;
    const float c2z = p2.custom_handles ? p2.z + p2.handle_in_z
                                        : p2.z - (p3.z - p1.z) / 6.0f;
    const float chord = std::hypot(p2.x - p1.x, p2.z - p1.z);
    if (chord < 0.01f) continue;
    const int steps = std::max(4, static_cast<int>(std::ceil(chord / 0.35f)));
    for (int step = 1; step <= steps; ++step) {
      const float t = static_cast<float>(step) / static_cast<float>(steps);
      const float u = 1.0f - t;
      const float x = u * u * u * p1.x + 3.0f * u * u * t * c1x +
                      3.0f * u * t * t * c2x + t * t * t * p2.x;
      const float fallback_y = u * u * u * p1.y + 3.0f * u * u * t * c1y +
                               3.0f * u * t * t * c2y + t * t * t * p2.y;
      const float z = u * u * u * p1.z + 3.0f * u * u * t * c1z +
                      3.0f * u * t * t * c2z + t * t * t * p2.z;
      path.push_back({x, terrain_y(x, z, fallback_y), z, 0.0f});
    }
  }
  size_t tangent_span = std::max<size_t>(2, static_cast<size_t>(
      std::lround(body_lookahead_m_ / 0.8f)));
  const size_t unique_count = closed && path.size() > 1 ? path.size() - 1 : path.size();
  if (closed) tangent_span = std::min(tangent_span, std::max<size_t>(1, (unique_count - 1) / 2));
  for (size_t i = 0; i < unique_count; ++i) {
    const size_t behind = closed ? (i + unique_count - tangent_span) % unique_count
                                 : (i > tangent_span ? i - tangent_span : 0);
    const size_t ahead = closed ? (i + tangent_span) % unique_count
                                : std::min(i + tangent_span, unique_count - 1);
    const float dx = path[ahead].x - path[behind].x;
    const float dz = path[ahead].z - path[behind].z;
    path[i].heading = normalize_heading(std::atan2(dx, -dz) * 180.0f / pi);
  }
  if (closed) path.back().heading = path.front().heading;
  distance_remaining.assign(path.size(), 0.0f);
  for (size_t i = path.size(); i > 1; --i) {
    const auto& a = path[i - 2];
    const auto& b = path[i - 1];
    distance_remaining[i - 2] = distance_remaining[i - 1] +
                                std::hypot(b.x - a.x, b.z - a.z);
  }
}

void RouteEditor::start_test() {
  if ((state_ != RouteEditorState::Editing && state_ != RouteEditorState::Planning) ||
      points_.size() < 2) {
    status_ = "Add at least two anchors";
    return;
  }
  return_to_planner_after_test_ = state_ == RouteEditorState::Planning;
  if (loop_enabled_ && points_.size() < 3) {
    status_ = "Loop needs at least three anchors";
    return;
  }
  build_bezier_path();
  if (test_points_.size() < 2) {
    return_to_planner_after_test_ = false;
    status_ = "Route points are too close together";
    return;
  }
  current_ = test_points_.front();
  test_index_ = 1;
  spin_ = 0.0f;
  current_speed_mps_ = 0.0f;
  display_steering_ = 0.0f;
  state_ = RouteEditorState::Testing;
  status_ = "Route test running";
  show(spin_, 0.0f);
}

void RouteEditor::start_saved_route(size_t index) {
  if (index >= traffic_routes_.size()) return;
  auto& route = traffic_routes_[index];
  if (!route.instance || route.path.size() < 2 || route.running) return;
  route.current = route.path.front();
  route.path_index = 1;
  route.current_speed_mps = 0.0f;
  route.spin = 0.0f;
  route.steering = 0.0f;
  route.running = true;
  show_instance(route.instance, route.current, route.spin, route.steering);
  status_ = "Running: " + route.label;
}

void RouteEditor::stop_saved_route(size_t index) {
  if (index >= traffic_routes_.size()) return;
  auto& route = traffic_routes_[index];
  route.running = false;
  route.current_speed_mps = 0.0f;
  route.steering = 0.0f;
  show_instance(route.instance, route.current, route.spin, 0.0f);
  status_ = "Stopped: " + route.label;
}

void RouteEditor::start_all_saved_routes() {
  for (size_t i = 0; i < traffic_routes_.size(); ++i) start_saved_route(i);
  if (!traffic_routes_.empty()) status_ = "All background traffic started";
}

void RouteEditor::stop_all_saved_routes() {
  for (size_t i = 0; i < traffic_routes_.size(); ++i) stop_saved_route(i);
  if (!traffic_routes_.empty()) status_ = "All background traffic stopped";
}

void RouteEditor::toggle_loop() {
  if (state_ != RouteEditorState::Planning && state_ != RouteEditorState::Editing) return;
  loop_enabled_ = !loop_enabled_;
  build_bezier_path();
  status_ = loop_enabled_ ? "Loop animation enabled" : "Loop animation disabled";
}

void RouteEditor::stop_test() {
  if (state_ != RouteEditorState::Testing) return;
  const bool resume_planner = return_to_planner_after_test_;
  state_ = RouteEditorState::Editing;
  status_ = "Route test stopped";
  return_to_planner_after_test_ = false;
  display_steering_ = 0.0f;
  current_speed_mps_ = 0.0f;
  show(spin_, 0.0f);
  if (resume_planner) {
    open_planner();
    status_ = "Route test stopped; planner resumed";
  }
}

void RouteEditor::update_traffic_route(TrafficRoute& route, float elapsed_seconds) {
  if (!route.running || route.path_index >= route.path.size() || elapsed_seconds <= 0.0f)
    return;
  const auto& target = route.path[route.path_index];
  const float dx = target.x - route.current.x;
  const float dz = target.z - route.current.z;
  const float distance = std::hypot(dx, dz);
  size_t lookahead_index = route.path_index;
  float lookahead_distance = 0.0f;
  size_t lookahead_guard = 0;
  while (lookahead_distance < body_lookahead_m_ &&
         lookahead_guard++ < route.path.size()) {
    size_t next_index = lookahead_index + 1;
    if (next_index >= route.path.size()) {
      if (!route.loop) break;
      next_index = 1;
    }
    const auto& a = route.path[lookahead_index];
    const auto& b = route.path[next_index];
    lookahead_distance += std::hypot(b.x - a.x, b.z - a.z);
    lookahead_index = next_index;
  }
  const auto& body_target = route.path[lookahead_index];
  const float desired_heading = normalize_heading(
      std::atan2(body_target.x - route.current.x,
                 -(body_target.z - route.current.z)) * 180.0f / pi);
  float delta = heading_delta(route.current.heading, desired_heading);
  if (std::abs(delta) < 0.25f) delta = 0.0f;
  const float heading_blend = 1.0f - std::exp(-body_heading_response_ * elapsed_seconds);
  route.current.heading = normalize_heading(
      route.current.heading + std::clamp(delta * heading_blend,
                                         -35.0f * elapsed_seconds,
                                         35.0f * elapsed_seconds));
  const float route_curve = heading_delta(target.heading, body_target.heading);
  const float corner_speed =
      std::clamp(1.0f - std::abs(route_curve) / 80.0f, 0.40f, 1.0f);
  float target_speed = route.speed_mps * corner_speed;
  if (!route.loop && route.path_index < route.distance_remaining.size()) {
    const float remaining = distance + route.distance_remaining[route.path_index];
    target_speed = std::min(target_speed, std::sqrt(2.0f * braking_mps2_ * remaining));
  }
  route.current_speed_mps += std::clamp(
      target_speed - route.current_speed_mps,
      -braking_mps2_ * elapsed_seconds,
      acceleration_mps2_ * elapsed_seconds);
  float travel = std::max(0.0f, route.current_speed_mps * elapsed_seconds);
  float travelled = 0.0f;
  while (route.running) {
    if (route.path_index >= route.path.size()) {
      if (route.loop) {
        route.current = route.path.front();
        route.path_index = 1;
      } else {
        route.running = false;
        route.current_speed_mps = 0.0f;
        route.steering = 0.0f;
        status_ = "Route complete: " + route.label;
        show_instance(route.instance, route.current, route.spin, 0.0f);
        return;
      }
    }
    const auto& movement_target = route.path[route.path_index];
    const float move_x = movement_target.x - route.current.x;
    const float move_z = movement_target.z - route.current.z;
    const float move_distance = std::hypot(move_x, move_z);
    if (move_distance <= 0.0001f) {
      route.current.x = movement_target.x;
      route.current.y = movement_target.y;
      route.current.z = movement_target.z;
      ++route.path_index;
      continue;
    }
    if (travel <= 0.00001f) break;
    if (travel >= move_distance) {
      route.current.x = movement_target.x;
      route.current.y = movement_target.y;
      route.current.z = movement_target.z;
      travel -= move_distance;
      travelled += move_distance;
      ++route.path_index;
    } else {
      const float ratio = travel / move_distance;
      route.current.x += move_x * ratio;
      route.current.z += move_z * ratio;
      route.current.y += (movement_target.y - route.current.y) * ratio;
      travelled += travel;
      travel = 0.0f;
    }
  }
  route.spin += travelled / 3.0f;
  route.spin -= std::floor(route.spin);
  const float curvature = (route_curve * pi / 180.0f) /
                          std::max(lookahead_distance, 0.5f);
  const float steering_angle = std::atan(wheelbase_m_ * curvature) * 180.0f / pi;
  float target_steering =
      std::clamp(steering_angle / max_steering_deg_, -1.0f, 1.0f);
  if (std::abs(route_curve) < 0.8f) target_steering = 0.0f;
  const float steering_blend = 1.0f - std::exp(-6.0f * elapsed_seconds);
  route.steering += (target_steering - route.steering) * steering_blend;
  show_instance(route.instance, route.current, route.spin, route.steering);
}

void RouteEditor::update(float elapsed_seconds) {
  update_planner_drag();
  if (route_load_pending_) {
    route_load_delay_seconds_ += std::clamp(elapsed_seconds, 0.0f, 0.25f);
    if (route_load_delay_seconds_ < 1.0f) return;
    route_load_pending_ = false;
    route_load_delay_seconds_ = 0.0f;
    if (!load_saved_route()) {
      status_ = "Ready: " + model_label_ + " | No saved route";
    } else {
      for (size_t i = 0; i < traffic_routes_.size(); ++i)
        if (traffic_routes_[i].autostart) start_saved_route(i);
    }
  }
  const float traffic_elapsed = std::clamp(elapsed_seconds, 0.0f, 0.10f);
  for (auto& route : traffic_routes_)
    if (route.running) update_traffic_route(route, traffic_elapsed);
  if (state_ != RouteEditorState::Testing || test_index_ >= test_points_.size()) return;
  elapsed_seconds = std::clamp(elapsed_seconds, 0.0f, 0.10f);
  if (elapsed_seconds <= 0.0f) return;
  const auto& target = test_points_[test_index_];
  const float dx = target.x - current_.x;
  const float dz = target.z - current_.z;
  const float distance = std::sqrt(dx * dx + dz * dz);
  size_t body_lookahead_index = test_index_;
  float lookahead_distance = 0.0f;
  size_t lookahead_guard = 0;
  while (lookahead_distance < body_lookahead_m_ &&
         lookahead_guard++ < test_points_.size()) {
    size_t next_index = body_lookahead_index + 1;
    if (next_index >= test_points_.size()) {
      if (!loop_enabled_) break;
      next_index = 1;
    }
    const auto& a = test_points_[body_lookahead_index];
    const auto& b = test_points_[next_index];
    lookahead_distance += std::hypot(b.x - a.x, b.z - a.z);
    body_lookahead_index = next_index;
  }
  const auto& body_target = test_points_[body_lookahead_index];
  const float body_dx = body_target.x - current_.x;
  const float body_dz = body_target.z - current_.z;
  const float desired_heading = normalize_heading(std::atan2(body_dx, -body_dz) * 180.0f / pi);
  float delta = heading_delta(current_.heading, desired_heading);
  if (std::abs(delta) < 0.25f) delta = 0.0f;
  const float heading_blend = 1.0f - std::exp(-body_heading_response_ * elapsed_seconds);
  const float turn_step = std::clamp(delta * heading_blend,
                                     -35.0f * elapsed_seconds,
                                     35.0f * elapsed_seconds);
  current_.heading = normalize_heading(current_.heading + turn_step);
  const float route_curve = heading_delta(target.heading, body_target.heading);
  const float corner_speed = std::clamp(1.0f - std::abs(route_curve) / 80.0f, 0.40f, 1.0f);
  float target_speed = speed_mps_ * corner_speed;
  if (!loop_enabled_ && test_index_ < test_distance_remaining_.size()) {
    const float remaining = distance + test_distance_remaining_[test_index_];
    target_speed = std::min(target_speed, std::sqrt(2.0f * braking_mps2_ * remaining));
  }
  const float speed_delta = target_speed - current_speed_mps_;
  current_speed_mps_ += std::clamp(speed_delta,
                                   -braking_mps2_ * elapsed_seconds,
                                   acceleration_mps2_ * elapsed_seconds);
  float travel = std::max(0.0f, current_speed_mps_ * elapsed_seconds);
  float travelled = 0.0f;
  while (state_ == RouteEditorState::Testing) {
    if (test_index_ >= test_points_.size()) {
      if (loop_enabled_) {
        current_.x = test_points_.front().x;
        current_.y = test_points_.front().y;
        current_.z = test_points_.front().z;
        test_index_ = 1;
        status_ = "Route loop running";
      } else {
        current_speed_mps_ = 0.0f;
        display_steering_ = 0.0f;
        state_ = RouteEditorState::Editing;
        status_ = "Route test complete";
        show(spin_, 0.0f);
        if (return_to_planner_after_test_) {
          return_to_planner_after_test_ = false;
          open_planner();
          status_ = "Route test complete; planner resumed";
        }
        return;
      }
    }
    const auto& movement_target = test_points_[test_index_];
    const float move_x = movement_target.x - current_.x;
    const float move_z = movement_target.z - current_.z;
    const float move_distance = std::hypot(move_x, move_z);
    if (move_distance <= 0.0001f) {
      current_.x = movement_target.x;
      current_.y = movement_target.y;
      current_.z = movement_target.z;
      ++test_index_;
      continue;
    }
    if (travel <= 0.00001f) break;
    if (travel >= move_distance) {
      current_.x = movement_target.x;
      current_.y = movement_target.y;
      current_.z = movement_target.z;
      travel -= move_distance;
      travelled += move_distance;
      ++test_index_;
    } else {
      const float ratio = travel / move_distance;
      current_.x += move_x * ratio;
      current_.z += move_z * ratio;
      current_.y += (movement_target.y - current_.y) * ratio;
      travelled += travel;
      travel = 0.0f;
    }
  }
  spin_ += travelled / 3.0f;
  spin_ -= std::floor(spin_);
  const float curvature = (route_curve * pi / 180.0f) /
                          std::max(lookahead_distance, 0.5f);
  const float steering_angle = std::atan(wheelbase_m_ * curvature) * 180.0f / pi;
  float target_steering =
      std::clamp(steering_angle / max_steering_deg_, -1.0f, 1.0f);
  if (std::abs(route_curve) < 0.8f) target_steering = 0.0f;
  const float steering_blend = 1.0f - std::exp(-6.0f * elapsed_seconds);
  display_steering_ += (target_steering - display_steering_) * steering_blend;
  show(spin_, display_steering_);
}

bool RouteEditor::save() {
  if (state_ != RouteEditorState::Editing || points_.size() < 2) {
    status_ = "Add at least two anchors";
    return false;
  }
  if (loop_enabled_ && points_.size() < 3) {
    status_ = "Loop needs at least three anchors";
    return false;
  }
  try {
    auto serialize_route = [&](const std::string& id, const std::string& label,
                               bool loop, bool autostart, float route_speed,
                               const std::vector<RoutePoint>& anchors) {
      json item = {{"id", id}, {"label", label}, {"model", model_id_},
                   {"path_type", "bezier"}, {"loop", loop},
                   {"autostart", autostart}, {"speed_mps", route_speed}};
      item["waypoints"] = json::array();
      for (const auto& point : anchors) {
        double latitude{}, longitude{}, altitude{};
        XPLMLocalToWorld(point.x, point.y, point.z, &latitude, &longitude, &altitude);
        json waypoint = {{"latitude", latitude}, {"longitude", longitude},
                         {"heading", point.heading}, {"kind", "bezier_anchor"},
                         {"handle_mode", point.custom_handles ? "aligned" : "auto"}};
        if (point.custom_handles) {
          double in_lat{}, in_lon{}, in_alt{};
          double out_lat{}, out_lon{}, out_alt{};
          XPLMLocalToWorld(point.x + point.handle_in_x, point.y,
                           point.z + point.handle_in_z, &in_lat, &in_lon, &in_alt);
          XPLMLocalToWorld(point.x + point.handle_out_x, point.y,
                           point.z + point.handle_out_z, &out_lat, &out_lon, &out_alt);
          waypoint["handle_in_latitude"] = in_lat;
          waypoint["handle_in_longitude"] = in_lon;
          waypoint["handle_out_latitude"] = out_lat;
          waypoint["handle_out_longitude"] = out_lon;
        }
        item["waypoints"].push_back(std::move(waypoint));
      }
      return item;
    };
    json route;
    route["schema"] = 2;
    route["routes"] = json::array();
    for (const auto& existing : traffic_routes_) {
      if (existing.id == editing_route_id_) continue;
      route["routes"].push_back(serialize_route(
          existing.id, existing.label, existing.loop, existing.autostart,
          existing.speed_mps, existing.anchors));
    }
    route["routes"].push_back(serialize_route(
        editing_route_id_.empty() ? "bus_route_01" : editing_route_id_,
        editing_route_label_.empty() ? "Apron Bus Route 1" : editing_route_label_,
        loop_enabled_, true, speed_mps_, points_));
    std::ofstream output(route_path_);
    output << route.dump(2) << '\n';
    if (!output) throw std::runtime_error("Cannot write route file");
    const std::string saved_label = editing_route_label_;
    load_saved_route();
    for (size_t i = 0; i < traffic_routes_.size(); ++i)
      if (traffic_routes_[i].autostart) start_saved_route(i);
    points_.clear();
    test_points_.clear();
    test_distance_remaining_.clear();
    loop_enabled_ = false;
    editing_route_id_.clear();
    editing_route_label_.clear();
    state_ = RouteEditorState::Idle;
    if (instance_) XPLMDestroyInstance(instance_);
    const char* datarefs[] = {"boldstudio31/ssa/vehicle/wheel_spin",
                              "boldstudio31/ssa/vehicle/steering", nullptr};
    instance_ = XPLMCreateInstance(object_, datarefs);
    status_ = "Saved " + saved_label + " | " +
              std::to_string(traffic_routes_.size()) + " route(s) total";
    return true;
  } catch (const std::exception& e) {
    status_ = e.what();
    return false;
  }
}

void RouteEditor::cancel() {
  if (state_ == RouteEditorState::Unavailable) return;
  if (planner_active_) close_planner();
  points_.clear();
  test_points_.clear();
  test_distance_remaining_.clear();
  return_to_planner_after_test_ = false;
  loop_enabled_ = false;
  editing_route_id_.clear();
  editing_route_label_.clear();
  handle_drag_anchor_ = -1;
  current_speed_mps_ = 0.0f;
  state_ = RouteEditorState::Idle;
  status_ = "Route editor idle";
  if (instance_) {
    XPLMDestroyInstance(instance_);
    const char* datarefs[] = {"boldstudio31/ssa/vehicle/wheel_spin",
                              "boldstudio31/ssa/vehicle/steering", nullptr};
    instance_ = XPLMCreateInstance(object_, datarefs);
  }
}

} // namespace ssa
