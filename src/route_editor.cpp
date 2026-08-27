#include "ssa/route_editor.hpp"
#include <XPLMGraphics.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

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
  if (object_) XPLMUnloadObject(object_);
  if (probe_) XPLMDestroyProbe(probe_);
  instance_ = nullptr;
  object_ = nullptr;
  probe_ = nullptr;
  planner_window_ = nullptr;
  points_.clear();
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
      heading_offset_deg_ = model.value("heading_offset_deg", 180.0f);
      steering_multiplier_ = model.value("steering_multiplier", -1.0f);
      smoothing_iterations_ = std::clamp(model.value("smoothing_iterations", 2), 0, 4);
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
      status_ = "Ready: " + model_label_;
      return true;
    }
  } catch (const std::exception& e) {
    status_ = e.what();
    return false;
  }
  status_ = "Add vehicle_models to ssa.json";
  return false;
}

float RouteEditor::terrain_y(float x, float z, float fallback) const {
  if (!probe_) return fallback;
  XPLMProbeInfo_t info{};
  info.structSize = sizeof(info);
  const auto result = XPLMProbeTerrainXYZ(probe_, x, fallback + 1000.0f, z, &info);
  return result == xplm_ProbeHitTerrain ? info.locationY + ground_offset_m_ : fallback;
}

void RouteEditor::show(float spin, float steering) {
  if (!instance_) return;
  XPLMDrawInfo_t draw{};
  draw.structSize = sizeof(draw);
  draw.x = current_.x;
  draw.y = current_.y;
  draw.z = current_.z;
  draw.heading = normalize_heading(current_.heading + heading_offset_deg_);
  const float data[] = {spin, std::clamp(steering * steering_multiplier_, -1.0f, 1.0f)};
  XPLMInstanceSetPosition(instance_, &draw, data);
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
  XPLMTakeKeyboardFocus(planner_window_);
  XPLMControlCamera(xplm_ControlCameraUntilViewChanges, camera_control, this);
}

void RouteEditor::close_planner() {
  if (!planner_active_) return;
  planner_active_ = false;
  XPLMTakeKeyboardFocus(nullptr);
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
  char title[] = "SSA ROUTE PLANNER | CLICK: ADD GPS POINT | ARROW KEYS: PAN | WHEEL: ZOOM";
  XPLMDrawString(title_color, left + 24, top - 30, title, nullptr,
                 xplmFont_Proportional);
  const int button_top = top - 42;
  const int button_bottom = top - 74;
  auto draw_button = [&](int button_left, int button_right, const char* label) {
    XPLMDrawTranslucentDarkBox(button_left, button_top, button_right, button_bottom);
    XPLMDrawString(title_color, button_left + 12, top - 63,
                   const_cast<char*>(label), nullptr, xplmFont_Proportional);
  };
  draw_button(left + 24, left + 110, "UNDO");
  draw_button(left + 124, left + 210, "TEST");
  draw_button(left + 224, left + 310, "SAVE");
  draw_button(right - 110, right - 24, "EXIT");
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
  for (size_t i = 1; i < points_.size(); ++i) {
    int x1{}, y1{}, x2{}, y2{};
    screen(points_[i - 1], x1, y1);
    screen(points_[i], x2, y2);
    const int dots = std::max(1, static_cast<int>(std::hypot(x2 - x1, y2 - y1) / 18.0));
    for (int dot = 1; dot < dots; ++dot) {
      const float ratio = static_cast<float>(dot) / static_cast<float>(dots);
      char point[] = ".";
      XPLMDrawString(route_color, static_cast<int>(x1 + (x2 - x1) * ratio),
                     static_cast<int>(y1 + (y2 - y1) * ratio), point, nullptr,
                     xplmFont_Proportional);
    }
  }
  for (size_t i = 0; i < points_.size(); ++i) {
    int sx{}, sy{};
    screen(points_[i], sx, sy);
    char marker[24];
    std::snprintf(marker, sizeof(marker), "[ %zu ]", i + 1);
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
      close_planner();
      start_test();
    } else if (x >= left + 224 && x <= left + 310) {
      state_ = RouteEditorState::Editing;
      save();
      close_planner();
    } else if (x >= right - 110 && x <= right - 24) {
      close_planner();
    }
    return 1;
  }
  // Keep the title/toolbar band from accidentally creating a waypoint.
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

void RouteEditor::planner_key(XPLMWindowID, char key, XPLMKeyFlags flags, char,
                              void* refcon, int losing_focus) {
  auto* self = static_cast<RouteEditor*>(refcon);
  if (!self || losing_focus || !self->planner_active_ || (flags & xplm_UpFlag)) return;
  const float step = std::max(5.0f, self->planner_height_m_ * 0.08f);
  switch (static_cast<unsigned char>(key)) {
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

void RouteEditor::add_point_at(float x, float z) {
  if (state_ != RouteEditorState::Planning) return;
  RoutePoint point{x, terrain_y(x, z, current_.y), z, current_.heading};
  if (!points_.empty()) {
    const auto& previous = points_.back();
    point.heading = normalize_heading(std::atan2(point.x - previous.x,
                                                  -(point.z - previous.z)) * 180.0f / pi);
  }
  points_.push_back(point);
  current_ = point;
  status_ = "GPS waypoint " + std::to_string(points_.size()) + " added";
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
  status_ = "Waypoint " + std::to_string(points_.size()) + " added";
  show(spin_, 0.0f);
}

void RouteEditor::undo_point() {
  if ((state_ != RouteEditorState::Editing && state_ != RouteEditorState::Planning) ||
      points_.size() <= 1) return;
  points_.pop_back();
  current_ = points_.back();
  status_ = "Last waypoint removed";
  show(spin_, 0.0f);
}

float RouteEditor::heading_delta(float from, float to) {
  float result = normalize_heading(to) - normalize_heading(from);
  if (result > 180.0f) result -= 360.0f;
  if (result < -180.0f) result += 360.0f;
  return result;
}

void RouteEditor::build_smooth_test_path() {
  test_points_ = points_;
  if (test_points_.size() >= 3) {
    for (int iteration = 0; iteration < smoothing_iterations_; ++iteration) {
      std::vector<RoutePoint> smooth;
      smooth.reserve(test_points_.size() * 2);
      smooth.push_back(test_points_.front());
      for (size_t i = 0; i + 1 < test_points_.size(); ++i) {
        const auto& a = test_points_[i];
        const auto& b = test_points_[i + 1];
        smooth.push_back({a.x * 0.75f + b.x * 0.25f,
                          a.y * 0.75f + b.y * 0.25f,
                          a.z * 0.75f + b.z * 0.25f, 0.0f});
        smooth.push_back({a.x * 0.25f + b.x * 0.75f,
                          a.y * 0.25f + b.y * 0.75f,
                          a.z * 0.25f + b.z * 0.75f, 0.0f});
      }
      smooth.push_back(test_points_.back());
      test_points_ = std::move(smooth);
    }
  }
  for (size_t i = 0; i + 1 < test_points_.size(); ++i) {
    const float dx = test_points_[i + 1].x - test_points_[i].x;
    const float dz = test_points_[i + 1].z - test_points_[i].z;
    test_points_[i].heading = normalize_heading(std::atan2(dx, -dz) * 180.0f / pi);
  }
  if (test_points_.size() > 1)
    test_points_.back().heading = test_points_[test_points_.size() - 2].heading;
}

void RouteEditor::start_test() {
  if ((state_ != RouteEditorState::Editing && state_ != RouteEditorState::Planning) ||
      points_.size() < 2) {
    status_ = "Add at least two waypoints";
    return;
  }
  build_smooth_test_path();
  current_ = test_points_.front();
  test_index_ = 1;
  spin_ = 0.0f;
  display_steering_ = 0.0f;
  state_ = RouteEditorState::Testing;
  status_ = "Route test running";
  show(spin_, 0.0f);
}

void RouteEditor::stop_test() {
  if (state_ != RouteEditorState::Testing) return;
  state_ = RouteEditorState::Editing;
  status_ = "Route test stopped";
  display_steering_ = 0.0f;
  show(spin_, 0.0f);
}

void RouteEditor::update(float elapsed_seconds) {
  if (state_ != RouteEditorState::Testing || test_index_ >= test_points_.size()) return;
  const auto& target = test_points_[test_index_];
  const float dx = target.x - current_.x;
  const float dz = target.z - current_.z;
  const float distance = std::sqrt(dx * dx + dz * dz);
  if (distance <= 0.05f) {
    current_.x = target.x;
    current_.y = target.y;
    current_.z = target.z;
    ++test_index_;
    if (test_index_ >= test_points_.size()) {
      state_ = RouteEditorState::Editing;
      status_ = "Route test complete";
      display_steering_ = 0.0f;
      show(spin_, 0.0f);
    }
    return;
  }
  const float desired_heading = normalize_heading(std::atan2(dx, -dz) * 180.0f / pi);
  const float delta = heading_delta(current_.heading, desired_heading);
  const float turn_step = std::clamp(delta, -60.0f * elapsed_seconds, 60.0f * elapsed_seconds);
  current_.heading = normalize_heading(current_.heading + turn_step);
  const float step = std::min(distance, speed_mps_ * elapsed_seconds);
  current_.x += dx / distance * step;
  current_.z += dz / distance * step;
  current_.y += (target.y - current_.y) * (step / distance);
  spin_ += step / 3.0f;
  spin_ -= std::floor(spin_);
  const float target_steering = std::clamp(delta / 35.0f, -1.0f, 1.0f);
  const float steering_blend = 1.0f - std::exp(-5.0f * elapsed_seconds);
  display_steering_ += (target_steering - display_steering_) * steering_blend;
  show(spin_, display_steering_);
}

bool RouteEditor::save() {
  if (state_ != RouteEditorState::Editing || points_.size() < 2) {
    status_ = "Add at least two waypoints";
    return false;
  }
  try {
    json route;
    route["schema"] = 1;
    route["routes"] = json::array();
    json item = {{"id", "bus_route_01"}, {"label", "Apron Bus Route 01"},
                 {"model", model_id_}, {"loop", true}, {"speed_mps", speed_mps_}};
    item["waypoints"] = json::array();
    for (const auto& point : points_) {
      double latitude{}, longitude{}, altitude{};
      XPLMLocalToWorld(point.x, point.y, point.z, &latitude, &longitude, &altitude);
      item["waypoints"].push_back({{"latitude", latitude}, {"longitude", longitude},
                                     {"heading", point.heading}});
    }
    route["routes"].push_back(std::move(item));
    std::ofstream output(route_path_);
    output << route.dump(2) << '\n';
    if (!output) throw std::runtime_error("Cannot write route file");
    status_ = "Saved: ssa_routes.json";
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
