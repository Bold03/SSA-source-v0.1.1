#include "ssa/route_editor.hpp"
#include <XPLMGraphics.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
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
  if (instance_) XPLMDestroyInstance(instance_);
  if (object_) XPLMUnloadObject(object_);
  if (probe_) XPLMDestroyProbe(probe_);
  instance_ = nullptr;
  object_ = nullptr;
  probe_ = nullptr;
  points_.clear();
  state_ = RouteEditorState::Unavailable;
}

bool RouteEditor::load(const std::string& xplane_root) {
  unload();
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
  draw.heading = normalize_heading(current_.heading);
  const float data[] = {spin, steering};
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
  if (state_ != RouteEditorState::Editing || points_.size() <= 1) return;
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

void RouteEditor::start_test() {
  if (state_ != RouteEditorState::Editing || points_.size() < 2) {
    status_ = "Add at least two waypoints";
    return;
  }
  current_ = points_.front();
  test_index_ = 1;
  spin_ = 0.0f;
  state_ = RouteEditorState::Testing;
  status_ = "Route test running";
  show(spin_, 0.0f);
}

void RouteEditor::stop_test() {
  if (state_ != RouteEditorState::Testing) return;
  state_ = RouteEditorState::Editing;
  status_ = "Route test stopped";
  show(spin_, 0.0f);
}

void RouteEditor::update(float elapsed_seconds) {
  if (state_ != RouteEditorState::Testing || test_index_ >= points_.size()) return;
  const auto& target = points_[test_index_];
  const float dx = target.x - current_.x;
  const float dz = target.z - current_.z;
  const float distance = std::sqrt(dx * dx + dz * dz);
  if (distance <= 0.05f) {
    current_ = target;
    ++test_index_;
    if (test_index_ >= points_.size()) {
      state_ = RouteEditorState::Editing;
      status_ = "Route test complete";
      show(spin_, 0.0f);
    }
    return;
  }
  const float desired_heading = normalize_heading(std::atan2(dx, -dz) * 180.0f / pi);
  const float delta = heading_delta(current_.heading, desired_heading);
  const float turn_step = std::clamp(delta, -45.0f * elapsed_seconds, 45.0f * elapsed_seconds);
  current_.heading = normalize_heading(current_.heading + turn_step);
  const float step = std::min(distance, speed_mps_ * elapsed_seconds);
  current_.x += dx / distance * step;
  current_.z += dz / distance * step;
  current_.y += (target.y - current_.y) * (step / distance);
  spin_ += step / 3.0f;
  spin_ -= std::floor(spin_);
  show(spin_, std::clamp(delta / 35.0f, -1.0f, 1.0f));
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
