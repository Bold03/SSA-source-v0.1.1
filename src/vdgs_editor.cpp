#include "ssa/vdgs_editor.hpp"
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
// XPLMCreateInstance in the X-Plane SDK takes const char** (the pointer array
// itself is mutable), so this must not be a constexpr/const pointer array.
const char* instance_datarefs[] = {
    "boldstudio31/ssa/vdgs/active", "boldstudio31/ssa/vdgs/left",
    "boldstudio31/ssa/vdgs/right", "boldstudio31/ssa/vdgs/center",
    "boldstudio31/ssa/vdgs/slow", "boldstudio31/ssa/vdgs/stop",
    "boldstudio31/ssa/vdgs/lateral", "boldstudio31/ssa/vdgs/distance_ratio",
    nullptr};
}

VdgsEditor::~VdgsEditor() { unload(); }

float VdgsEditor::normalize_heading(float heading) {
  heading = std::fmod(heading, 360.0f);
  return heading < 0.0f ? heading + 360.0f : heading;
}

void VdgsEditor::unload() {
  if (preview_instance_) XPLMDestroyInstance(preview_instance_);
  if (preview_object_) XPLMUnloadObject(preview_object_);
  if (probe_) XPLMDestroyProbe(probe_);
  preview_instance_ = nullptr;
  preview_object_ = nullptr;
  probe_ = nullptr;
  for (auto& placement : placements_) {
    if (placement.instance) XPLMDestroyInstance(placement.instance);
    if (placement.object) XPLMUnloadObject(placement.object);
  }
  placements_.clear();
  state_ = VdgsEditorState::Unavailable;
}

bool VdgsEditor::load(const std::string& xplane_root) {
  unload();
  xplane_root_ = xplane_root;
  const fs::path custom = fs::path(xplane_root) / "Custom Scenery";
  try {
    if (!fs::exists(custom)) {
      status_ = "Custom Scenery folder not found";
      return false;
    }
    for (const auto& entry : fs::directory_iterator(custom)) {
      const fs::path config = entry.path() / "ssa.json";
      if (!entry.is_directory() || !fs::exists(config)) continue;
      load_saved(config.string());
      if (!absolute_object_path_.empty()) continue;

      std::ifstream input(config);
      const json root = json::parse(input);
      if (root.contains("vdgs_models") && root.at("vdgs_models").is_array() &&
          !root.at("vdgs_models").empty()) {
        const auto& model = root.at("vdgs_models").front();
        if (model.contains("object")) {
          const std::string relative = model.at("object").get<std::string>();
          const fs::path candidate = entry.path() / relative;
          if (fs::exists(candidate)) {
            scenery_directory_ = entry.path().string();
            config_path_ = config.string();
            relative_object_path_ = relative;
            absolute_object_path_ = candidate.string();
          }
        }
      }
      if (!absolute_object_path_.empty()) continue;
      for (const char* relative : {"object/VDGS.obj", "objects/VDGS.obj", "VDGS.obj"}) {
        const fs::path candidate = entry.path() / relative;
        if (!fs::exists(candidate)) continue;
        scenery_directory_ = entry.path().string();
        config_path_ = config.string();
        relative_object_path_ = relative;
        absolute_object_path_ = candidate.string();
        break;
      }
    }
    if (absolute_object_path_.empty()) {
      status_ = "Put VDGS.obj in scenery/object or configure vdgs_models";
      return false;
    }
    preview_object_ = XPLMLoadObject(absolute_object_path_.c_str());
    probe_ = XPLMCreateProbe(xplm_ProbeY);
    if (!preview_object_ || !probe_) {
      status_ = "Cannot load VDGS OBJ";
      unload();
      return false;
    }
    preview_instance_ = XPLMCreateInstance(preview_object_, instance_datarefs);
    if (!preview_instance_) {
      status_ = "Cannot create VDGS preview";
      unload();
      return false;
    }
    XPLMDrawInfo_t hidden{};
    hidden.structSize = sizeof(hidden);
    hidden.y = -10000.0f;
    const float hidden_data[] = {0.0f, 0.0f, 0.0f, 0.0f,
                                 0.0f, 0.0f, 0.5f, 1.0f};
    XPLMInstanceSetPosition(preview_instance_, &hidden, hidden_data);
    state_ = VdgsEditorState::Idle;
    status_ = "VDGS placement ready";
    return true;
  } catch (const std::exception& e) {
    const std::string message = e.what();
    unload();
    status_ = message;
    return false;
  }
}

bool VdgsEditor::load_saved(const std::string& config_path) {
  std::ifstream input(config_path);
  const json root = json::parse(input);
  if (!root.contains("objects") || !root.at("objects").is_array()) return true;
  for (const auto& item : root.at("objects")) {
    if (item.value("type", std::string()) != "parking_display" ||
        !item.contains("object")) continue;
    VdgsPlacement placement;
    placement.id = item.at("id").get<std::string>();
    placement.object_path =
        (fs::path(config_path).parent_path() /
         item.at("object").get<std::string>()).string();
    placement.latitude = item.at("latitude").get<double>();
    placement.longitude = item.at("longitude").get<double>();
    placement.altitude_m = item.value("altitude_m", 0.0);
    placement.heading = item.value(
        "heading", item.contains("vdgs")
                       ? item.at("vdgs").value("object_heading_deg", 0.0f)
                       : 0.0f);
    placement.data[6] = 0.5f;
    placement.data[7] = 1.0f;
    if (create_saved_instance(placement)) placements_.push_back(std::move(placement));
  }
  return true;
}

bool VdgsEditor::create_saved_instance(VdgsPlacement& placement) {
  placement.object = XPLMLoadObject(placement.object_path.c_str());
  if (!placement.object) return false;
  placement.instance = XPLMCreateInstance(placement.object, instance_datarefs);
  if (!placement.instance) {
    XPLMUnloadObject(placement.object);
    placement.object = nullptr;
    return false;
  }
  return true;
}

float VdgsEditor::terrain_y(float x, float z, float fallback) const {
  if (!probe_) return fallback;
  XPLMProbeInfo_t info{};
  info.structSize = sizeof(info);
  return XPLMProbeTerrainXYZ(probe_, x, fallback + 1000.0f, z, &info) ==
                 xplm_ProbeHitTerrain
             ? info.locationY
             : fallback;
}

void VdgsEditor::begin(double aircraft_latitude, double aircraft_longitude,
                       float aircraft_heading) {
  if (state_ == VdgsEditorState::Unavailable || !preview_instance_) return;
  double aircraft_x{}, aircraft_y{}, aircraft_z{};
  XPLMWorldToLocal(aircraft_latitude, aircraft_longitude, 0.0,
                   &aircraft_x, &aircraft_y, &aircraft_z);
  const float angle = aircraft_heading * pi / 180.0f;
  x_ = static_cast<float>(aircraft_x) + std::sin(angle) * 20.0f;
  z_ = static_cast<float>(aircraft_z) - std::cos(angle) * 20.0f;
  ground_y_ = terrain_y(x_, z_, static_cast<float>(aircraft_y));
  altitude_offset_m_ = 0.0f;
  y_ = ground_y_;
  heading_ = normalize_heading(aircraft_heading + 180.0f);
  double ignored_altitude{};
  XPLMLocalToWorld(x_, y_, z_, &latitude_, &longitude_, &ignored_altitude);
  altitude_m_ = ignored_altitude;
  state_ = VdgsEditorState::Placing;
  status_ = "Move, rotate and set altitude, then SAVE";
  show_preview();
}

void VdgsEditor::move_forward(float metres) {
  if (state_ != VdgsEditorState::Placing) return;
  const float angle = heading_ * pi / 180.0f;
  x_ += std::sin(angle) * metres;
  z_ -= std::cos(angle) * metres;
  ground_y_ = terrain_y(x_, z_, ground_y_);
  y_ = ground_y_ + altitude_offset_m_;
  show_preview();
}

void VdgsEditor::move_side(float metres) {
  if (state_ != VdgsEditorState::Placing) return;
  const float angle = heading_ * pi / 180.0f;
  x_ += std::cos(angle) * metres;
  z_ += std::sin(angle) * metres;
  ground_y_ = terrain_y(x_, z_, ground_y_);
  y_ = ground_y_ + altitude_offset_m_;
  show_preview();
}

void VdgsEditor::turn(float degrees) {
  if (state_ != VdgsEditorState::Placing) return;
  heading_ = normalize_heading(heading_ + degrees);
  show_preview();
}

void VdgsEditor::adjust_altitude(float metres) {
  if (state_ != VdgsEditorState::Placing) return;
  altitude_offset_m_ = std::clamp(altitude_offset_m_ + metres, -10.0f, 50.0f);
  y_ = ground_y_ + altitude_offset_m_;
  show_preview();
}

void VdgsEditor::show_preview() {
  if (!preview_instance_ || state_ != VdgsEditorState::Placing) return;
  XPLMDrawInfo_t draw{};
  draw.structSize = sizeof(draw);
  draw.x = x_;
  draw.y = y_;
  draw.z = z_;
  draw.heading = heading_;
  const float data[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.5f, 0.5f};
  XPLMInstanceSetPosition(preview_instance_, &draw, data);
  double ignored{};
  XPLMLocalToWorld(x_, y_, z_, &latitude_, &longitude_, &ignored);
  altitude_m_ = ignored;
}

void VdgsEditor::cancel() {
  if (state_ != VdgsEditorState::Placing) return;
  XPLMDrawInfo_t hidden{};
  hidden.structSize = sizeof(hidden);
  hidden.y = -10000.0f;
  const float data[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f};
  XPLMInstanceSetPosition(preview_instance_, &hidden, data);
  state_ = VdgsEditorState::Idle;
  status_ = "VDGS placement cancelled";
}

bool VdgsEditor::save() {
  if (state_ != VdgsEditorState::Placing || config_path_.empty()) return false;
  try {
    std::ifstream input(config_path_);
    json root = json::parse(input);
    if (!root.contains("objects") || !root.at("objects").is_array())
      root["objects"] = json::array();
    size_t number = 1;
    std::string id;
    for (;;) {
      id = "vdgs_" + (number < 10 ? std::string("0") : std::string()) +
           std::to_string(number);
      bool used = false;
      for (const auto& item : root.at("objects"))
        if (item.value("id", std::string()) == id) used = true;
      if (!used) break;
      ++number;
    }
    json item = {
        {"id", id}, {"label", "VDGS " + std::to_string(number)},
        {"type", "parking_display"}, {"object", relative_object_path_},
        {"latitude", latitude_}, {"longitude", longitude_},
        {"altitude_m", altitude_m_}, {"heading", heading_}, {"radius_m", 90.0f},
        {"dataref", "boldstudio31/ssa/animation/vdgs/" + id + "/state"}};
    item["vdgs"] = {{"object_heading_deg", heading_}, {"stop_distance_m", 18.0f},
                     {"acquisition_distance_m", 80.0f}, {"slow_distance_m", 12.0f},
                     {"stop_tolerance_m", 0.5f}, {"lateral_full_scale_m", 3.0f},
                     {"lateral_deadband_m", 0.15f},
                     {"lateral_stop_tolerance_m", 0.35f},
                     {"lateral_multiplier", -1.0f}};
    root["objects"].push_back(item);
    std::ofstream output(config_path_);
    output << root.dump(2) << '\n';
    if (!output) throw std::runtime_error("Cannot write ssa.json");

    VdgsPlacement placement;
    placement.id = id;
    placement.object_path = absolute_object_path_;
    placement.latitude = latitude_;
    placement.longitude = longitude_;
    placement.altitude_m = altitude_m_;
    placement.heading = heading_;
    placement.data[6] = 0.5f;
    placement.data[7] = 1.0f;
    if (create_saved_instance(placement)) placements_.push_back(std::move(placement));
    cancel();
    status_ = "Saved " + id + " to ssa.json";
    return true;
  } catch (const std::exception& e) {
    status_ = e.what();
    return false;
  }
}

void VdgsEditor::clear_guidance() {
  for (auto& placement : placements_) {
    placement.data.fill(0.0f);
    placement.data[6] = 0.5f;
    placement.data[7] = 1.0f;
  }
}

void VdgsEditor::set_guidance(const std::string& id,
                              const std::array<float, 8>& data) {
  const auto found = std::find_if(placements_.begin(), placements_.end(),
                                  [&](const auto& placement) { return placement.id == id; });
  if (found != placements_.end()) found->data = data;
}

void VdgsEditor::update() {
  for (auto& placement : placements_) {
    if (!placement.instance) continue;
    double x{}, y{}, z{};
    XPLMWorldToLocal(placement.latitude, placement.longitude, placement.altitude_m,
                     &x, &y, &z);
    XPLMDrawInfo_t draw{};
    draw.structSize = sizeof(draw);
    draw.x = static_cast<float>(x);
    draw.y = static_cast<float>(y);
    draw.z = static_cast<float>(z);
    draw.heading = placement.heading;
    XPLMInstanceSetPosition(placement.instance, &draw, placement.data.data());
  }
  if (state_ == VdgsEditorState::Placing) show_preview();
}

} // namespace ssa
