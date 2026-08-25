#include "ssa/scenery.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace ssa {
namespace {
ServiceType parse_type(const std::string& value) {
  if (value == "jetway") return ServiceType::Jetway;
  if (value == "vehicle") return ServiceType::Vehicle;
  if (value == "ground_staff") return ServiceType::GroundStaff;
  if (value == "parking_display") return ServiceType::ParkingDisplay;
  if (value == "billboard") return ServiceType::Billboard;
  return ServiceType::Hangar;
}

double distance_m(double lat1, double lon1, double lat2, double lon2) {
  constexpr double earth = 6371000.0;
  constexpr double deg = 3.14159265358979323846 / 180.0;
  const double p1 = lat1 * deg, p2 = lat2 * deg;
  const double dp = (lat2 - lat1) * deg, dl = (lon2 - lon1) * deg;
  const double a = std::sin(dp / 2) * std::sin(dp / 2) +
                   std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
  return earth * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}
} // namespace

bool SceneryManager::load(const std::string& xplane_root) {
  objects_.clear();
  last_error_.clear();
  const fs::path custom = fs::path(xplane_root) / "Custom Scenery";
  if (!fs::exists(custom)) {
    last_error_ = "Custom Scenery folder not found";
    return false;
  }
  try {
    for (const auto& entry : fs::directory_iterator(custom)) {
      if (!entry.is_directory()) continue;
      const fs::path config = entry.path() / "ssa.json";
      if (fs::exists(config)) load_file(config.string());
    }
  } catch (const std::exception& e) {
    last_error_ = e.what();
  }
  return !objects_.empty();
}

bool SceneryManager::load_file(const std::string& path) {
  try {
    std::ifstream input(path);
    const json root = json::parse(input);
    const std::string airport = root.value("airport", "----");
    for (const auto& item : root.at("objects")) {
      ServiceObject object;
      object.id = item.at("id").get<std::string>();
      object.label = item.value("label", object.id);
      object.airport = airport;
      object.type = parse_type(item.value("type", "hangar"));
      object.latitude = item.at("latitude").get<double>();
      object.longitude = item.at("longitude").get<double>();
      object.radius_m = item.value("radius_m", object.type == ServiceType::Hangar ? 2000.0f : 35.0f);
      object.speed = item.value("speed", 0.20f);
      object.dataref = item.value("dataref", "boldstudio31/ssa/" + airport + "/" + object.id);
      refs_.create(object.dataref, 0.0f, false);
      objects_.push_back(std::move(object));
    }
    return true;
  } catch (const std::exception& e) {
    last_error_ = path + ": " + e.what();
    return false;
  }
}

void SceneryManager::update(float elapsed_seconds) {
  for (auto& object : objects_) {
    const float step = std::max(0.0f, object.speed * elapsed_seconds);
    if (object.progress < object.target) object.progress = std::min(object.target, object.progress + step);
    else if (object.progress > object.target) object.progress = std::max(object.target, object.progress - step);
    if (auto* ref = refs_.find(object.dataref)) ref->set(object.progress);
  }
}

std::vector<ServiceObject*> SceneryManager::nearby(ServiceType type, double lat, double lon, double radius_m) {
  std::vector<ServiceObject*> result;
  for (auto& object : objects_) {
    if (object.type == type && distance_m(lat, lon, object.latitude, object.longitude) <= radius_m)
      result.push_back(&object);
  }
  std::sort(result.begin(), result.end(), [=](const auto* a, const auto* b) {
    return distance_m(lat, lon, a->latitude, a->longitude) < distance_m(lat, lon, b->latitude, b->longitude);
  });
  return result;
}

} // namespace ssa

