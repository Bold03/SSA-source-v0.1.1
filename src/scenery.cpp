#include "ssa/scenery.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <unordered_map>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace ssa {
namespace {
bool parse_type(const std::string& value, ServiceType& result) {
  if (value == "hangar") result = ServiceType::Hangar;
  else if (value == "jetway") result = ServiceType::Jetway;
  else if (value == "vehicle") result = ServiceType::Vehicle;
  else if (value == "ground_staff") result = ServiceType::GroundStaff;
  else if (value == "parking_display") result = ServiceType::ParkingDisplay;
  else return false;
  return true;
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
  std::unordered_map<std::string, std::pair<float, float>> previous_state;
  for (const auto& object : objects_) {
    previous_state.emplace(object.dataref, std::make_pair(object.progress, object.target));
    for (const auto& channel : object.channels)
      previous_state.emplace(channel.dataref, std::make_pair(channel.progress, channel.target));
  }
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
  for (auto& object : objects_) {
    const auto previous = previous_state.find(object.dataref);
    if (previous == previous_state.end()) continue;
    object.progress = previous->second.first;
    object.target = previous->second.second;
    if (auto* ref = refs_.find(object.dataref)) ref->set(object.progress);
    for (auto& channel : object.channels) {
      const auto old_channel = previous_state.find(channel.dataref);
      if (old_channel == previous_state.end()) continue;
      channel.progress = old_channel->second.first;
      channel.target = old_channel->second.second;
      if (auto* ref = refs_.find(channel.dataref)) ref->set(channel.progress);
    }
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
      const std::string type_name = item.value("type", "hangar");
      if (!parse_type(type_name, object.type)) {
        last_error_ = path + ": unsupported object type '" + type_name + "'";
        continue;
      }
      object.latitude = item.at("latitude").get<double>();
      object.longitude = item.at("longitude").get<double>();
      object.radius_m = item.value("radius_m", object.type == ServiceType::Hangar ? 2000.0f : 35.0f);
      object.speed = std::clamp(item.value("speed", 0.20f), 0.01f, 2.0f);
      object.dataref = item.value("dataref", "boldstudio31/ssa/" + airport + "/" + object.id);
      const bool duplicate = std::any_of(objects_.begin(), objects_.end(), [&](const auto& existing) {
        return existing.dataref == object.dataref;
      });
      if (duplicate) {
        last_error_ = path + ": duplicate dataref '" + object.dataref + "'";
        continue;
      }
      refs_.create(object.dataref, 0.0f, false);
      if (item.contains("channels")) {
        for (const auto& [channel_name, channel_value] : item.at("channels").items()) {
          AnimationChannel channel;
          channel.name = channel_name;
          if (channel_value.is_string()) {
            channel.dataref = channel_value.get<std::string>();
            channel.speed = object.speed;
          } else {
            channel.dataref = channel_value.at("dataref").get<std::string>();
            channel.speed = std::clamp(channel_value.value("speed", object.speed), 0.01f, 2.0f);
          }
          const bool channel_duplicate = std::any_of(objects_.begin(), objects_.end(), [&](const auto& existing) {
            if (existing.dataref == channel.dataref) return true;
            return std::any_of(existing.channels.begin(), existing.channels.end(), [&](const auto& existing_channel) {
              return existing_channel.dataref == channel.dataref;
            });
          }) || channel.dataref == object.dataref ||
          std::any_of(object.channels.begin(), object.channels.end(), [&](const auto& existing_channel) {
            return existing_channel.dataref == channel.dataref;
          });
          if (channel_duplicate) {
            last_error_ = path + ": duplicate channel dataref '" + channel.dataref + "'";
            continue;
          }
          refs_.create(channel.dataref, 0.0f, false);
          object.channels.push_back(std::move(channel));
        }
      }
      objects_.push_back(std::move(object));
    }
    return true;
  } catch (const std::exception& e) {
    last_error_ = path + ": " + e.what();
    return false;
  }
}

void SceneryManager::update(float elapsed_seconds, bool suppress_ground_services) {
  for (auto& object : objects_) {
    if (suppress_ground_services &&
        (object.type == ServiceType::Vehicle || object.type == ServiceType::GroundStaff)) {
      object.target = 0.0f;
    }
    if (!object.channels.empty()) {
      float total = 0.0f;
      for (auto& channel : object.channels) {
        channel.target = object.target;
        const float channel_step = std::max(0.0f, channel.speed * elapsed_seconds);
        if (channel.progress < channel.target)
          channel.progress = std::min(channel.target, channel.progress + channel_step);
        else if (channel.progress > channel.target)
          channel.progress = std::max(channel.target, channel.progress - channel_step);
        if (auto* ref = refs_.find(channel.dataref)) ref->set(channel.progress);
        total += channel.progress;
      }
      object.progress = total / static_cast<float>(object.channels.size());
      if (auto* ref = refs_.find(object.dataref)) ref->set(object.progress);
      continue;
    }
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
