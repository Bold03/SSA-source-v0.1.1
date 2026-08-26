#pragma once

#include "ssa/datarefs.hpp"
#include <string>
#include <vector>

namespace ssa {

enum class ServiceType { Hangar, Jetway, Vehicle, GroundStaff, ParkingDisplay };

struct AnimationChannel {
  std::string name;
  std::string dataref;
  float speed{0.20f};
  float progress{};
  float target{};
};

struct JetwayKinematics {
  bool enabled{};
  float parked_heading_deg{};
  float rotunda_min_deg{};
  float rotunda_max_deg{90.0f};
  float parked_length_m{18.0f};
  float extension_travel_m{12.0f};
  float deck_min_m{2.0f};
  float deck_max_m{5.5f};
  float cabin_yaw_min_deg{-45.0f};
  float cabin_yaw_max_deg{45.0f};
};

struct ServiceObject {
  std::string id;
  std::string label;
  std::string airport;
  std::string dataref;
  ServiceType type{ServiceType::Hangar};
  double latitude{};
  double longitude{};
  float radius_m{35.0f};
  float speed{0.20f};
  float progress{};
  float target{};
  JetwayKinematics kinematics;
  std::vector<AnimationChannel> channels;
};

class SceneryManager {
public:
  explicit SceneryManager(DataRefRegistry& refs) : refs_(refs) {}
  bool load(const std::string& xplane_root);
  void update(float elapsed_seconds, bool suppress_ground_services = false);
  void set_uniform_target(ServiceObject& object, float target);
  bool set_channel_target(ServiceObject& object, const std::string& channel_name, float target);
  std::vector<ServiceObject*> nearby(ServiceType type, double lat, double lon, double radius_m);
  std::vector<ServiceObject>& objects() { return objects_; }
  const std::string& last_error() const { return last_error_; }

private:
  bool load_file(const std::string& path);
  DataRefRegistry& refs_;
  std::vector<ServiceObject> objects_;
  std::string last_error_;
};

} // namespace ssa
