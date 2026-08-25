#pragma once

#include "ssa/datarefs.hpp"
#include <string>
#include <vector>

namespace ssa {

enum class ServiceType { Hangar, Jetway, Vehicle, GroundStaff, ParkingDisplay, Billboard };

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
};

class SceneryManager {
public:
  explicit SceneryManager(DataRefRegistry& refs) : refs_(refs) {}
  bool load(const std::string& xplane_root);
  void update(float elapsed_seconds);
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

