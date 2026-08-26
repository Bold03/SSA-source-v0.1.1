#pragma once

#include "ssa/datarefs.hpp"
#include <array>
#include <string>
#include <vector>

namespace ssa {

enum class ServiceType { Hangar, Jetway, Vehicle, GroundStaff, ParkingDisplay };
enum class JetwayState {
  Parked, WheelAligning, HeadPreAligning, Aligning, Approaching, Sealing,
  Connected, OutOfRange, Parking
};

struct AnimationChannel {
  std::string name;
  std::string dataref;
  float speed{0.20f};
  float progress{};
  float target{};
};

struct JetwayKinematics {
  bool enabled{};
  bool door_override{};
  float door_forward_m{};
  float door_right_m{};
  float door_sill_height_m{2.8f};
  float object_heading_deg{};
  float root_height_m{6.2116299f};
  float height_pivot_x_m{-1.5035599f};
  float height_pivot_y_m{-1.41546f};
  float height_pivot_z_m{0.05581f};
  float tunnel_parked_x_m{-14.0849003f};
  float extension_x_m{-10.7840999f};
  float head_x_m{-4.505846f};
  float head_y_m{-1.5702132f};
  float head_z_m{};
  float rotunda_degrees{90.000207f};
  float height_degrees{4.0599788f};
  float cabin_degrees{45.000104f};
  float cabin_pre_align_ratio{1.0f};
  float pre_dock_clearance_m{1.0f};
  float connect_tolerance_m{0.05f};
  float max_solution_error_m{0.25f};
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
  JetwayState jetway_state{JetwayState::Parked};
  float head_error_m{-1.0f};
  float solution_error_m{-1.0f};
  std::array<float, 4> solution_targets{};
  bool solution_ready{};
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
