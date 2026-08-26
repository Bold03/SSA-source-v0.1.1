#include "ssa/datarefs.hpp"
#include "ssa/scenery.hpp"
#include "ssa/tablet.hpp"
#include <XPLMMenus.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMGraphics.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <memory>
#include <string>

namespace {
ssa::DataRefRegistry refs;
std::unique_ptr<ssa::SceneryManager> scenery;
std::unique_ptr<ssa::Tablet> tablet;
ssa::FloatDataRef* automatic_ref{};
XPLMDataRef lat_ref{}, lon_ref{}, heading_ref{}, icao_ref{}, door_open_ref{}, prop_ref{}, onground_ref{}, groundspeed_ref{};
XPLMCommandRef tablet_command{};
XPLMCommandRef reload_command{};
XPLMCommandRef hangar_toggle_command{};
XPLMCommandRef hangar_open_command{};
XPLMCommandRef hangar_close_command{};
XPLMCommandRef jetway_toggle_command{};
XPLMCommandRef jetway_connect_command{};
XPLMCommandRef jetway_disconnect_command{};
XPLMMenuID menu{};
bool realops_detected{};
float compatibility_timer{};
int action_toggle{}, action_open{1}, action_close{2};

struct DoorProfile {
  float forward_m;
  float right_m;
  float sill_height_m;
};

struct Vec3 {
  float x{};
  float y{};
  float z{};
};

struct JetwaySolution {
  std::array<float, 4> ratios{}; // rotunda, extension, height, cabin yaw
  float position_error_m{1000000.0f};
};

void log(const std::string& message) { XPLMDebugString(("[SSA] " + message + "\n").c_str()); }

void toggle_auto() {
  automatic_ref->set(automatic_ref->value() > 0.5f ? 0.0f : 1.0f);
  tablet->set_auto(automatic_ref->value() > 0.5f);
}

float wrap_degrees(float value) {
  while (value > 180.0f) value -= 360.0f;
  while (value < -180.0f) value += 360.0f;
  return value;
}

std::string aircraft_icao() {
  char buffer[16]{};
  if (icao_ref) XPLMGetDatab(icao_ref, buffer, 0, sizeof(buffer) - 1);
  return buffer;
}

DoorProfile door_profile(const std::string& icao) {
  // Offsets are measured from the aircraft reference point: forward, right,
  // and passenger-door sill height above the ground. B738 is calibrated first.
  // The default B738 pilot eye is 8.6 m forward of the aircraft reference;
  // L1 is approximately 1.5 m aft of that point.
  if (icao == "B738") return {7.1f, -1.85f, 2.75f};
  if (icao == "B737") return {11.5f, -1.85f, 2.70f};
  if (icao == "B739") return {13.4f, -1.85f, 2.75f};
  if (icao == "A319") return {11.2f, -1.95f, 2.85f};
  if (icao == "A320") return {12.0f, -1.95f, 2.85f};
  if (icao == "A321") return {14.5f, -1.95f, 2.90f};
  return {11.5f, -1.8f, 2.8f};
}

Vec3 rotate_y(Vec3 value, float degrees) {
  constexpr float radians = 3.14159265358979323846f / 180.0f;
  const float angle = degrees * radians;
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {cosine * value.x + sine * value.z, value.y,
          -sine * value.x + cosine * value.z};
}

Vec3 rotate_z(Vec3 value, float degrees) {
  constexpr float radians = 3.14159265358979323846f / 180.0f;
  const float angle = degrees * radians;
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {cosine * value.x - sine * value.y,
          sine * value.x + cosine * value.y, value.z};
}

Vec3 head_local(const ssa::JetwayKinematics& k, const std::array<float, 4>& ratios) {
  Vec3 point{k.head_x_m, k.head_y_m, k.head_z_m};
  point = rotate_y(point, k.cabin_degrees * ratios[3]);
  point.x += k.tunnel_parked_x_m + k.extension_x_m * ratios[1];
  point = rotate_z(point, k.height_degrees * ratios[2]);
  point.x += k.height_pivot_x_m;
  point.y += k.height_pivot_y_m;
  point.z += k.height_pivot_z_m;
  point = rotate_y(point, k.rotunda_degrees * ratios[0]);
  point.y += k.root_height_m;
  return point;
}

float cabin_heading_world(const ssa::JetwayKinematics& k,
                          const std::array<float, 4>& ratios) {
  Vec3 direction{-1.0f, 0.0f, 0.0f};
  direction = rotate_y(direction, k.cabin_degrees * ratios[3]);
  direction = rotate_z(direction, k.height_degrees * ratios[2]);
  direction = rotate_y(direction, k.rotunda_degrees * ratios[0]);
  constexpr float radians = 3.14159265358979323846f / 180.0f;
  const float heading = k.object_heading_deg * radians;
  const float world_x = std::cos(heading) * direction.x - std::sin(heading) * direction.z;
  const float world_z = std::sin(heading) * direction.x + std::cos(heading) * direction.z;
  return std::atan2(world_x, -world_z) / radians;
}

Vec3 door_local(const ssa::ServiceObject& jetway, float& aircraft_heading) {
  const double aircraft_lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
  const double aircraft_lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
  aircraft_heading = heading_ref ? XPLMGetDataf(heading_ref) : 0.0f;
  double aircraft_x{}, aircraft_y{}, aircraft_z{};
  double base_x{}, base_y{}, base_z{};
  XPLMWorldToLocal(aircraft_lat, aircraft_lon, 0.0, &aircraft_x, &aircraft_y, &aircraft_z);
  XPLMWorldToLocal(jetway.latitude, jetway.longitude, 0.0, &base_x, &base_y, &base_z);

  constexpr double radians = 3.14159265358979323846 / 180.0;
  const double aircraft_angle = static_cast<double>(aircraft_heading) * radians;
  DoorProfile door = door_profile(aircraft_icao());
  if (jetway.kinematics.door_override) {
    door.forward_m = jetway.kinematics.door_forward_m;
    door.right_m = jetway.kinematics.door_right_m;
    door.sill_height_m = jetway.kinematics.door_sill_height_m;
  }
  const double door_x = aircraft_x + std::sin(aircraft_angle) * door.forward_m +
                        std::cos(aircraft_angle) * door.right_m;
  const double door_z = aircraft_z - std::cos(aircraft_angle) * door.forward_m +
                        std::sin(aircraft_angle) * door.right_m;
  const double dx = door_x - base_x;
  const double dz = door_z - base_z;

  const double object_angle = static_cast<double>(jetway.kinematics.object_heading_deg) * radians;
  return {static_cast<float>(std::cos(object_angle) * dx + std::sin(object_angle) * dz),
          door.sill_height_m,
          static_cast<float>(-std::sin(object_angle) * dx + std::cos(object_angle) * dz)};
}

float position_error(const ssa::JetwayKinematics& k, const Vec3& door,
                     const std::array<float, 4>& ratios) {
  const Vec3 head = head_local(k, ratios);
  return std::sqrt((head.x - door.x) * (head.x - door.x) +
                   (head.y - door.y) * (head.y - door.y) +
                   (head.z - door.z) * (head.z - door.z));
}

float objective(const ssa::JetwayKinematics& k, const Vec3& door,
                float aircraft_heading, const std::array<float, 4>& ratios) {
  const float error = position_error(k, door, ratios);
  const float desired_heading = aircraft_heading + 90.0f;
  const float heading_error = wrap_degrees(cabin_heading_world(k, ratios) - desired_heading);
  return error * error + heading_error * heading_error * 0.00001f;
}

JetwaySolution solve_jetway(const ssa::ServiceObject& jetway, const Vec3& door,
                            float aircraft_heading) {
  JetwaySolution best;
  float best_score = 1.0e30f;
  for (int r = 0; r <= 5; ++r) {
    for (int e = 0; e <= 5; ++e) {
      for (int h = 0; h <= 5; ++h) {
        for (int c = 0; c <= 5; ++c) {
          std::array<float, 4> candidate{r / 5.0f, e / 5.0f, h / 5.0f, c / 5.0f};
          const float score = objective(jetway.kinematics, door, aircraft_heading, candidate);
          if (score < best_score) {
            best_score = score;
            best.ratios = candidate;
          }
        }
      }
    }
  }
  float step = 0.1f;
  for (int iteration = 0; iteration < 9; ++iteration, step *= 0.5f) {
    for (size_t axis = 0; axis < best.ratios.size(); ++axis) {
      const auto center = best.ratios;
      for (int direction : {-1, 1}) {
        auto candidate = center;
        candidate[axis] = std::clamp(candidate[axis] + direction * step, 0.0f, 1.0f);
        const float score = objective(jetway.kinematics, door, aircraft_heading, candidate);
        if (score < best_score) {
          best_score = score;
          best.ratios = candidate;
        }
      }
    }
  }
  best.position_error_m = position_error(jetway.kinematics, door, best.ratios);
  return best;
}

float channel_progress(const ssa::ServiceObject& object, const char* name) {
  const auto found = std::find_if(object.channels.begin(), object.channels.end(),
                                  [&](const auto& channel) { return channel.name == name; });
  return found == object.channels.end() ? 0.0f : found->progress;
}

std::array<float, 4> current_ratios(const ssa::ServiceObject& object) {
  return {channel_progress(object, "rotunda"), channel_progress(object, "extension"),
          channel_progress(object, "height"), channel_progress(object, "cabin_yaw")};
}

bool docking_channels_at_target(const ssa::ServiceObject& object) {
  for (const char* name : {"rotunda", "extension", "height", "cabin_yaw"}) {
    const auto found = std::find_if(object.channels.begin(), object.channels.end(),
                                    [&](const auto& channel) { return channel.name == name; });
    if (found == object.channels.end() || std::abs(found->progress - found->target) > 0.001f)
      return false;
  }
  return true;
}

bool apply_door_target(ssa::ServiceObject& jetway) {
  if (!scenery) return false;
  if (!jetway.kinematics.enabled) {
    scenery->set_uniform_target(jetway, 1.0f);
    return true;
  }

  float aircraft_heading{};
  const Vec3 door = door_local(jetway, aircraft_heading);
  const JetwaySolution solution = solve_jetway(jetway, door, aircraft_heading);
  jetway.solution_error_m = solution.position_error_m;
  if (solution.position_error_m > jetway.kinematics.max_solution_error_m) {
    scenery->set_uniform_target(jetway, 0.0f);
    jetway.jetway_state = ssa::JetwayState::OutOfRange;
    jetway.head_error_m = solution.position_error_m;
    return false;
  }
  jetway.target = 1.0f;
  jetway.jetway_state = ssa::JetwayState::Docking;
  scenery->set_channel_target(jetway, "rotunda", solution.ratios[0]);
  scenery->set_channel_target(jetway, "extension", solution.ratios[1]);
  scenery->set_channel_target(jetway, "height", solution.ratios[2]);
  scenery->set_channel_target(jetway, "cabin_yaw", solution.ratios[3]);
  scenery->set_channel_target(jetway, "wheel_steer", solution.ratios[0]);
  scenery->set_channel_target(jetway, "wheel_rotation", solution.ratios[1]);
  return true;
}

void evaluate_jetway_head(ssa::ServiceObject& jetway) {
  if (!jetway.kinematics.enabled || jetway.target < 0.5f ||
      jetway.jetway_state == ssa::JetwayState::OutOfRange) return;
  float aircraft_heading{};
  const Vec3 door = door_local(jetway, aircraft_heading);
  const auto ratios = current_ratios(jetway);
  jetway.head_error_m = position_error(jetway.kinematics, door, ratios);
  if (jetway.head_error_m <= jetway.kinematics.connect_tolerance_m) {
    for (auto& channel : jetway.channels) channel.target = channel.progress;
    jetway.jetway_state = ssa::JetwayState::Connected;
    jetway.progress = 1.0f;
  } else if (docking_channels_at_target(jetway)) {
    // The requested pose has been reached, but the cabin head is still too far
    // from the door. Never report a false CONNECTED state.
    jetway.jetway_state = ssa::JetwayState::OutOfRange;
  }
}

int command_handler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
  if (phase == xplm_CommandBegin && tablet) tablet->toggle();
  return 1;
}

void reload_scenery() {
  if (!scenery) return;
  char root[2048]{};
  XPLMGetSystemPath(root);
  const bool loaded = scenery->load(root);
  log("Configuration reloaded: " + std::to_string(scenery->objects().size()) +
      " object(s)" + (loaded ? "" : " (none found)"));
}

int reload_handler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
  if (phase == xplm_CommandBegin) reload_scenery();
  return 1;
}

bool control_nearest_hangar(int action) {
  if (!scenery) return false;
  const double lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
  const double lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
  auto nearby = scenery->nearby(ssa::ServiceType::Hangar, lat, lon, 2000.0);
  if (nearby.empty()) {
    log("No hangar found within 2 km");
    return false;
  }
  auto* hangar = nearby.front();
  if (action == action_open) hangar->target = 1.0f;
  else if (action == action_close) hangar->target = 0.0f;
  else hangar->target = hangar->target > 0.5f ? 0.0f : 1.0f;
  log("Nearest hangar '" + hangar->label + "' target: " +
      (hangar->target > 0.5f ? "OPEN" : "CLOSED"));
  return true;
}

bool control_nearest_jetway(int action) {
  if (!scenery) return false;
  const double lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
  const double lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
  auto nearby = scenery->nearby(ssa::ServiceType::Jetway, lat, lon, 35.0);
  if (nearby.empty()) {
    log("No jetway found within 35 m");
    return false;
  }
  auto* jetway = nearby.front();
  if (action == action_close || (action == action_toggle && jetway->target > 0.5f)) {
    scenery->set_uniform_target(*jetway, 0.0f);
  } else {
    apply_door_target(*jetway);
  }
  log("Nearest jetway '" + jetway->label + "' target: " +
      (jetway->jetway_state == ssa::JetwayState::OutOfRange ? "OUT OF RANGE" :
       jetway->target > 0.5f ? "DOCKING" : "PARKED"));
  return true;
}

int hangar_handler(XPLMCommandRef, XPLMCommandPhase phase, void* refcon) {
  if (phase == xplm_CommandBegin) control_nearest_hangar(*static_cast<int*>(refcon));
  return 1;
}

int jetway_handler(XPLMCommandRef, XPLMCommandPhase phase, void* refcon) {
  if (phase == xplm_CommandBegin) control_nearest_jetway(*static_cast<int*>(refcon));
  return 1;
}

bool matches_icao(const std::string& icao, const std::initializer_list<const char*>& values) {
  return std::any_of(values.begin(), values.end(), [&](const char* value) { return icao == value; });
}

bool is_turboprop(const std::string& icao, int engine_type) {
  return matches_icao(icao, {"AT43", "AT45", "AT72", "AT73", "AT75", "AT76",
                             "DH8A", "DH8B", "DH8C", "DH8D", "SF34", "F50",
                             "C208", "BE20", "BE30", "JS41"}) || engine_type == 1;
}

bool is_widebody(const std::string& icao) {
  return matches_icao(icao, {"A306", "A30B", "A310", "A332", "A333", "A338",
                             "A339", "A342", "A343", "A345", "A346", "A359",
                             "A35K", "B741", "B742", "B743", "B744", "B748",
                             "B762", "B763", "B764", "B772", "B773", "B77L",
                             "B77W", "B788", "B789", "B78X", "DC10", "L101", "MD11"});
}

void menu_handler(void*, void* item_ref) {
  if (item_ref == reinterpret_cast<void*>(1)) {
    if (tablet) tablet->toggle();
  } else if (item_ref == reinterpret_cast<void*>(2)) {
    reload_scenery();
  } else if (item_ref == reinterpret_cast<void*>(3)) {
    control_nearest_hangar(action_toggle);
  } else if (item_ref == reinterpret_cast<void*>(4)) {
    control_nearest_jetway(action_toggle);
  }
}

float flight_loop(float elapsed, float, int, void*) {
  if (!scenery || !tablet) return 1.0f;
  const double lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
  const double lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
  tablet->set_position(lat, lon);
  compatibility_timer += elapsed;
  if (compatibility_timer >= 1.0f) {
    compatibility_timer = 0.0f;
    const bool detected = XPLMFindPluginBySignature("realops") != XPLM_NO_PLUGIN_ID;
    if (detected != realops_detected) {
      realops_detected = detected;
      log(std::string("RealOps compatibility ") + (detected ? "active" : "inactive"));
    }
    tablet->set_realops(realops_detected);
  }
  // Automatic docking calculates an independent target for every animation
  // channel instead of sending every part to its Blender value of 1.0.
  if (automatic_ref->value() > 0.5f) {
    const std::string icao = aircraft_icao();
    int prop_type = 0;
    if (prop_ref) XPLMGetDatavi(prop_ref, &prop_type, 0, 1);
    const bool turboprop = is_turboprop(icao, prop_type);
    const bool parked = (!onground_ref || XPLMGetDatai(onground_ref) != 0) &&
                        (!groundspeed_ref || std::abs(XPLMGetDataf(groundspeed_ref)) < 0.5f);
    auto nearby = scenery->nearby(ssa::ServiceType::Jetway, lat, lon, 35.0);
    for (auto& object : scenery->objects())
      if (object.type == ssa::ServiceType::Jetway) scenery->set_uniform_target(object, 0.0f);
    if (parked && !turboprop) {
      const size_t connect_count = is_widebody(icao) ? std::min<size_t>(2, nearby.size())
                                                     : std::min<size_t>(1, nearby.size());
      for (size_t i = 0; i < connect_count; ++i) apply_door_target(*nearby[i]);
    }
  }
  scenery->update(elapsed, realops_detected);
  for (auto& object : scenery->objects())
    if (object.type == ssa::ServiceType::Jetway) evaluate_jetway_head(object);
  return 0.05f;
}
} // namespace

PLUGIN_API int XPluginStart(char* name, char* signature, char* description) {
  std::snprintf(name, 256, "%s", "SSA");
  std::snprintf(signature, 256, "%s", "boldstudio31.scenery-service-animation");
  std::snprintf(description, 256, "%s", "Scenery Service Animation by BoldStudio31");
  try {
    // Start safely in manual mode so a newly-authored jetway cannot deploy
    // before its kinematic limits have been tested at the airport.
    automatic_ref = &refs.create("boldstudio31/ssa/jetway/automatic", 0.0f, true);
    scenery = std::make_unique<ssa::SceneryManager>(refs);
    char root[2048]{};
    XPLMGetSystemPath(root);
    scenery->load(root);
    tablet = std::make_unique<ssa::Tablet>(*scenery, toggle_auto, reload_scenery);

    lat_ref = XPLMFindDataRef("sim/flightmodel/position/latitude");
    lon_ref = XPLMFindDataRef("sim/flightmodel/position/longitude");
    heading_ref = XPLMFindDataRef("sim/flightmodel/position/psi");
    icao_ref = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    door_open_ref = XPLMFindDataRef("sim/cockpit2/switches/door_open");
    prop_ref = XPLMFindDataRef("sim/aircraft/prop/acf_en_type");
    onground_ref = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    groundspeed_ref = XPLMFindDataRef("sim/flightmodel/position/groundspeed");

    tablet_command = XPLMCreateCommand("boldstudio31/ssa/tablet/toggle", "Toggle SSA tablet");
    XPLMRegisterCommandHandler(tablet_command, command_handler, 1, nullptr);
    reload_command = XPLMCreateCommand("boldstudio31/ssa/config/reload", "Reload SSA scenery configuration");
    XPLMRegisterCommandHandler(reload_command, reload_handler, 1, nullptr);
    hangar_toggle_command = XPLMCreateCommand("boldstudio31/ssa/hangar/nearest_toggle", "Toggle nearest SSA hangar");
    hangar_open_command = XPLMCreateCommand("boldstudio31/ssa/hangar/nearest_open", "Open nearest SSA hangar");
    hangar_close_command = XPLMCreateCommand("boldstudio31/ssa/hangar/nearest_close", "Close nearest SSA hangar");
    XPLMRegisterCommandHandler(hangar_toggle_command, hangar_handler, 1, &action_toggle);
    XPLMRegisterCommandHandler(hangar_open_command, hangar_handler, 1, &action_open);
    XPLMRegisterCommandHandler(hangar_close_command, hangar_handler, 1, &action_close);
    jetway_toggle_command = XPLMCreateCommand("boldstudio31/ssa/jetway/nearest_toggle", "Toggle nearest SSA jetway");
    jetway_connect_command = XPLMCreateCommand("boldstudio31/ssa/jetway/nearest_connect", "Connect nearest SSA jetway");
    jetway_disconnect_command = XPLMCreateCommand("boldstudio31/ssa/jetway/nearest_disconnect", "Disconnect nearest SSA jetway");
    XPLMRegisterCommandHandler(jetway_toggle_command, jetway_handler, 1, &action_toggle);
    XPLMRegisterCommandHandler(jetway_connect_command, jetway_handler, 1, &action_open);
    XPLMRegisterCommandHandler(jetway_disconnect_command, jetway_handler, 1, &action_close);
    const int parent = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "SSA", nullptr, 0);
    menu = XPLMCreateMenu("SSA", XPLMFindPluginsMenu(), parent, menu_handler, nullptr);
    XPLMAppendMenuItem(menu, "Open tablet", reinterpret_cast<void*>(1), 0);
    XPLMAppendMenuItem(menu, "Reload scenery configuration", reinterpret_cast<void*>(2), 0);
    XPLMAppendMenuItem(menu, "Toggle nearest hangar", reinterpret_cast<void*>(3), 0);
    XPLMAppendMenuItem(menu, "Toggle nearest jetway", reinterpret_cast<void*>(4), 0);
    XPLMRegisterFlightLoopCallback(flight_loop, 0.05f, nullptr);
    log("SSA 0.6.0 started: " + std::to_string(scenery->objects().size()) +
        " object(s), L1 door dataref " + (door_open_ref ? "detected" : "not found"));
    return 1;
  } catch (const std::exception& e) {
    log(std::string("Start failed: ") + e.what());
    return 0;
  }
}

PLUGIN_API void XPluginStop() {
  XPLMUnregisterFlightLoopCallback(flight_loop, nullptr);
  if (tablet_command) XPLMUnregisterCommandHandler(tablet_command, command_handler, 1, nullptr);
  if (reload_command) XPLMUnregisterCommandHandler(reload_command, reload_handler, 1, nullptr);
  if (hangar_toggle_command) XPLMUnregisterCommandHandler(hangar_toggle_command, hangar_handler, 1, &action_toggle);
  if (hangar_open_command) XPLMUnregisterCommandHandler(hangar_open_command, hangar_handler, 1, &action_open);
  if (hangar_close_command) XPLMUnregisterCommandHandler(hangar_close_command, hangar_handler, 1, &action_close);
  if (jetway_toggle_command) XPLMUnregisterCommandHandler(jetway_toggle_command, jetway_handler, 1, &action_toggle);
  if (jetway_connect_command) XPLMUnregisterCommandHandler(jetway_connect_command, jetway_handler, 1, &action_open);
  if (jetway_disconnect_command) XPLMUnregisterCommandHandler(jetway_disconnect_command, jetway_handler, 1, &action_close);
  if (menu) XPLMDestroyMenu(menu);
  tablet.reset();
  scenery.reset();
  refs.clear();
}

PLUGIN_API int XPluginEnable() { return 1; }
PLUGIN_API void XPluginDisable() {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int message, void*) {
  if (message == XPLM_MSG_SCENERY_LOADED) reload_scenery();
}
