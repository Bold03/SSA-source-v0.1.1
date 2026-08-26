#include "ssa/datarefs.hpp"
#include "ssa/scenery.hpp"
#include "ssa/tablet.hpp"
#include <XPLMMenus.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMGraphics.h>
#include <algorithm>
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
XPLMDataRef lat_ref{}, lon_ref{}, heading_ref{}, icao_ref{}, prop_ref{}, onground_ref{}, groundspeed_ref{};
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

void log(const std::string& message) { XPLMDebugString(("[SSA] " + message + "\n").c_str()); }

void toggle_auto() {
  automatic_ref->set(automatic_ref->value() > 0.5f ? 0.0f : 1.0f);
  tablet->set_auto(automatic_ref->value() > 0.5f);
}

float clamp_ratio(float value, float minimum, float maximum) {
  if (std::abs(maximum - minimum) < 0.001f) return 0.0f;
  return std::clamp((value - minimum) / (maximum - minimum), 0.0f, 1.0f);
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
  if (icao == "B738") return {12.6f, -1.85f, 2.75f};
  if (icao == "B737") return {11.5f, -1.85f, 2.70f};
  if (icao == "B739") return {13.4f, -1.85f, 2.75f};
  if (icao == "A319") return {11.2f, -1.95f, 2.85f};
  if (icao == "A320") return {12.0f, -1.95f, 2.85f};
  if (icao == "A321") return {14.5f, -1.95f, 2.90f};
  return {11.5f, -1.8f, 2.8f};
}

bool apply_door_target(ssa::ServiceObject& jetway) {
  if (!scenery) return false;
  if (!jetway.kinematics.enabled) {
    scenery->set_uniform_target(jetway, 1.0f);
    return true;
  }

  const double aircraft_lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
  const double aircraft_lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
  const float aircraft_heading = heading_ref ? XPLMGetDataf(heading_ref) : 0.0f;
  double aircraft_x{}, aircraft_y{}, aircraft_z{};
  double base_x{}, base_y{}, base_z{};
  XPLMWorldToLocal(aircraft_lat, aircraft_lon, 0.0, &aircraft_x, &aircraft_y, &aircraft_z);
  XPLMWorldToLocal(jetway.latitude, jetway.longitude, 0.0, &base_x, &base_y, &base_z);

  constexpr double pi = 3.14159265358979323846;
  const double heading_rad = static_cast<double>(aircraft_heading) * pi / 180.0;
  const DoorProfile door = door_profile(aircraft_icao());
  const double forward_x = std::sin(heading_rad);
  const double forward_z = -std::cos(heading_rad);
  const double right_x = std::cos(heading_rad);
  const double right_z = std::sin(heading_rad);
  const double door_x = aircraft_x + forward_x * door.forward_m + right_x * door.right_m;
  const double door_z = aircraft_z + forward_z * door.forward_m + right_z * door.right_m;
  const double dx = door_x - base_x;
  const double dz = door_z - base_z;
  const float distance = static_cast<float>(std::hypot(dx, dz));
  const float target_heading = static_cast<float>(std::atan2(dx, -dz) * 180.0 / pi);

  const auto& k = jetway.kinematics;
  const float rotunda_angle = wrap_degrees(target_heading - k.parked_heading_deg);
  const float rotunda = clamp_ratio(rotunda_angle, k.rotunda_min_deg, k.rotunda_max_deg);
  const float extension = clamp_ratio(distance, k.parked_length_m,
                                      k.parked_length_m + k.extension_travel_m);
  const float height = clamp_ratio(door.sill_height_m, k.deck_min_m, k.deck_max_m);
  const float cabin_angle = wrap_degrees(aircraft_heading + 90.0f - target_heading);
  const float cabin = clamp_ratio(cabin_angle, k.cabin_yaw_min_deg, k.cabin_yaw_max_deg);

  jetway.target = 1.0f;
  scenery->set_channel_target(jetway, "rotunda", rotunda);
  scenery->set_channel_target(jetway, "extension", extension);
  scenery->set_channel_target(jetway, "height", height);
  scenery->set_channel_target(jetway, "cabin_yaw", cabin);
  scenery->set_channel_target(jetway, "wheel_steer", rotunda);
  scenery->set_channel_target(jetway, "wheel_rotation", extension);
  return true;
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
  if (action == action_close || (action == action_toggle && jetway->target > 0.5f))
    scenery->set_uniform_target(*jetway, 0.0f);
  else
    apply_door_target(*jetway);
  log("Nearest jetway '" + jetway->label + "' target: " +
      (jetway->target > 0.5f ? "CONNECTED" : "PARKED"));
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
    log("SSA 0.5.0 started: " + std::to_string(scenery->objects().size()) + " object(s)");
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
