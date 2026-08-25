#include "ssa/datarefs.hpp"
#include "ssa/scenery.hpp"
#include "ssa/tablet.hpp"
#include <XPLMMenus.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
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
XPLMDataRef lat_ref{}, lon_ref{}, icao_ref{}, prop_ref{}, onground_ref{}, groundspeed_ref{};
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

void log(const std::string& message) { XPLMDebugString(("[SSA] " + message + "\n").c_str()); }

void toggle_auto() {
  automatic_ref->set(automatic_ref->value() > 0.5f ? 0.0f : 1.0f);
  tablet->set_auto(automatic_ref->value() > 0.5f);
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
  if (action == action_open) jetway->target = 1.0f;
  else if (action == action_close) jetway->target = 0.0f;
  else jetway->target = jetway->target > 0.5f ? 0.0f : 1.0f;
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
  scenery->update(elapsed, realops_detected);

  // MVP automatic rule. Aircraft profiles/door geometry will refine wide-body selection.
  if (automatic_ref->value() > 0.5f) {
    char icao_buffer[16]{};
    if (icao_ref) XPLMGetDatab(icao_ref, icao_buffer, 0, sizeof(icao_buffer) - 1);
    const std::string icao(icao_buffer);
    int prop_type = 0;
    if (prop_ref) XPLMGetDatavi(prop_ref, &prop_type, 0, 1);
    const bool turboprop = is_turboprop(icao, prop_type);
    const bool parked = (!onground_ref || XPLMGetDatai(onground_ref) != 0) &&
                        (!groundspeed_ref || std::abs(XPLMGetDataf(groundspeed_ref)) < 0.5f);
    auto nearby = scenery->nearby(ssa::ServiceType::Jetway, lat, lon, 35.0);
    for (auto& object : scenery->objects())
      if (object.type == ssa::ServiceType::Jetway) object.target = 0.0f;
    if (parked && !turboprop) {
      const size_t connect_count = is_widebody(icao) ? std::min<size_t>(2, nearby.size())
                                                     : std::min<size_t>(1, nearby.size());
      for (size_t i = 0; i < connect_count; ++i) nearby[i]->target = 1.0f;
    }
  }
  return 0.05f;
}
} // namespace

PLUGIN_API int XPluginStart(char* name, char* signature, char* description) {
  std::snprintf(name, 256, "%s", "SSA");
  std::snprintf(signature, 256, "%s", "boldstudio31.scenery-service-animation");
  std::snprintf(description, 256, "%s", "Scenery Service Animation by BoldStudio31");
  try {
    automatic_ref = &refs.create("boldstudio31/ssa/jetway/automatic", 1.0f, true);
    scenery = std::make_unique<ssa::SceneryManager>(refs);
    char root[2048]{};
    XPLMGetSystemPath(root);
    scenery->load(root);
    tablet = std::make_unique<ssa::Tablet>(*scenery, toggle_auto, reload_scenery);

    lat_ref = XPLMFindDataRef("sim/flightmodel/position/latitude");
    lon_ref = XPLMFindDataRef("sim/flightmodel/position/longitude");
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
    log("SSA 0.4.0 started: " + std::to_string(scenery->objects().size()) + " object(s)");
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
