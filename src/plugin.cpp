#include "ssa/datarefs.hpp"
#include "ssa/scenery.hpp"
#include "ssa/tablet.hpp"
#include <XPLMMenus.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

namespace {
ssa::DataRefRegistry refs;
std::unique_ptr<ssa::SceneryManager> scenery;
std::unique_ptr<ssa::Tablet> tablet;
ssa::FloatDataRef* automatic_ref{};
XPLMDataRef lat_ref{}, lon_ref{}, icao_ref{}, prop_ref{};
XPLMCommandRef tablet_command{};
XPLMCommandRef reload_command{};
XPLMMenuID menu{};
bool realops_detected{};
float compatibility_timer{};

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

void menu_handler(void*, void* item_ref) {
  if (item_ref == reinterpret_cast<void*>(1)) {
    if (tablet) tablet->toggle();
  } else if (item_ref == reinterpret_cast<void*>(2)) {
    reload_scenery();
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
    char icao[16]{};
    if (icao_ref) XPLMGetDatab(icao_ref, icao, 0, sizeof(icao) - 1);
    int prop_type = 0;
    if (prop_ref) XPLMGetDatavi(prop_ref, &prop_type, 0, 1);
    const bool turboprop = prop_type == 1 || prop_type == 2;
    auto nearby = scenery->nearby(ssa::ServiceType::Jetway, lat, lon, 35.0);
    for (auto* jetway : nearby) jetway->target = 0.0f;
    if (!turboprop && !nearby.empty()) nearby.front()->target = 1.0f;
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

    tablet_command = XPLMCreateCommand("boldstudio31/ssa/tablet/toggle", "Toggle SSA tablet");
    XPLMRegisterCommandHandler(tablet_command, command_handler, 1, nullptr);
    reload_command = XPLMCreateCommand("boldstudio31/ssa/config/reload", "Reload SSA scenery configuration");
    XPLMRegisterCommandHandler(reload_command, reload_handler, 1, nullptr);
    const int parent = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "SSA", nullptr, 0);
    menu = XPLMCreateMenu("SSA", XPLMFindPluginsMenu(), parent, menu_handler, nullptr);
    XPLMAppendMenuItem(menu, "Open tablet", reinterpret_cast<void*>(1), 0);
    XPLMAppendMenuItem(menu, "Reload scenery configuration", reinterpret_cast<void*>(2), 0);
    XPLMRegisterFlightLoopCallback(flight_loop, 0.05f, nullptr);
    log("SSA 0.2.0 started: " + std::to_string(scenery->objects().size()) + " object(s)");
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
