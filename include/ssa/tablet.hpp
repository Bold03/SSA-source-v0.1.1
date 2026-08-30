#pragma once

#include "ssa/scenery.hpp"
#include "ssa/route_editor.hpp"
#include "ssa/vdgs_editor.hpp"
#include "ssa/announcement_editor.hpp"
#include <XPLMDisplay.h>
#include <functional>

namespace ssa {

class Tablet {
public:
  Tablet(SceneryManager& scenery, RouteEditor& route_editor, VdgsEditor& vdgs_editor,
         AnnouncementEditor& announcement_editor,
         std::function<void()> toggle_auto,
         std::function<void()> reload_config,
         std::function<void(ServiceObject&)> toggle_object,
         std::function<void(ServiceObject*)> select_vdgs,
         std::function<void()> toggle_vehicle_spin,
         std::function<void(float)> set_vehicle_steering);
  ~Tablet();
  void toggle();
  bool visible() const;
  void toggle_developer_mode();
  bool developer_mode() const { return developer_mode_; }
  void set_position(double latitude, double longitude, float heading);
  void set_auto(bool enabled) { automatic_ = enabled; }
  void set_realops(bool detected) { realops_detected_ = detected; }
  void set_vehicle_test(bool spinning, float steering) {
    vehicle_spinning_ = spinning;
    vehicle_steering_ = steering;
  }

private:
  static void draw(XPLMWindowID id, void* refcon);
  static int mouse(XPLMWindowID id, int x, int y, XPLMMouseStatus status, void* refcon);
  void draw_impl();
  int mouse_impl(int x, int y, XPLMMouseStatus status);
  XPLMWindowID window_{};
  SceneryManager& scenery_;
  RouteEditor& route_editor_;
  VdgsEditor& vdgs_editor_;
  AnnouncementEditor& announcement_editor_;
  std::function<void()> toggle_auto_;
  std::function<void()> reload_config_;
  std::function<void(ServiceObject&)> toggle_object_;
  std::function<void(ServiceObject*)> select_vdgs_;
  std::function<void()> toggle_vehicle_spin_;
  std::function<void(float)> set_vehicle_steering_;
  double latitude_{};
  double longitude_{};
  float heading_{};
  bool automatic_{};
  bool realops_detected_{};
  bool developer_mode_{};
  bool vehicle_spinning_{};
  float vehicle_steering_{};
  int tab_{};
};

} // namespace ssa
