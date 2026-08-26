#pragma once

#include "ssa/scenery.hpp"
#include <XPLMDisplay.h>
#include <functional>

namespace ssa {

class Tablet {
public:
  Tablet(SceneryManager& scenery, std::function<void()> toggle_auto,
         std::function<void()> reload_config);
  ~Tablet();
  void toggle();
  bool visible() const;
  void set_position(double latitude, double longitude);
  void set_auto(bool enabled) { automatic_ = enabled; }
  void set_realops(bool detected) { realops_detected_ = detected; }

private:
  static void draw(XPLMWindowID id, void* refcon);
  static int mouse(XPLMWindowID id, int x, int y, XPLMMouseStatus status, void* refcon);
  void draw_impl();
  int mouse_impl(int x, int y, XPLMMouseStatus status);
  XPLMWindowID window_{};
  SceneryManager& scenery_;
  std::function<void()> toggle_auto_;
  std::function<void()> reload_config_;
  double latitude_{};
  double longitude_{};
  bool automatic_{};
  bool realops_detected_{};
  int tab_{};
};

} // namespace ssa
