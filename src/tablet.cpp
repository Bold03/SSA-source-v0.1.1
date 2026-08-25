#include "ssa/tablet.hpp"
#include <XPLMGraphics.h>
#include <algorithm>
#include <cstdio>

namespace ssa {
namespace {
void label(int x, int y, const char* text, float r = 0.90f, float g = 0.95f, float b = 1.0f) {
  float color[] = {r, g, b};
  XPLMDrawString(color, x, y, const_cast<char*>(text), nullptr, xplmFont_Proportional);
}
}

Tablet::Tablet(SceneryManager& scenery, std::function<void()> toggle_auto)
    : scenery_(scenery), toggle_auto_(std::move(toggle_auto)) {
  XPLMCreateWindow_t params{};
  params.structSize = sizeof(params);
  params.left = 100; params.top = 700; params.right = 470; params.bottom = 220;
  params.visible = 0;
  params.drawWindowFunc = draw;
  params.handleMouseClickFunc = mouse;
  params.refcon = this;
  params.layer = xplm_WindowLayerFloatingWindows;
  params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
  window_ = XPLMCreateWindowEx(&params);
  XPLMSetWindowTitle(window_, "SSA - Scenery Service Animation");
}

Tablet::~Tablet() { if (window_) XPLMDestroyWindow(window_); }
void Tablet::toggle() { XPLMSetWindowIsVisible(window_, visible() ? 0 : 1); }
bool Tablet::visible() const { return window_ && XPLMGetWindowIsVisible(window_) != 0; }
void Tablet::set_position(double latitude, double longitude) { latitude_ = latitude; longitude_ = longitude; }
void Tablet::draw(XPLMWindowID, void* refcon) { static_cast<Tablet*>(refcon)->draw_impl(); }
int Tablet::mouse(XPLMWindowID, int x, int y, XPLMMouseStatus status, void* refcon) {
  return static_cast<Tablet*>(refcon)->mouse_impl(x, y, status);
}

void Tablet::draw_impl() {
  int l, t, r, b;
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
  label(l + 22, t - 35, "BOLDSTUDIO31  |  SSA", 0.25f, 0.95f, 0.65f);
  label(l + 22, t - 70, tab_ == 0 ? "[ HANGAR ]    JETWAY" : "HANGAR    [ JETWAY ]");
  if (tab_ == 1) {
    label(l + 22, t - 105, automatic_ ? "Automatic jetway: ON" : "Automatic jetway: OFF");
    label(l + 22, t - 130, "Turboprop: 0 | Narrow: 1 | Wide: by forward doors");
  } else {
    label(l + 22, t - 105, "Nearby hangars (maximum 2 km):");
  }
  auto list = scenery_.nearby(tab_ == 0 ? ServiceType::Hangar : ServiceType::Jetway,
                              latitude_, longitude_, tab_ == 0 ? 2000.0 : 35.0);
  int y = t - 165;
  for (size_t i = 0; i < std::min<size_t>(list.size(), 8); ++i, y -= 34) {
    const auto* object = list[i];
    char line[180];
    std::snprintf(line, sizeof(line), "%zu. %s   [%s]  %3.0f%%", i + 1, object->label.c_str(),
                  object->target > 0.5f ? "CLOSE" : "OPEN", object->progress * 100.0f);
    label(l + 26, y, line);
  }
  if (list.empty()) label(l + 26, y, "No SSA object found in range.", 1.0f, 0.65f, 0.35f);
  label(l + 22, b + 22, "Click tabs, AUTO text, or an object row");
}

int Tablet::mouse_impl(int x, int y, XPLMMouseStatus status) {
  if (status != xplm_MouseDown) return 1;
  int l, t, r, b;
  XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
  if (y < t - 45 && y > t - 85) { tab_ = x < l + 155 ? 0 : 1; return 1; }
  if (tab_ == 1 && y < t - 85 && y > t - 145) { toggle_auto_(); return 1; }
  const int index = (t - 145 - y) / 34;
  auto list = scenery_.nearby(tab_ == 0 ? ServiceType::Hangar : ServiceType::Jetway,
                              latitude_, longitude_, tab_ == 0 ? 2000.0 : 35.0);
  if (index >= 0 && static_cast<size_t>(index) < list.size())
    list[index]->target = list[index]->target > 0.5f ? 0.0f : 1.0f;
  return 1;
}

} // namespace ssa

