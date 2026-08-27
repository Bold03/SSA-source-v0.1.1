#pragma once

#include <XPLMScenery.h>
#include <XPLMInstance.h>
#include <XPLMCamera.h>
#include <XPLMDisplay.h>
#include <XPLMDataAccess.h>
#include <string>
#include <vector>

namespace ssa {

enum class RouteEditorState { Unavailable, Idle, Editing, Planning, Testing };

struct RoutePoint {
  float x{};
  float y{};
  float z{};
  float heading{};
};

class RouteEditor {
public:
  RouteEditor() = default;
  ~RouteEditor();
  bool load(const std::string& xplane_root);
  void unload();
  void update(float elapsed_seconds);

  void create_route(double latitude, double longitude, float heading);
  void begin_planner(double latitude, double longitude, float heading);
  void move(float metres);
  void turn(float degrees);
  void add_point();
  void undo_point();
  void start_test();
  void stop_test();
  void cancel();
  bool save();

  RouteEditorState state() const { return state_; }
  const std::string& model_label() const { return model_label_; }
  const std::string& status() const { return status_; }
  size_t point_count() const { return points_.size(); }
  float heading() const { return current_.heading; }

private:
  float terrain_y(float x, float z, float fallback) const;
  void show(float spin, float steering);
  void add_point_at(float x, float z);
  void close_planner();
  void draw_planner();
  int planner_mouse(int x, int y, XPLMMouseStatus status);
  int planner_wheel(int x, int y, int wheel, int clicks);
  float planner_half_width() const;
  static void planner_draw(XPLMWindowID, void* refcon);
  static int planner_mouse_cb(XPLMWindowID, int x, int y, XPLMMouseStatus status,
                              void* refcon);
  static void planner_key(XPLMWindowID, char, XPLMKeyFlags, char, void*, int);
  static XPLMCursorStatus planner_cursor(XPLMWindowID, int, int, void*);
  static int planner_wheel_cb(XPLMWindowID, int x, int y, int wheel, int clicks,
                              void* refcon);
  static int camera_control(XPLMCameraPosition_t* position, int losing_control, void* refcon);
  static float heading_delta(float from, float to);

  RouteEditorState state_{RouteEditorState::Unavailable};
  std::string model_id_;
  std::string model_label_;
  std::string scenery_directory_;
  std::string route_path_;
  std::string status_{"No vehicle model configured"};
  float ground_offset_m_{0.445f};
  float speed_mps_{4.0f};
  float heading_offset_deg_{180.0f};
  float spin_{};
  float planner_height_m_{160.0f};
  float planner_center_x_{};
  float planner_center_y_{};
  float planner_center_z_{};
  bool planner_active_{};
  size_t test_index_{};
  RoutePoint current_{};
  std::vector<RoutePoint> points_;
  XPLMObjectRef object_{};
  XPLMInstanceRef instance_{};
  XPLMProbeRef probe_{};
  XPLMWindowID planner_window_{};
  XPLMDataRef field_of_view_ref_{};
};

} // namespace ssa
