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
  float handle_in_x{};
  float handle_in_z{};
  float handle_out_x{};
  float handle_out_z{};
  bool custom_handles{};
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
  void toggle_loop();
  void start_saved_route();
  void stop_saved_route();
  void cancel();
  bool save();

  RouteEditorState state() const { return state_; }
  const std::string& model_label() const { return model_label_; }
  const std::string& status() const { return status_; }
  size_t point_count() const { return points_.size(); }
  float heading() const { return current_.heading; }
  bool loop_enabled() const { return loop_enabled_; }
  bool saved_route_available() const { return saved_route_available_; }
  bool saved_route_running() const { return runtime_playback_; }
  bool saved_route_loop() const { return saved_loop_enabled_; }
  bool saved_route_autostart() const { return saved_autostart_; }
  const std::string& saved_route_label() const { return saved_route_label_; }

private:
  float terrain_y(float x, float z, float fallback) const;
  void show(float spin, float steering);
  void add_point_at(float x, float z);
  bool load_saved_route();
  void build_bezier_path();
  void update_planner_drag();
  void open_planner();
  void close_planner();
  void draw_planner();
  int planner_mouse(int x, int y, XPLMMouseStatus status);
  int planner_right_mouse(int x, int y, XPLMMouseStatus status);
  int planner_wheel(int x, int y, int wheel, int clicks);
  float planner_half_width() const;
  static void planner_draw(XPLMWindowID, void* refcon);
  static int planner_mouse_cb(XPLMWindowID, int x, int y, XPLMMouseStatus status,
                              void* refcon);
  static int planner_right_mouse_cb(XPLMWindowID, int x, int y,
                                    XPLMMouseStatus status, void* refcon);
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
  float acceleration_mps2_{1.5f};
  float braking_mps2_{2.5f};
  float heading_offset_deg_{180.0f};
  float steering_multiplier_{-1.0f};
  float body_lookahead_m_{6.0f};
  float body_heading_response_{1.8f};
  float rear_axle_to_origin_m_{3.8f};
  float wheelbase_m_{7.6f};
  float max_steering_deg_{35.0f};
  float spin_{};
  float current_speed_mps_{};
  float display_steering_{};
  float planner_height_m_{160.0f};
  float planner_center_x_{};
  float planner_center_y_{};
  float planner_center_z_{};
  bool planner_active_{};
  bool planner_drag_active_{};
  int handle_drag_anchor_{-1};
  int planner_drag_x_{};
  int planner_drag_y_{};
  bool return_to_planner_after_test_{};
  bool loop_enabled_{};
  bool saved_route_available_{};
  bool saved_loop_enabled_{};
  bool saved_autostart_{true};
  bool runtime_playback_{};
  std::string saved_route_label_{"No saved route"};
  size_t test_index_{};
  RoutePoint current_{};
  std::vector<RoutePoint> points_;
  std::vector<RoutePoint> saved_points_;
  std::vector<RoutePoint> test_points_;
  std::vector<float> test_distance_remaining_;
  XPLMObjectRef object_{};
  XPLMInstanceRef instance_{};
  XPLMProbeRef probe_{};
  XPLMWindowID planner_window_{};
  XPLMDataRef field_of_view_ref_{};
};

} // namespace ssa
