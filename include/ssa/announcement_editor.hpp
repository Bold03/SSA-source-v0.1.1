#pragma once

#include <XPLMDataAccess.h>
#include <XPLMDisplay.h>
#include <XPLMScenery.h>
#include <string>
#include <vector>

namespace ssa {

enum class AnnouncementEditorState { Unavailable, Idle, Placing };

class AnnouncementEditor {
public:
  ~AnnouncementEditor();
  bool load(const std::string& xplane_root);
  void unload();
  void begin(double aircraft_latitude, double aircraft_longitude,
             float aircraft_heading);
  void move_forward(float metres);
  void move_side(float metres);
  void move_world_x(float metres);
  void move_world_z(float metres);
  void adjust_altitude(float metres);
  void cancel();
  bool save();
  bool request_delete_nearest(double aircraft_latitude,
                              double aircraft_longitude);

  void next_airline(int direction);
  void next_origin(int direction);
  void next_destination(int direction);
  void next_event(int direction);
  void adjust_flight_number(int amount);
  void adjust_gate(int amount);
  void adjust_gain(float amount);
  void adjust_radius(float amount);
  bool detect_airline_from_livery();

  AnnouncementEditorState state() const { return state_; }
  const std::string& status() const { return status_; }
  const std::string& airline() const;
  const std::string& origin() const;
  const std::string& destination() const;
  const std::string& event() const;
  const std::string& livery_name() const { return livery_name_; }
  bool airline_auto_detected() const { return airline_auto_detected_; }
  bool delete_confirmation_pending() const {
    return !pending_delete_id_.empty();
  }
  int flight_number() const { return flight_number_; }
  int gate() const { return gate_; }
  float gain() const { return gain_; }
  float radius_m() const { return radius_m_; }
  double latitude() const { return latitude_; }
  double longitude() const { return longitude_; }
  double altitude_m() const { return altitude_m_; }

private:
  enum class GizmoDrag { None, MoveX, MoveY, MoveZ };
  static void draw_callback(XPLMWindowID window, void* refcon);
  static void draw_guide_callback(XPLMWindowID window, void* refcon);
  static int mouse_callback(XPLMWindowID window, int x, int y,
                            XPLMMouseStatus status, void* refcon);
  static XPLMCursorStatus cursor_callback(XPLMWindowID window, int x, int y,
                                          void* refcon);
  void open_overlay();
  void close_overlay();
  void draw_overlay();
  void draw_radius_guides();
  int overlay_mouse(int x, int y, XPLMMouseStatus status);
  bool project_to_screen(int& screen_x, int& screen_y) const;
  bool project_point(float x, float y, float z, int& screen_x,
                     int& screen_y) const;
  bool axis_direction(GizmoDrag axis, float& screen_dx,
                      float& screen_dy) const;
  float terrain_y(float x, float z, float fallback) const;
  void update_world_position();
  static void cycle_index(size_t& index, int direction, size_t count);

  AnnouncementEditorState state_{AnnouncementEditorState::Unavailable};
  std::string config_path_;
  std::string status_{"Announcement library not found"};
  std::string livery_name_;
  std::string pending_delete_id_;
  std::vector<std::string> airlines_;
  std::vector<std::string> origins_;
  std::vector<std::string> destinations_;
  std::vector<std::string> events_;
  size_t airline_index_{};
  size_t origin_index_{};
  size_t destination_index_{};
  size_t event_index_{};
  int flight_number_{684};
  int gate_{1};
  float gain_{0.8f};
  float radius_m_{150.0f};
  float aircraft_heading_{};
  double latitude_{};
  double longitude_{};
  double altitude_m_{};
  float x_{};
  float y_{};
  float z_{};
  float ground_y_{};
  float altitude_offset_m_{4.0f};
  bool airline_auto_detected_{};
  XPLMProbeRef probe_{};
  XPLMDataRef livery_path_ref_{};
  XPLMDataRef world_matrix_ref_{};
  XPLMDataRef projection_matrix_ref_{};
  XPLMWindowID overlay_window_{};
  XPLMWindowID guide_window_{};
  GizmoDrag gizmo_drag_{GizmoDrag::None};
  int drag_last_x_{};
  int drag_last_y_{};
};

} // namespace ssa
