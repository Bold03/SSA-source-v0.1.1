#pragma once

#include <XPLMScenery.h>
#include <XPLMInstance.h>
#include <string>
#include <vector>

namespace ssa {

enum class RouteEditorState { Unavailable, Idle, Editing, Testing };

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
  static float heading_delta(float from, float to);

  RouteEditorState state_{RouteEditorState::Unavailable};
  std::string model_id_;
  std::string model_label_;
  std::string scenery_directory_;
  std::string route_path_;
  std::string status_{"No vehicle model configured"};
  float ground_offset_m_{0.445f};
  float speed_mps_{4.0f};
  float spin_{};
  size_t test_index_{};
  RoutePoint current_{};
  std::vector<RoutePoint> points_;
  XPLMObjectRef object_{};
  XPLMInstanceRef instance_{};
  XPLMProbeRef probe_{};
};

} // namespace ssa
