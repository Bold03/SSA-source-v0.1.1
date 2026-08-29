#pragma once

#include <XPLMInstance.h>
#include <XPLMScenery.h>
#include <array>
#include <string>
#include <vector>

namespace ssa {

enum class VdgsEditorState { Unavailable, Idle, Placing };

struct VdgsPlacement {
  std::string id;
  std::string object_path;
  double latitude{};
  double longitude{};
  double altitude_m{};
  float heading{};
  XPLMObjectRef object{};
  XPLMInstanceRef instance{};
  std::array<float, 8> data{};
};

class VdgsEditor {
public:
  ~VdgsEditor();
  bool load(const std::string& xplane_root);
  void unload();
  void update();
  void begin(double aircraft_latitude, double aircraft_longitude,
             float aircraft_heading);
  void move_forward(float metres);
  void move_side(float metres);
  void turn(float degrees);
  void adjust_altitude(float metres);
  void cancel();
  bool save();
  void clear_guidance();
  void set_guidance(const std::string& id, const std::array<float, 8>& data);

  VdgsEditorState state() const { return state_; }
  const std::string& status() const { return status_; }
  double latitude() const { return latitude_; }
  double longitude() const { return longitude_; }
  double altitude_m() const { return altitude_m_; }
  float heading() const { return heading_; }

private:
  static float normalize_heading(float heading);
  float terrain_y(float x, float z, float fallback) const;
  void show_preview();
  bool load_saved(const std::string& config_path);
  bool create_saved_instance(VdgsPlacement& placement);

  VdgsEditorState state_{VdgsEditorState::Unavailable};
  std::string xplane_root_;
  std::string scenery_directory_;
  std::string config_path_;
  std::string relative_object_path_;
  std::string absolute_object_path_;
  std::string status_{"VDGS model not found"};
  double latitude_{};
  double longitude_{};
  double altitude_m_{};
  float heading_{};
  float x_{};
  float y_{};
  float z_{};
  float ground_y_{};
  float altitude_offset_m_{};
  XPLMObjectRef preview_object_{};
  XPLMInstanceRef preview_instance_{};
  XPLMProbeRef probe_{};
  std::vector<VdgsPlacement> placements_;
};

} // namespace ssa
