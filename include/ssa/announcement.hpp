#pragma once

#include <AL/al.h>
#include <AL/alc.h>
#include <XPLMDataAccess.h>
#include <string>
#include <vector>

namespace ssa {

struct AnnouncementSource {
  std::string id;
  std::string label;
  std::vector<std::string> audio_paths;
  unsigned int gap_ms{100};
  double latitude{};
  double longitude{};
  double altitude_m{};
  float x{};
  float y{};
  float z{};
  float gain{1.0f};
  float radius_m{150.0f};
  float repeat_interval_s{60.0f};
  float start_delay_s{};
  float timer{};
  bool autoplay{true};
  bool loop{};
  bool played_once{};
  ALuint buffer{};
  ALuint source{};
};

class AnnouncementManager {
public:
  ~AnnouncementManager();
  bool load(const std::string& xplane_root);
  void unload();
  void update(float elapsed_seconds);
  const std::string& status() const { return status_; }
  size_t source_count() const { return sources_.size(); }

private:
  bool initialize_audio();
  bool load_audio(AnnouncementSource& announcement);
  void shutdown_audio();

  ALCdevice* device_{};
  ALCcontext* context_{};
  XPLMDataRef view_x_ref_{};
  XPLMDataRef view_y_ref_{};
  XPLMDataRef view_z_ref_{};
  XPLMDataRef view_heading_ref_{};
  XPLMDataRef view_pitch_ref_{};
  std::vector<AnnouncementSource> sources_;
  std::string status_{"3D announcements not loaded"};
};

} // namespace ssa
