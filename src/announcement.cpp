#include "ssa/announcement.hpp"
#include <XPLMGraphics.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace ssa {
namespace {
constexpr float radians = 3.14159265358979323846f / 180.0f;

std::uint16_t read_u16(std::istream& input) {
  std::array<unsigned char, 2> bytes{};
  input.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
  return static_cast<std::uint16_t>(bytes[0] | (bytes[1] << 8));
}

std::uint32_t read_u32(std::istream& input) {
  std::array<unsigned char, 4> bytes{};
  input.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
  return static_cast<std::uint32_t>(bytes[0] | (bytes[1] << 8) |
                                    (bytes[2] << 16) | (bytes[3] << 24));
}
}

AnnouncementManager::~AnnouncementManager() { shutdown_audio(); }

bool AnnouncementManager::initialize_audio() {
  if (context_) return true;
  device_ = alcOpenDevice(nullptr);
  if (!device_) {
    status_ = "Cannot open 3D audio device";
    return false;
  }
  context_ = alcCreateContext(device_, nullptr);
  if (!context_ || alcMakeContextCurrent(context_) == ALC_FALSE) {
    status_ = "Cannot create 3D audio context";
    shutdown_audio();
    return false;
  }
  alDistanceModel(AL_INVERSE_DISTANCE_CLAMPED);
  return true;
}

void AnnouncementManager::unload() {
  if (context_) alcMakeContextCurrent(context_);
  for (auto& announcement : sources_) {
    if (announcement.source) alDeleteSources(1, &announcement.source);
    if (announcement.buffer) alDeleteBuffers(1, &announcement.buffer);
  }
  sources_.clear();
}

void AnnouncementManager::shutdown_audio() {
  unload();
  if (context_) {
    alcMakeContextCurrent(nullptr);
    alcDestroyContext(context_);
  }
  if (device_) alcCloseDevice(device_);
  context_ = nullptr;
  device_ = nullptr;
}

bool AnnouncementManager::load_audio(AnnouncementSource& announcement) {
  if (announcement.audio_paths.empty()) return false;
  std::vector<std::int16_t> combined;
  std::uint32_t output_rate{};

  for (const auto& audio_path : announcement.audio_paths) {
  std::ifstream input(audio_path, std::ios::binary);
  char riff[4]{}, wave[4]{};
  input.read(riff, 4);
  (void)read_u32(input);
  input.read(wave, 4);
  if (!input || std::string(riff, 4) != "RIFF" || std::string(wave, 4) != "WAVE")
    return false;

  std::uint16_t format{}, channels{}, bits{};
  std::uint32_t sample_rate{};
  std::vector<char> pcm;
  while (input && (format == 0 || pcm.empty())) {
    char chunk_id[4]{};
    input.read(chunk_id, 4);
    if (!input) break;
    const std::uint32_t chunk_size = read_u32(input);
    const std::string id(chunk_id, 4);
    if (id == "fmt ") {
      format = read_u16(input);
      channels = read_u16(input);
      sample_rate = read_u32(input);
      input.seekg(6, std::ios::cur);
      bits = read_u16(input);
      if (chunk_size > 16) input.seekg(chunk_size - 16, std::ios::cur);
    } else if (id == "data") {
      pcm.resize(chunk_size);
      input.read(pcm.data(), static_cast<std::streamsize>(pcm.size()));
    } else {
      input.seekg(chunk_size, std::ios::cur);
    }
    if ((chunk_size & 1U) != 0U) input.seekg(1, std::ios::cur);
  }
  if (!input || format != 1 || channels != 1 || bits != 16 ||
      sample_rate == 0 || pcm.empty())
    return false;

  const auto* raw = reinterpret_cast<const std::int16_t*>(pcm.data());
  const size_t sample_count = pcm.size() / sizeof(std::int16_t);
  if (output_rate == 0) output_rate = sample_rate;
  std::vector<std::int16_t> converted;
  if (sample_rate == output_rate) {
    converted.assign(raw, raw + sample_count);
  } else {
    const size_t converted_count = std::max<size_t>(
        1, static_cast<size_t>(std::llround(
               static_cast<double>(sample_count) * output_rate / sample_rate)));
    converted.resize(converted_count);
    for (size_t i = 0; i < converted_count; ++i) {
      const double source_position =
          static_cast<double>(i) * sample_rate / output_rate;
      const size_t lower =
          std::min(static_cast<size_t>(source_position), sample_count - 1);
      const size_t upper = std::min(lower + 1, sample_count - 1);
      const double blend = source_position - static_cast<double>(lower);
      converted[i] = static_cast<std::int16_t>(
          std::llround(raw[lower] * (1.0 - blend) + raw[upper] * blend));
    }
  }
  if (!combined.empty()) {
    const size_t silence_samples =
        static_cast<size_t>((static_cast<std::uint64_t>(output_rate) *
                             announcement.gap_ms) /
                            1000U);
    combined.insert(combined.end(), silence_samples, 0);
  }
  combined.insert(combined.end(), converted.begin(), converted.end());
  }

  if (combined.empty() || output_rate == 0) return false;
  alGenBuffers(1, &announcement.buffer);
  alBufferData(announcement.buffer, AL_FORMAT_MONO16, combined.data(),
               static_cast<ALsizei>(combined.size() * sizeof(std::int16_t)),
               static_cast<ALsizei>(output_rate));
  alGenSources(1, &announcement.source);
  alSourcei(announcement.source, AL_BUFFER,
            static_cast<ALint>(announcement.buffer));
  alSourcei(announcement.source, AL_SOURCE_RELATIVE, AL_TRUE);
  alSourcei(announcement.source, AL_LOOPING,
            announcement.loop ? AL_TRUE : AL_FALSE);
  alSourcef(announcement.source, AL_GAIN, announcement.gain);
  alSourcef(announcement.source, AL_REFERENCE_DISTANCE,
            std::max(1.0f, announcement.radius_m * 0.15f));
  alSourcef(announcement.source, AL_MAX_DISTANCE, announcement.radius_m);
  alSourcef(announcement.source, AL_ROLLOFF_FACTOR, 1.0f);
  return alGetError() == AL_NO_ERROR;
}

bool AnnouncementManager::load(const std::string& xplane_root) {
  unload();
  if (!initialize_audio()) return false;
  view_x_ref_ = XPLMFindDataRef("sim/graphics/view/view_x");
  view_y_ref_ = XPLMFindDataRef("sim/graphics/view/view_y");
  view_z_ref_ = XPLMFindDataRef("sim/graphics/view/view_z");
  view_heading_ref_ = XPLMFindDataRef("sim/graphics/view/view_heading");
  view_pitch_ref_ = XPLMFindDataRef("sim/graphics/view/view_pitch");

  const fs::path custom = fs::path(xplane_root) / "Custom Scenery";
  if (!fs::exists(custom)) return false;
  try {
    for (const auto& entry : fs::directory_iterator(custom)) {
      const fs::path config_path = entry.path() / "ssa.json";
      if (!entry.is_directory() || !fs::exists(config_path)) continue;
      std::ifstream config_file(config_path);
      const json root = json::parse(config_file);
      auto configure_source = [&](AnnouncementSource& announcement,
                                  const json& item) {
        if (!item.contains("latitude") || !item.contains("longitude"))
          return false;
        announcement.latitude = item.at("latitude").get<double>();
        announcement.longitude = item.at("longitude").get<double>();
        announcement.altitude_m = item.value("altitude_m", 0.0);
        announcement.gain =
            std::clamp(item.value("gain", 1.0f), 0.0f, 2.0f);
        announcement.radius_m =
            std::clamp(item.value("radius_m", 150.0f), 5.0f, 2000.0f);
        announcement.repeat_interval_s =
            std::max(1.0f, item.value("repeat_interval_s", 60.0f));
        announcement.start_delay_s =
            std::max(0.0f, item.value("start_delay_s", 0.0f));
        announcement.autoplay = item.value("autoplay", true);
        announcement.loop = item.value("loop", false);
        announcement.timer = -announcement.start_delay_s;
        double local_x{}, local_y{}, local_z{};
        XPLMWorldToLocal(announcement.latitude, announcement.longitude,
                         announcement.altitude_m, &local_x, &local_y, &local_z);
        announcement.x = static_cast<float>(local_x);
        announcement.y = static_cast<float>(local_y);
        announcement.z = static_cast<float>(local_z);
        return true;
      };

      if (root.contains("announcements") &&
          root.at("announcements").is_array()) {
      for (const auto& item : root.at("announcements")) {
        if (!item.contains("audio") || !item.contains("latitude") ||
            !item.contains("longitude")) continue;
        AnnouncementSource announcement;
        announcement.id = item.value("id", std::string("announcement"));
        announcement.label = item.value("label", announcement.id);
        announcement.audio_paths.push_back(
            (entry.path() / item.at("audio").get<std::string>()).string());
        announcement.gap_ms = static_cast<unsigned int>(
            std::clamp(item.value("gap_ms", 100), 0, 2000));
        if (configure_source(announcement, item) && load_audio(announcement))
          sources_.push_back(std::move(announcement));
      }
      }

      if (!root.contains("announcement_library") ||
          !root.contains("flight_announcements") ||
          !root.at("flight_announcements").is_array())
        continue;

      const auto& library = root.at("announcement_library");
      const fs::path audio_folder =
          entry.path() /
          library.value("base_folder", std::string("audio/announcements"));
      const auto clip_path = [&](const char* group,
                                 const std::string& key) -> std::string {
        if (key.empty() || !library.contains(group) ||
            !library.at(group).contains(key.c_str()))
          return {};
        return (audio_folder /
                library.at(group).at(key.c_str()).get<std::string>()).string();
      };

      for (const auto& item : root.at("flight_announcements")) {
        if (!item.contains("airline") || !item.contains("flight_number"))
          continue;
        AnnouncementSource announcement;
        announcement.id = item.value("id", std::string("flight_announcement"));
        announcement.label = item.value("label", announcement.id);
        announcement.gap_ms = static_cast<unsigned int>(std::clamp(
            item.value("gap_ms", library.value("gap_ms", 100)), 0, 2000));

        bool complete = true;
        const auto add_named_clip = [&](const char* group,
                                        const std::string& key) {
          const std::string path = clip_path(group, key);
          if (path.empty()) complete = false;
          else announcement.audio_paths.push_back(path);
        };
        const auto add_digits = [&](const std::string& digits) {
          for (const char digit : digits) {
            if (digit < '0' || digit > '9') {
              complete = false;
              continue;
            }
            add_named_clip("digits", std::string(1, digit));
          }
        };

        add_named_clip("airlines", item.at("airline").get<std::string>());
        add_digits(item.at("flight_number").get<std::string>());
        if (item.contains("origin"))
          add_named_clip("origins", item.at("origin").get<std::string>());
        if (item.contains("destination"))
          add_named_clip("destinations",
                         item.at("destination").get<std::string>());
        if (item.contains("event"))
          add_named_clip("events", item.at("event").get<std::string>());
        if (item.contains("gate"))
          add_digits(item.at("gate").get<std::string>());

        if (complete && configure_source(announcement, item) &&
            load_audio(announcement))
          sources_.push_back(std::move(announcement));
      }
    }
    status_ = std::to_string(sources_.size()) + " 3D announcement(s) ready";
    return true;
  } catch (const std::exception& error) {
    status_ = error.what();
    unload();
    return false;
  }
}

void AnnouncementManager::update(float elapsed_seconds) {
  if (!context_ || sources_.empty()) return;
  alcMakeContextCurrent(context_);
  const float camera_x = view_x_ref_ ? XPLMGetDataf(view_x_ref_) : 0.0f;
  const float camera_y = view_y_ref_ ? XPLMGetDataf(view_y_ref_) : 0.0f;
  const float camera_z = view_z_ref_ ? XPLMGetDataf(view_z_ref_) : 0.0f;
  const float heading =
      (view_heading_ref_ ? XPLMGetDataf(view_heading_ref_) : 0.0f) * radians;
  const float pitch =
      (view_pitch_ref_ ? XPLMGetDataf(view_pitch_ref_) : 0.0f) * radians;
  const float sin_h = std::sin(heading), cos_h = std::cos(heading);
  const float sin_p = std::sin(pitch), cos_p = std::cos(pitch);
  const std::array<float, 3> right{cos_h, 0.0f, sin_h};
  const std::array<float, 3> forward{sin_h * cos_p, sin_p, -cos_h * cos_p};
  const std::array<float, 3> up{-sin_h * sin_p, cos_p, cos_h * sin_p};
  alListener3f(AL_POSITION, 0.0f, 0.0f, 0.0f);
  const float orientation[] = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f};
  alListenerfv(AL_ORIENTATION, orientation);

  for (auto& announcement : sources_) {
    const std::array<float, 3> delta{announcement.x - camera_x,
                                     announcement.y - camera_y,
                                     announcement.z - camera_z};
    const auto dot = [&](const std::array<float, 3>& axis) {
      return delta[0] * axis[0] + delta[1] * axis[1] + delta[2] * axis[2];
    };
    const float local_x = dot(right);
    const float local_y = dot(up);
    const float local_z = -dot(forward);
    alSource3f(announcement.source, AL_POSITION, local_x, local_y, local_z);
    const float distance =
        std::sqrt(local_x * local_x + local_y * local_y + local_z * local_z);
    ALint state{};
    alGetSourcei(announcement.source, AL_SOURCE_STATE, &state);
    if (!announcement.autoplay || state == AL_PLAYING ||
        distance > announcement.radius_m) continue;
    announcement.timer += elapsed_seconds;
    const float wait = announcement.played_once
                           ? announcement.repeat_interval_s
                           : 0.0f;
    if (announcement.timer < wait) continue;
    alSourcePlay(announcement.source);
    announcement.played_once = true;
    announcement.timer = 0.0f;
  }
}

} // namespace ssa
