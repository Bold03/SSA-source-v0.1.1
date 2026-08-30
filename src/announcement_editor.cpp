#include "ssa/announcement_editor.hpp"
#include <XPLMGraphics.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace ssa {
namespace {
constexpr float pi = 3.14159265358979323846f;

void multiply_matrix_vector(float destination[4], const float matrix[16],
                            const float vector[4]) {
  destination[0] = vector[0] * matrix[0] + vector[1] * matrix[4] +
                   vector[2] * matrix[8] + vector[3] * matrix[12];
  destination[1] = vector[0] * matrix[1] + vector[1] * matrix[5] +
                   vector[2] * matrix[9] + vector[3] * matrix[13];
  destination[2] = vector[0] * matrix[2] + vector[1] * matrix[6] +
                   vector[2] * matrix[10] + vector[3] * matrix[14];
  destination[3] = vector[0] * matrix[3] + vector[1] * matrix[7] +
                   vector[2] * matrix[11] + vector[3] * matrix[15];
}

void draw_text(int x, int y, char* text, float r, float g, float b) {
  float color[] = {r, g, b};
  XPLMDrawString(color, x, y, text, nullptr, xplmFont_Proportional);
}

std::string lower_copy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    if (c == '_' || c == '-' || c == '/' || c == '\\') return ' ';
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

float distance_to_segment(float px, float py, float ax, float ay,
                          float bx, float by) {
  const float vx = bx - ax;
  const float vy = by - ay;
  const float length_squared = vx * vx + vy * vy;
  if (length_squared < 0.001f) return std::numeric_limits<float>::max();
  const float t = std::clamp(((px - ax) * vx + (py - ay) * vy) /
                                 length_squared,
                             0.0f, 1.0f);
  return std::hypot(px - (ax + t * vx), py - (ay + t * vy));
}

void draw_dotted_axis(int cx, int cy, float dx, float dy, float r, float g,
                      float b, const char* axis) {
  constexpr float start = 28.0f;
  constexpr float length = 92.0f;
  for (int i = 0; i < 9; ++i) {
    const float distance = start + (length - start) * i / 8.0f;
    const int x = cx + static_cast<int>(dx * distance);
    const int y = cy + static_cast<int>(dy * distance);
    char dot[] = "+";
    draw_text(x - 3, y - 4, dot, r, g, b);
  }
  const int end_x = cx + static_cast<int>(dx * length);
  const int end_y = cy + static_cast<int>(dy * length);
  XPLMDrawTranslucentDarkBox(end_x - 13, end_y + 13, end_x + 13,
                             end_y - 13);
  draw_text(end_x - 4, end_y - 5, const_cast<char*>(axis), r, g, b);
}
} // namespace

AnnouncementEditor::~AnnouncementEditor() { unload(); }

void AnnouncementEditor::unload() {
  close_overlay();
  if (overlay_window_) XPLMDestroyWindow(overlay_window_);
  if (guide_window_) XPLMDestroyWindow(guide_window_);
  if (probe_) XPLMDestroyProbe(probe_);
  overlay_window_ = nullptr;
  guide_window_ = nullptr;
  probe_ = nullptr;
  world_matrix_ref_ = nullptr;
  projection_matrix_ref_ = nullptr;
  livery_path_ref_ = nullptr;
  config_path_.clear();
  airlines_.clear();
  origins_.clear();
  destinations_.clear();
  events_.clear();
  livery_name_.clear();
  pending_delete_id_.clear();
  airline_auto_detected_ = false;
  state_ = AnnouncementEditorState::Unavailable;
}

bool AnnouncementEditor::load(const std::string& xplane_root) {
  unload();
  try {
    const fs::path custom = fs::path(xplane_root) / "Custom Scenery";
    if (!fs::exists(custom)) {
      status_ = "Custom Scenery folder not found";
      return false;
    }
    for (const auto& entry : fs::directory_iterator(custom)) {
      const fs::path config = entry.path() / "ssa.json";
      if (!entry.is_directory() || !fs::exists(config)) continue;
      std::ifstream input(config);
      const json root = json::parse(input);
      if (!root.contains("announcement_library")) continue;
      const auto& library = root.at("announcement_library");
      const auto read_keys = [&](const char* group,
                                 std::vector<std::string>& destination) {
        if (!library.contains(group)) return;
        for (const auto& [key, value] : library.at(group).items()) {
          (void)value;
          destination.push_back(key);
        }
        std::sort(destination.begin(), destination.end());
      };
      read_keys("airlines", airlines_);
      read_keys("origins", origins_);
      read_keys("destinations", destinations_);
      read_keys("events", events_);
      if (!airlines_.empty() && !events_.empty()) {
        config_path_ = config.string();
        break;
      }
      airlines_.clear();
      origins_.clear();
      destinations_.clear();
      events_.clear();
    }
    if (config_path_.empty()) {
      status_ = "Add announcement_library to scenery ssa.json";
      return false;
    }
    probe_ = XPLMCreateProbe(xplm_ProbeY);
    world_matrix_ref_ = XPLMFindDataRef("sim/graphics/view/world_matrix");
    projection_matrix_ref_ =
        XPLMFindDataRef("sim/graphics/view/projection_matrix_3d");
    livery_path_ref_ =
        XPLMFindDataRef("sim/aircraft/view/acf_livery_path");
    if (!probe_) {
      status_ = "Cannot create terrain probe";
      return false;
    }
    state_ = AnnouncementEditorState::Idle;
    if (!detect_airline_from_livery())
      status_ = "Composer ready | airline can be selected manually";
    return true;
  } catch (const std::exception& error) {
    status_ = error.what();
    return false;
  }
}

const std::string& AnnouncementEditor::airline() const {
  static const std::string none{"none"};
  return airlines_.empty() ? none : airlines_[airline_index_];
}
const std::string& AnnouncementEditor::origin() const {
  static const std::string none{"none"};
  return origins_.empty() ? none : origins_[origin_index_];
}
const std::string& AnnouncementEditor::destination() const {
  static const std::string none{"none"};
  return destinations_.empty() ? none : destinations_[destination_index_];
}
const std::string& AnnouncementEditor::event() const {
  static const std::string none{"none"};
  return events_.empty() ? none : events_[event_index_];
}

void AnnouncementEditor::cycle_index(size_t& index, int direction,
                                     size_t count) {
  if (count == 0) { index = 0; return; }
  if (direction >= 0) index = (index + 1) % count;
  else index = index == 0 ? count - 1 : index - 1;
}
void AnnouncementEditor::next_airline(int direction) {
  cycle_index(airline_index_, direction, airlines_.size());
  airline_auto_detected_ = false;
  status_ = "Airline selected manually: " + airline();
}
void AnnouncementEditor::next_origin(int direction) { cycle_index(origin_index_, direction, origins_.size()); }
void AnnouncementEditor::next_destination(int direction) { cycle_index(destination_index_, direction, destinations_.size()); }
void AnnouncementEditor::next_event(int direction) { cycle_index(event_index_, direction, events_.size()); }
void AnnouncementEditor::adjust_flight_number(int amount) { flight_number_ = std::clamp(flight_number_ + amount, 1, 9999); }
void AnnouncementEditor::adjust_gate(int amount) { gate_ = std::clamp(gate_ + amount, 1, 999); }
void AnnouncementEditor::adjust_gain(float amount) { gain_ = std::clamp(gain_ + amount, 0.1f, 2.0f); }
void AnnouncementEditor::adjust_radius(float amount) { radius_m_ = std::clamp(radius_m_ + amount, 10.0f, 2000.0f); }

bool AnnouncementEditor::detect_airline_from_livery() {
  airline_auto_detected_ = false;
  livery_name_.clear();
  if (!livery_path_ref_ || airlines_.empty()) return false;
  const int byte_count = XPLMGetDatab(livery_path_ref_, nullptr, 0, 0);
  if (byte_count <= 0) return false;
  const int safe_byte_count = std::min(byte_count, 4096);
  std::vector<char> bytes(static_cast<size_t>(safe_byte_count) + 1, '\0');
  const int copied =
      XPLMGetDatab(livery_path_ref_, bytes.data(), 0, safe_byte_count);
  if (copied <= 0) return false;
  livery_name_.assign(bytes.data(), static_cast<size_t>(copied));
  while (!livery_name_.empty() && livery_name_.back() == '\0')
    livery_name_.pop_back();
  const std::string haystack = lower_copy(livery_name_);

  const auto matches = [&](const std::string& key) {
    const std::string normalized = lower_copy(key);
    if (haystack.find(normalized) != std::string::npos) return true;
    if (key == "garuda_indonesia") return haystack.find("garuda") != std::string::npos;
    if (key == "citilink_indonesia") return haystack.find("citilink") != std::string::npos;
    if (key == "batik_air") return haystack.find("batik") != std::string::npos;
    if (key == "sriwijaya_air") return haystack.find("sriwijaya") != std::string::npos;
    if (key == "super_air_jet") return haystack.find("super air jet") != std::string::npos;
    if (key == "transnusa") return haystack.find("transnusa") != std::string::npos;
    if (key == "wings_air") return haystack.find("wings") != std::string::npos;
    return false;
  };
  for (size_t index = 0; index < airlines_.size(); ++index) {
    if (!matches(airlines_[index])) continue;
    airline_index_ = index;
    airline_auto_detected_ = true;
    status_ = "AUTO livery: " + airlines_[index];
    return true;
  }
  status_ = "Livery not matched; select airline manually";
  return false;
}

float AnnouncementEditor::terrain_y(float x, float z, float fallback) const {
  if (!probe_) return fallback;
  XPLMProbeInfo_t info{};
  info.structSize = sizeof(info);
  return XPLMProbeTerrainXYZ(probe_, x, fallback + 1000.0f, z, &info) ==
                 xplm_ProbeHitTerrain
             ? info.locationY : fallback;
}

void AnnouncementEditor::begin(double aircraft_latitude, double aircraft_longitude,
                               float aircraft_heading) {
  if (state_ == AnnouncementEditorState::Unavailable) return;
  pending_delete_id_.clear();
  detect_airline_from_livery();
  double aircraft_x{}, aircraft_y{}, aircraft_z{};
  XPLMWorldToLocal(aircraft_latitude, aircraft_longitude, 0.0,
                   &aircraft_x, &aircraft_y, &aircraft_z);
  aircraft_heading_ = aircraft_heading;
  const float angle = aircraft_heading * pi / 180.0f;
  x_ = static_cast<float>(aircraft_x) + std::sin(angle) * 20.0f;
  z_ = static_cast<float>(aircraft_z) - std::cos(angle) * 20.0f;
  ground_y_ = terrain_y(x_, z_, static_cast<float>(aircraft_y));
  altitude_offset_m_ = 4.0f;
  y_ = ground_y_ + altitude_offset_m_;
  state_ = AnnouncementEditorState::Placing;
  status_ = airline_auto_detected_
                ? "Speaker active | AUTO " + airline()
                : "Speaker active | drag X, Y or Z";
  update_world_position();
  open_overlay();
}

void AnnouncementEditor::move_forward(float metres) {
  if (state_ != AnnouncementEditorState::Placing) return;
  const float angle = aircraft_heading_ * pi / 180.0f;
  x_ += std::sin(angle) * metres;
  z_ -= std::cos(angle) * metres;
  ground_y_ = terrain_y(x_, z_, ground_y_); y_ = ground_y_ + altitude_offset_m_;
  update_world_position();
}
void AnnouncementEditor::move_side(float metres) {
  if (state_ != AnnouncementEditorState::Placing) return;
  const float angle = aircraft_heading_ * pi / 180.0f;
  x_ += std::cos(angle) * metres;
  z_ += std::sin(angle) * metres;
  ground_y_ = terrain_y(x_, z_, ground_y_); y_ = ground_y_ + altitude_offset_m_;
  update_world_position();
}
void AnnouncementEditor::move_world_x(float metres) {
  if (state_ != AnnouncementEditorState::Placing) return;
  x_ += metres; ground_y_ = terrain_y(x_, z_, ground_y_);
  y_ = ground_y_ + altitude_offset_m_; update_world_position();
}
void AnnouncementEditor::move_world_z(float metres) {
  if (state_ != AnnouncementEditorState::Placing) return;
  z_ += metres; ground_y_ = terrain_y(x_, z_, ground_y_);
  y_ = ground_y_ + altitude_offset_m_; update_world_position();
}
void AnnouncementEditor::adjust_altitude(float metres) {
  if (state_ != AnnouncementEditorState::Placing) return;
  altitude_offset_m_ = std::clamp(altitude_offset_m_ + metres, -10.0f, 100.0f);
  y_ = ground_y_ + altitude_offset_m_; update_world_position();
}
void AnnouncementEditor::update_world_position() {
  double ignored{};
  XPLMLocalToWorld(x_, y_, z_, &latitude_, &longitude_, &ignored);
  altitude_m_ = ignored;
}

void AnnouncementEditor::cancel() {
  if (state_ != AnnouncementEditorState::Placing) return;
  close_overlay();
  state_ = AnnouncementEditorState::Idle;
  status_ = "Speaker placement cancelled";
}

bool AnnouncementEditor::save() {
  if (state_ != AnnouncementEditorState::Placing || config_path_.empty()) return false;
  try {
    std::ifstream input(config_path_);
    json root = json::parse(input);
    if (!root.contains("flight_announcements") ||
        !root.at("flight_announcements").is_array())
      root["flight_announcements"] = json::array();
    size_t number = 1;
    std::string id;
    for (;;) {
      id = "flight_announcement_" +
           (number < 10 ? std::string("0") : std::string()) +
           std::to_string(number);
      bool used = false;
      for (const auto& item : root.at("flight_announcements"))
        if (item.value("id", std::string()) == id) used = true;
      if (!used) break;
      ++number;
    }
    json item = {
        {"id", id}, {"label", "Flight Announcement " + std::to_string(number)},
        {"airline", airline()}, {"flight_number", std::to_string(flight_number_)},
        {"event", event()}, {"latitude", latitude_}, {"longitude", longitude_},
        {"altitude_m", altitude_m_}, {"gain", gain_}, {"radius_m", radius_m_},
        {"autoplay", true}, {"loop", false}, {"start_delay_s", 0.0f},
        {"repeat_interval_s", 120.0f}};
    const bool arrival = event().find("landed") != std::string::npos ||
                         event().find("arrival") != std::string::npos;
    if (arrival) {
      if (!origins_.empty()) item["origin"] = origin();
    } else {
      if (!destinations_.empty()) item["destination"] = destination();
      item["gate"] = std::to_string(gate_);
    }
    root["flight_announcements"].push_back(item);
    std::ofstream output(config_path_);
    output << root.dump(2) << '\n';
    if (!output) throw std::runtime_error("Cannot write ssa.json");
    close_overlay();
    state_ = AnnouncementEditorState::Idle;
    status_ = "Saved " + id + " to ssa.json";
    return true;
  } catch (const std::exception& error) {
    status_ = error.what();
    return false;
  }
}

bool AnnouncementEditor::request_delete_nearest(double aircraft_latitude,
                                                double aircraft_longitude) {
  if (state_ != AnnouncementEditorState::Idle || config_path_.empty())
    return false;
  try {
    std::ifstream input(config_path_);
    json root = json::parse(input);
    if (!root.contains("flight_announcements") ||
        !root.at("flight_announcements").is_array() ||
        root.at("flight_announcements").empty()) {
      pending_delete_id_.clear();
      status_ = "No saved speakers to delete";
      return false;
    }

    double aircraft_x{}, aircraft_y{}, aircraft_z{};
    XPLMWorldToLocal(aircraft_latitude, aircraft_longitude, 0.0, &aircraft_x,
                     &aircraft_y, &aircraft_z);
    auto& announcements = root["flight_announcements"];
    size_t nearest_index = 0;
    std::string nearest_id;
    double nearest_distance_squared = std::numeric_limits<double>::max();
    bool found = false;
    size_t index = 0;
    for (const auto& item : announcements) {
      const double item_latitude = item.value(
          "latitude", std::numeric_limits<double>::quiet_NaN());
      const double item_longitude = item.value(
          "longitude", std::numeric_limits<double>::quiet_NaN());
      if (!std::isfinite(item_latitude) || !std::isfinite(item_longitude)) {
        ++index;
        continue;
      }
      double speaker_x{}, speaker_y{}, speaker_z{};
      XPLMWorldToLocal(item_latitude, item_longitude,
                       item.value("altitude_m", 0.0), &speaker_x, &speaker_y,
                       &speaker_z);
      const double dx = speaker_x - aircraft_x;
      const double dz = speaker_z - aircraft_z;
      const double distance_squared = dx * dx + dz * dz;
      if (distance_squared < nearest_distance_squared) {
        nearest_distance_squared = distance_squared;
        nearest_index = index;
        nearest_id = item.value(
            "id", "speaker_" + std::to_string(nearest_index + 1));
        found = true;
      }
      ++index;
    }
    if (!found) {
      pending_delete_id_.clear();
      status_ = "No speaker with valid coordinates found";
      return false;
    }

    const std::string& id = nearest_id;
    const double distance_m = std::sqrt(nearest_distance_squared);
    if (pending_delete_id_ != id) {
      pending_delete_id_ = id;
      char confirmation[180];
      std::snprintf(confirmation, sizeof(confirmation),
                    "DELETE %s at %.1f m? Press DELETE again", id.c_str(),
                    distance_m);
      status_ = confirmation;
      return false;
    }

    json retained = json::array();
    index = 0;
    for (const auto& item : announcements) {
      if (index != nearest_index) retained.push_back(item);
      ++index;
    }
    root["flight_announcements"] = retained;
    std::ofstream output(config_path_);
    output << root.dump(2) << '\n';
    if (!output) throw std::runtime_error("Cannot write ssa.json");
    pending_delete_id_.clear();
    status_ = "Deleted " + id + " from ssa.json";
    return true;
  } catch (const std::exception& error) {
    pending_delete_id_.clear();
    status_ = error.what();
    return false;
  }
}

void AnnouncementEditor::open_overlay() {
  int screen_left{}, screen_top{}, screen_right{}, screen_bottom{};
  XPLMGetScreenBoundsGlobal(&screen_left, &screen_top, &screen_right, &screen_bottom);
  const int center_x = (screen_left + screen_right) / 2;
  const int center_y = (screen_top + screen_bottom) / 2;
  constexpr int half = 150;
  if (!guide_window_) {
    XPLMCreateWindow_t guide_params{};
    guide_params.structSize = sizeof(guide_params);
    guide_params.left = screen_left;
    guide_params.top = screen_top;
    guide_params.right = screen_right;
    guide_params.bottom = screen_bottom;
    guide_params.visible = 1;
    guide_params.drawWindowFunc = draw_guide_callback;
    guide_params.refcon = this;
    guide_params.layer = xplm_WindowLayerFlightOverlay;
    guide_params.decorateAsFloatingWindow = xplm_WindowDecorationNone;
    guide_window_ = XPLMCreateWindowEx(&guide_params);
  } else {
    XPLMSetWindowGeometry(guide_window_, screen_left, screen_top, screen_right,
                          screen_bottom);
    XPLMSetWindowIsVisible(guide_window_, 1);
  }
  if (!overlay_window_) {
    XPLMCreateWindow_t params{};
    params.structSize = sizeof(params);
    params.left = center_x - half; params.top = center_y + half;
    params.right = center_x + half; params.bottom = center_y - half;
    params.visible = 1; params.drawWindowFunc = draw_callback;
    params.handleMouseClickFunc = mouse_callback;
    params.handleCursorFunc = cursor_callback; params.refcon = this;
    params.layer = xplm_WindowLayerFlightOverlay;
    params.decorateAsFloatingWindow = xplm_WindowDecorationNone;
    overlay_window_ = XPLMCreateWindowEx(&params);
  } else {
    XPLMSetWindowGeometry(overlay_window_, center_x - half, center_y + half,
                          center_x + half, center_y - half);
    XPLMSetWindowIsVisible(overlay_window_, 1);
  }
}
void AnnouncementEditor::close_overlay() {
  gizmo_drag_ = GizmoDrag::None;
  if (overlay_window_) XPLMSetWindowIsVisible(overlay_window_, 0);
  if (guide_window_) XPLMSetWindowIsVisible(guide_window_, 0);
}
void AnnouncementEditor::draw_callback(XPLMWindowID, void* refcon) {
  static_cast<AnnouncementEditor*>(refcon)->draw_overlay();
}
void AnnouncementEditor::draw_guide_callback(XPLMWindowID, void* refcon) {
  static_cast<AnnouncementEditor*>(refcon)->draw_radius_guides();
}

void AnnouncementEditor::draw_radius_guides() {
  if (state_ != AnnouncementEditorState::Placing || !guide_window_) return;
  int screen_left{}, screen_top{}, screen_right{}, screen_bottom{};
  XPLMGetScreenBoundsGlobal(&screen_left, &screen_top, &screen_right,
                            &screen_bottom);
  XPLMSetWindowGeometry(guide_window_, screen_left, screen_top, screen_right,
                        screen_bottom);
  char dot[] = ".";
  constexpr int segments = 48;
  for (int ring = 1; ring <= 4; ++ring) {
    const float ring_radius = radius_m_ * static_cast<float>(ring) / 4.0f;
    const float brightness = ring == 4 ? 1.0f : 0.60f;
    for (int segment = 0; segment < segments; ++segment) {
      const float angle = 2.0f * pi * static_cast<float>(segment) /
                          static_cast<float>(segments);
      const float guide_x = x_ + std::cos(angle) * ring_radius;
      const float guide_z = z_ + std::sin(angle) * ring_radius;
      int pixel_x{}, pixel_y{};
      if (!project_point(guide_x, ground_y_ + 0.08f, guide_z, pixel_x,
                         pixel_y))
        continue;
      draw_text(pixel_x, pixel_y, dot, 0.20f, brightness, 0.70f);
    }
  }
  int label_x{}, label_y{};
  if (project_point(x_ + radius_m_, ground_y_ + 0.08f, z_, label_x,
                    label_y)) {
    char label[64];
    std::snprintf(label, sizeof(label), "AUDIO RADIUS %.0f M", radius_m_);
    XPLMDrawTranslucentDarkBox(label_x - 8, label_y + 18, label_x + 145,
                               label_y - 8);
    draw_text(label_x, label_y, label, 0.25f, 1.0f, 0.65f);
  }
}
void AnnouncementEditor::draw_overlay() {
  if (state_ != AnnouncementEditorState::Placing || !overlay_window_) return;
  int projected_x{}, projected_y{};
  if (project_to_screen(projected_x, projected_y)) {
    constexpr int half = 150;
    XPLMSetWindowGeometry(overlay_window_, projected_x - half, projected_y + half,
                          projected_x + half, projected_y - half);
  }
  int left{}, top{}, right{}, bottom{};
  XPLMGetWindowGeometry(overlay_window_, &left, &top, &right, &bottom);
  const int cx = (left + right) / 2; const int cy = (top + bottom) / 2;
  char title[] = "SSA 3D SPEAKER";
  char icon[] = "<|  )))  SPEAKER";
  char help[] = "World axes | drag X, Y or Z";
  char range[64];
  std::snprintf(range, sizeof(range), "RADIUS %.0f M | GAIN %.1f", radius_m_, gain_);
  XPLMDrawTranslucentDarkBox(cx - 58, cy + 22, cx + 58, cy - 22);
  draw_text(left + 86, top - 22, title, 0.95f, 0.95f, 1.0f);
  draw_text(cx - 49, cy - 5, icon, 0.25f, 1.0f, 0.65f);
  float x_dx{}, x_dy{}, y_dx{}, y_dy{}, z_dx{}, z_dy{};
  if (axis_direction(GizmoDrag::MoveX, x_dx, x_dy))
    draw_dotted_axis(cx, cy, x_dx, x_dy, 1.0f, 0.25f, 0.20f, "X");
  if (axis_direction(GizmoDrag::MoveY, y_dx, y_dy))
    draw_dotted_axis(cx, cy, y_dx, y_dy, 0.25f, 1.0f, 0.40f, "Y");
  if (axis_direction(GizmoDrag::MoveZ, z_dx, z_dy))
    draw_dotted_axis(cx, cy, z_dx, z_dy, 0.25f, 0.65f, 1.0f, "Z");
  draw_text(left + 82, bottom + 32, range, 1.0f, 0.75f, 0.20f);
  draw_text(left + 61, bottom + 12, help, 0.95f, 0.95f, 1.0f);
}

bool AnnouncementEditor::project_to_screen(int& screen_x, int& screen_y) const {
  return project_point(x_, y_, z_, screen_x, screen_y);
}

bool AnnouncementEditor::project_point(float x, float y, float z,
                                       int& screen_x, int& screen_y) const {
  if (!world_matrix_ref_ || !projection_matrix_ref_) return false;
  float world_matrix[16]{}, projection_matrix[16]{};
  if (XPLMGetDatavf(world_matrix_ref_, world_matrix, 0, 16) != 16 ||
      XPLMGetDatavf(projection_matrix_ref_, projection_matrix, 0, 16) != 16) return false;
  const float world_position[4] = {x, y, z, 1.0f};
  float eye_position[4]{}, clip_position[4]{};
  multiply_matrix_vector(eye_position, world_matrix, world_position);
  multiply_matrix_vector(clip_position, projection_matrix, eye_position);
  if (clip_position[3] <= 0.001f) return false;
  const float ndc_x = clip_position[0] / clip_position[3];
  const float ndc_y = clip_position[1] / clip_position[3];
  const float ndc_z = clip_position[2] / clip_position[3];
  if (ndc_x < -1.15f || ndc_x > 1.15f || ndc_y < -1.15f ||
      ndc_y > 1.15f || ndc_z < -1.0f || ndc_z > 1.0f) return false;
  int left{}, top{}, right{}, bottom{};
  XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
  screen_x = left + static_cast<int>((ndc_x * 0.5f + 0.5f) * (right - left));
  screen_y = bottom + static_cast<int>((ndc_y * 0.5f + 0.5f) * (top - bottom));
  return true;
}

bool AnnouncementEditor::axis_direction(GizmoDrag axis, float& screen_dx,
                                        float& screen_dy) const {
  int center_x{}, center_y{}, endpoint_x{}, endpoint_y{};
  if (!project_point(x_, y_, z_, center_x, center_y)) return false;
  float endpoint_world_x = x_;
  float endpoint_world_y = y_;
  float endpoint_world_z = z_;
  if (axis == GizmoDrag::MoveX) endpoint_world_x += 2.0f;
  else if (axis == GizmoDrag::MoveY) endpoint_world_y += 2.0f;
  else if (axis == GizmoDrag::MoveZ) endpoint_world_z += 2.0f;
  else return false;
  if (!project_point(endpoint_world_x, endpoint_world_y, endpoint_world_z,
                     endpoint_x, endpoint_y)) return false;
  screen_dx = static_cast<float>(endpoint_x - center_x);
  screen_dy = static_cast<float>(endpoint_y - center_y);
  const float length = std::hypot(screen_dx, screen_dy);
  if (length < 1.0f) return false;
  screen_dx /= length;
  screen_dy /= length;
  return true;
}

int AnnouncementEditor::mouse_callback(XPLMWindowID, int x, int y,
                                       XPLMMouseStatus status, void* refcon) {
  return static_cast<AnnouncementEditor*>(refcon)->overlay_mouse(x, y, status);
}
int AnnouncementEditor::overlay_mouse(int x, int y, XPLMMouseStatus status) {
  if (state_ != AnnouncementEditorState::Placing || !overlay_window_) return 0;
  int left{}, top{}, right{}, bottom{};
  XPLMGetWindowGeometry(overlay_window_, &left, &top, &right, &bottom);
  const int cx = (left + right) / 2; const int cy = (top + bottom) / 2;
  if (status == xplm_MouseDown) {
    float best_distance = 15.0f;
    GizmoDrag best_axis = GizmoDrag::None;
    for (const GizmoDrag axis : {GizmoDrag::MoveX, GizmoDrag::MoveY,
                                 GizmoDrag::MoveZ}) {
      float axis_dx{}, axis_dy{};
      if (!axis_direction(axis, axis_dx, axis_dy)) continue;
      const float distance = distance_to_segment(
          static_cast<float>(x), static_cast<float>(y), cx + axis_dx * 25.0f,
          cy + axis_dy * 25.0f, cx + axis_dx * 108.0f,
          cy + axis_dy * 108.0f);
      if (distance < best_distance) {
        best_distance = distance;
        best_axis = axis;
      }
    }
    gizmo_drag_ = best_axis;
    if (gizmo_drag_ == GizmoDrag::None) return 0;
    drag_last_x_ = x; drag_last_y_ = y; return 1;
  }
  if (status == xplm_MouseUp) {
    const bool active = gizmo_drag_ != GizmoDrag::None;
    gizmo_drag_ = GizmoDrag::None; return active ? 1 : 0;
  }
  if (status != xplm_MouseDrag || gizmo_drag_ == GizmoDrag::None) return 0;
  const float dx = static_cast<float>(x - drag_last_x_);
  const float dy = static_cast<float>(y - drag_last_y_);
  drag_last_x_ = x; drag_last_y_ = y;
  float axis_dx{}, axis_dy{};
  if (!axis_direction(gizmo_drag_, axis_dx, axis_dy)) return 1;
  const float projected_drag = dx * axis_dx + dy * axis_dy;
  if (gizmo_drag_ == GizmoDrag::MoveX)
    move_world_x(projected_drag * 0.05f);
  else if (gizmo_drag_ == GizmoDrag::MoveY)
    adjust_altitude(projected_drag * 0.035f);
  else if (gizmo_drag_ == GizmoDrag::MoveZ)
    move_world_z(projected_drag * 0.05f);
  return 1;
}
XPLMCursorStatus AnnouncementEditor::cursor_callback(XPLMWindowID, int, int, void*) {
  return xplm_CursorArrow;
}

} // namespace ssa
