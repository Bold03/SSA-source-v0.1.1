#include "ssa/announcement_editor.hpp"
#include <XPLMGraphics.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
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
} // namespace

AnnouncementEditor::~AnnouncementEditor() { unload(); }

void AnnouncementEditor::unload() {
  close_overlay();
  if (overlay_window_) XPLMDestroyWindow(overlay_window_);
  if (probe_) XPLMDestroyProbe(probe_);
  overlay_window_ = nullptr;
  probe_ = nullptr;
  world_matrix_ref_ = nullptr;
  projection_matrix_ref_ = nullptr;
  config_path_.clear();
  airlines_.clear();
  origins_.clear();
  destinations_.clear();
  events_.clear();
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
    if (!probe_) {
      status_ = "Cannot create terrain probe";
      return false;
    }
    state_ = AnnouncementEditorState::Idle;
    status_ = "Announcement Composer ready";
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
void AnnouncementEditor::next_airline(int direction) { cycle_index(airline_index_, direction, airlines_.size()); }
void AnnouncementEditor::next_origin(int direction) { cycle_index(origin_index_, direction, origins_.size()); }
void AnnouncementEditor::next_destination(int direction) { cycle_index(destination_index_, direction, destinations_.size()); }
void AnnouncementEditor::next_event(int direction) { cycle_index(event_index_, direction, events_.size()); }
void AnnouncementEditor::adjust_flight_number(int amount) { flight_number_ = std::clamp(flight_number_ + amount, 1, 9999); }
void AnnouncementEditor::adjust_gate(int amount) { gate_ = std::clamp(gate_ + amount, 1, 999); }
void AnnouncementEditor::adjust_gain(float amount) { gain_ = std::clamp(gain_ + amount, 0.1f, 2.0f); }
void AnnouncementEditor::adjust_radius(float amount) { radius_m_ = std::clamp(radius_m_ + amount, 10.0f, 2000.0f); }

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
  status_ = "Speaker icon active: drag X, Y or Z";
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

void AnnouncementEditor::open_overlay() {
  int screen_left{}, screen_top{}, screen_right{}, screen_bottom{};
  XPLMGetScreenBoundsGlobal(&screen_left, &screen_top, &screen_right, &screen_bottom);
  const int center_x = (screen_left + screen_right) / 2;
  const int center_y = (screen_top + screen_bottom) / 2;
  constexpr int half = 130;
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
}
void AnnouncementEditor::draw_callback(XPLMWindowID, void* refcon) {
  static_cast<AnnouncementEditor*>(refcon)->draw_overlay();
}
void AnnouncementEditor::draw_overlay() {
  if (state_ != AnnouncementEditorState::Placing || !overlay_window_) return;
  int projected_x{}, projected_y{};
  if (project_to_screen(projected_x, projected_y)) {
    constexpr int half = 130;
    XPLMSetWindowGeometry(overlay_window_, projected_x - half, projected_y + half,
                          projected_x + half, projected_y - half);
  }
  int left{}, top{}, right{}, bottom{};
  XPLMGetWindowGeometry(overlay_window_, &left, &top, &right, &bottom);
  const int cx = (left + right) / 2; const int cy = (top + bottom) / 2;
  char title[] = "SSA 3D AUDIO SOURCE";
  char icon[] = "[ |>  ))) ]";
  char x_axis[] = "< X- -------- +X >";
  char y_plus[] = "[ +Y ]"; char y_minus[] = "[ -Y ]";
  char z_axis[] = "< Z- -------- +Z >";
  char help[] = "Speaker preview only | Drag a handle";
  char range[64];
  std::snprintf(range, sizeof(range), "RADIUS %.0f M | GAIN %.1f", radius_m_, gain_);
  draw_text(left + 55, top - 22, title, 0.95f, 0.95f, 1.0f);
  draw_text(cx - 45, cy + 12, icon, 0.25f, 1.0f, 0.65f);
  draw_text(cx - 70, cy - 20, x_axis, 1.0f, 0.25f, 0.20f);
  draw_text(cx - 18, cy + 82, y_plus, 0.25f, 1.0f, 0.40f);
  draw_text(cx - 18, cy + 52, y_minus, 0.25f, 1.0f, 0.40f);
  draw_text(cx - 70, cy - 60, z_axis, 0.25f, 0.65f, 1.0f);
  draw_text(left + 65, bottom + 32, range, 1.0f, 0.75f, 0.20f);
  draw_text(left + 20, bottom + 12, help, 0.95f, 0.95f, 1.0f);
}

bool AnnouncementEditor::project_to_screen(int& screen_x, int& screen_y) const {
  if (!world_matrix_ref_ || !projection_matrix_ref_) return false;
  float world_matrix[16]{}, projection_matrix[16]{};
  if (XPLMGetDatavf(world_matrix_ref_, world_matrix, 0, 16) != 16 ||
      XPLMGetDatavf(projection_matrix_ref_, projection_matrix, 0, 16) != 16) return false;
  const float world_position[4] = {x_, y_, z_, 1.0f};
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
    if (x >= cx - 95 && x <= cx + 95 && y >= cy - 38 && y <= cy + 5)
      gizmo_drag_ = GizmoDrag::MoveX;
    else if (x >= cx - 40 && x <= cx + 40 && y >= cy + 42 && y <= cy + 112)
      gizmo_drag_ = GizmoDrag::MoveY;
    else if (x >= cx - 95 && x <= cx + 95 && y >= cy - 82 && y <= cy - 42)
      gizmo_drag_ = GizmoDrag::MoveZ;
    else return 0;
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
  if (gizmo_drag_ == GizmoDrag::MoveX) move_world_x(dx * 0.05f);
  else if (gizmo_drag_ == GizmoDrag::MoveY) adjust_altitude(dy * 0.02f);
  else if (gizmo_drag_ == GizmoDrag::MoveZ) move_world_z(dx * 0.05f);
  return 1;
}
XPLMCursorStatus AnnouncementEditor::cursor_callback(XPLMWindowID, int, int, void*) {
  return xplm_CursorArrow;
}

} // namespace ssa
