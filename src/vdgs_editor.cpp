#include "ssa/vdgs_editor.hpp"
#include <XPLMGraphics.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
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
// XPLMCreateInstance in the X-Plane SDK takes const char** (the pointer array
// itself is mutable), so this must not be a constexpr/const pointer array.
const char* instance_datarefs[] = {
    "boldstudio31/ssa/vdgs/active", "boldstudio31/ssa/vdgs/left",
    "boldstudio31/ssa/vdgs/right", "boldstudio31/ssa/vdgs/center",
    "boldstudio31/ssa/vdgs/slow", "boldstudio31/ssa/vdgs/stop",
    "boldstudio31/ssa/vdgs/lateral", "boldstudio31/ssa/vdgs/distance_ratio",
    nullptr};
}

VdgsEditor::~VdgsEditor() { unload(); }

float VdgsEditor::normalize_heading(float heading) {
  heading = std::fmod(heading, 360.0f);
  return heading < 0.0f ? heading + 360.0f : heading;
}

void VdgsEditor::unload() {
  close_gizmo();
  if (gizmo_window_) XPLMDestroyWindow(gizmo_window_);
  gizmo_window_ = nullptr;
  if (preview_instance_) XPLMDestroyInstance(preview_instance_);
  if (preview_object_) XPLMUnloadObject(preview_object_);
  if (probe_) XPLMDestroyProbe(probe_);
  preview_instance_ = nullptr;
  preview_object_ = nullptr;
  probe_ = nullptr;
  world_matrix_ref_ = nullptr;
  projection_matrix_ref_ = nullptr;
  for (auto& placement : placements_) {
    if (placement.instance) XPLMDestroyInstance(placement.instance);
    if (placement.object) XPLMUnloadObject(placement.object);
  }
  placements_.clear();
  state_ = VdgsEditorState::Unavailable;
}

bool VdgsEditor::load(const std::string& xplane_root) {
  unload();
  xplane_root_ = xplane_root;
  const fs::path custom = fs::path(xplane_root) / "Custom Scenery";
  try {
    if (!fs::exists(custom)) {
      status_ = "Custom Scenery folder not found";
      return false;
    }
    for (const auto& entry : fs::directory_iterator(custom)) {
      const fs::path config = entry.path() / "ssa.json";
      if (!entry.is_directory() || !fs::exists(config)) continue;
      load_saved(config.string());
      if (!absolute_object_path_.empty()) continue;

      std::ifstream input(config);
      const json root = json::parse(input);
      if (root.contains("vdgs_models") && root.at("vdgs_models").is_array() &&
          !root.at("vdgs_models").empty()) {
        const auto& model = root.at("vdgs_models").front();
        if (model.contains("object")) {
          const std::string relative = model.at("object").get<std::string>();
          const fs::path candidate = entry.path() / relative;
          if (fs::exists(candidate)) {
            scenery_directory_ = entry.path().string();
            config_path_ = config.string();
            relative_object_path_ = relative;
            absolute_object_path_ = candidate.string();
          }
        }
      }
      if (!absolute_object_path_.empty()) continue;
      for (const char* relative : {"object/VDGS.obj", "objects/VDGS.obj", "VDGS.obj"}) {
        const fs::path candidate = entry.path() / relative;
        if (!fs::exists(candidate)) continue;
        scenery_directory_ = entry.path().string();
        config_path_ = config.string();
        relative_object_path_ = relative;
        absolute_object_path_ = candidate.string();
        break;
      }
    }
    if (absolute_object_path_.empty()) {
      status_ = "Put VDGS.obj in scenery/object or configure vdgs_models";
      return false;
    }
    preview_object_ = XPLMLoadObject(absolute_object_path_.c_str());
    probe_ = XPLMCreateProbe(xplm_ProbeY);
    world_matrix_ref_ = XPLMFindDataRef("sim/graphics/view/world_matrix");
    projection_matrix_ref_ =
        XPLMFindDataRef("sim/graphics/view/projection_matrix_3d");
    if (!preview_object_ || !probe_) {
      status_ = "Cannot load VDGS OBJ";
      unload();
      return false;
    }
    preview_instance_ = XPLMCreateInstance(preview_object_, instance_datarefs);
    if (!preview_instance_) {
      status_ = "Cannot create VDGS preview";
      unload();
      return false;
    }
    XPLMDrawInfo_t hidden{};
    hidden.structSize = sizeof(hidden);
    hidden.y = -10000.0f;
    const float hidden_data[] = {0.0f, 0.0f, 0.0f, 0.0f,
                                 0.0f, 0.0f, 0.5f, 1.0f};
    XPLMInstanceSetPosition(preview_instance_, &hidden, hidden_data);
    state_ = VdgsEditorState::Idle;
    status_ = "VDGS placement ready";
    return true;
  } catch (const std::exception& e) {
    const std::string message = e.what();
    unload();
    status_ = message;
    return false;
  }
}

bool VdgsEditor::load_saved(const std::string& config_path) {
  std::ifstream input(config_path);
  const json root = json::parse(input);
  if (!root.contains("objects") || !root.at("objects").is_array()) return true;
  for (const auto& item : root.at("objects")) {
    if (item.value("type", std::string()) != "parking_display" ||
        !item.contains("object")) continue;
    VdgsPlacement placement;
    placement.id = item.at("id").get<std::string>();
    placement.object_path =
        (fs::path(config_path).parent_path() /
         item.at("object").get<std::string>()).string();
    placement.latitude = item.at("latitude").get<double>();
    placement.longitude = item.at("longitude").get<double>();
    placement.altitude_m = item.value("altitude_m", 0.0);
    placement.heading = item.value(
        "heading", item.contains("vdgs")
                       ? item.at("vdgs").value("object_heading_deg", 0.0f)
                       : 0.0f);
    placement.data[6] = 0.5f;
    placement.data[7] = 1.0f;
    if (create_saved_instance(placement)) placements_.push_back(std::move(placement));
  }
  return true;
}

bool VdgsEditor::create_saved_instance(VdgsPlacement& placement) {
  placement.object = XPLMLoadObject(placement.object_path.c_str());
  if (!placement.object) return false;
  placement.instance = XPLMCreateInstance(placement.object, instance_datarefs);
  if (!placement.instance) {
    XPLMUnloadObject(placement.object);
    placement.object = nullptr;
    return false;
  }
  return true;
}

float VdgsEditor::terrain_y(float x, float z, float fallback) const {
  if (!probe_) return fallback;
  XPLMProbeInfo_t info{};
  info.structSize = sizeof(info);
  return XPLMProbeTerrainXYZ(probe_, x, fallback + 1000.0f, z, &info) ==
                 xplm_ProbeHitTerrain
             ? info.locationY
             : fallback;
}

void VdgsEditor::begin(double aircraft_latitude, double aircraft_longitude,
                       float aircraft_heading) {
  if (state_ == VdgsEditorState::Unavailable || !preview_instance_) return;
  double aircraft_x{}, aircraft_y{}, aircraft_z{};
  XPLMWorldToLocal(aircraft_latitude, aircraft_longitude, 0.0,
                   &aircraft_x, &aircraft_y, &aircraft_z);
  const float angle = aircraft_heading * pi / 180.0f;
  x_ = static_cast<float>(aircraft_x) + std::sin(angle) * 20.0f;
  z_ = static_cast<float>(aircraft_z) - std::cos(angle) * 20.0f;
  ground_y_ = terrain_y(x_, z_, static_cast<float>(aircraft_y));
  altitude_offset_m_ = 0.0f;
  y_ = ground_y_;
  heading_ = normalize_heading(aircraft_heading + 180.0f);
  double ignored_altitude{};
  XPLMLocalToWorld(x_, y_, z_, &latitude_, &longitude_, &ignored_altitude);
  altitude_m_ = ignored_altitude;
  state_ = VdgsEditorState::Placing;
  status_ = "3D gizmo active: drag X, Y, Z or ROTATE";
  open_gizmo();
  show_preview();
}

void VdgsEditor::open_gizmo() {
  int screen_left{}, screen_top{}, screen_right{}, screen_bottom{};
  XPLMGetScreenBoundsGlobal(&screen_left, &screen_top, &screen_right,
                            &screen_bottom);
  const int center_x = (screen_left + screen_right) / 2;
  const int center_y = (screen_top + screen_bottom) / 2;
  constexpr int half = 130;
  if (!gizmo_window_) {
    XPLMCreateWindow_t params{};
    params.structSize = sizeof(params);
    params.left = center_x - half;
    params.top = center_y + half;
    params.right = center_x + half;
    params.bottom = center_y - half;
    params.visible = 1;
    params.drawWindowFunc = draw_gizmo_callback;
    params.handleMouseClickFunc = gizmo_mouse_callback;
    params.handleCursorFunc = gizmo_cursor_callback;
    params.refcon = this;
    params.layer = xplm_WindowLayerFlightOverlay;
    params.decorateAsFloatingWindow = xplm_WindowDecorationNone;
    gizmo_window_ = XPLMCreateWindowEx(&params);
  } else {
    XPLMSetWindowGeometry(gizmo_window_, center_x - half, center_y + half,
                          center_x + half, center_y - half);
    XPLMSetWindowIsVisible(gizmo_window_, 1);
  }
}

void VdgsEditor::close_gizmo() {
  gizmo_drag_ = GizmoDrag::None;
  if (gizmo_window_) XPLMSetWindowIsVisible(gizmo_window_, 0);
}

void VdgsEditor::draw_gizmo_callback(XPLMWindowID, void* refcon) {
  static_cast<VdgsEditor*>(refcon)->draw_gizmo();
}

void VdgsEditor::draw_gizmo() {
  if (state_ != VdgsEditorState::Placing || !gizmo_window_) return;
  int projected_x{}, projected_y{};
  if (project_object_to_screen(projected_x, projected_y)) {
    constexpr int half = 130;
    XPLMSetWindowGeometry(gizmo_window_, projected_x - half,
                          projected_y + half, projected_x + half,
                          projected_y - half);
  }
  int left{}, top{}, right{}, bottom{};
  XPLMGetWindowGeometry(gizmo_window_, &left, &top, &right, &bottom);
  const int cx = (left + right) / 2;
  const int cy = (top + bottom) / 2;
  float red[] = {1.0f, 0.25f, 0.20f};
  float green[] = {0.25f, 1.0f, 0.40f};
  float blue[] = {0.25f, 0.65f, 1.0f};
  float yellow[] = {1.0f, 0.75f, 0.20f};
  float white[] = {0.95f, 0.95f, 1.0f};
  char title[] = "SSA 3D OBJECT GIZMO";
  char x_axis[] = "< X- -------- +X >";
  char y_plus[] = "[ +Y ]";
  char y_minus[] = "[ -Y ]";
  char z_axis[] = "< Z- -------- +Z >";
  char rotate[] = "(  ROTATE  )";
  char help[] = "Drag a handle | Camera remains 3D";
  XPLMDrawString(white, left + 55, top - 22, title, nullptr,
                 xplmFont_Proportional);
  XPLMDrawString(red, cx - 70, cy, x_axis, nullptr, xplmFont_Proportional);
  XPLMDrawString(green, cx - 18, cy + 72, y_plus, nullptr,
                 xplmFont_Proportional);
  XPLMDrawString(green, cx - 18, cy + 40, y_minus, nullptr,
                 xplmFont_Proportional);
  XPLMDrawString(blue, cx - 70, cy - 48, z_axis, nullptr,
                 xplmFont_Proportional);
  XPLMDrawString(yellow, cx - 45, cy - 92, rotate, nullptr,
                 xplmFont_Proportional);
  XPLMDrawString(white, left + 25, bottom + 12, help, nullptr,
                 xplmFont_Proportional);
}

bool VdgsEditor::project_object_to_screen(int& screen_x, int& screen_y) const {
  if (!world_matrix_ref_ || !projection_matrix_ref_) return false;
  float world_matrix[16]{};
  float projection_matrix[16]{};
  if (XPLMGetDatavf(world_matrix_ref_, world_matrix, 0, 16) != 16 ||
      XPLMGetDatavf(projection_matrix_ref_, projection_matrix, 0, 16) != 16)
    return false;
  const float world_position[4] = {x_, y_ + 2.0f, z_, 1.0f};
  float eye_position[4]{};
  float clip_position[4]{};
  multiply_matrix_vector(eye_position, world_matrix, world_position);
  multiply_matrix_vector(clip_position, projection_matrix, eye_position);
  if (clip_position[3] <= 0.001f) return false;
  const float ndc_x = clip_position[0] / clip_position[3];
  const float ndc_y = clip_position[1] / clip_position[3];
  const float ndc_z = clip_position[2] / clip_position[3];
  if (ndc_x < -1.15f || ndc_x > 1.15f || ndc_y < -1.15f ||
      ndc_y > 1.15f || ndc_z < -1.0f || ndc_z > 1.0f)
    return false;
  int left{}, top{}, right{}, bottom{};
  XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
  screen_x = left + static_cast<int>((ndc_x * 0.5f + 0.5f) * (right - left));
  screen_y = bottom + static_cast<int>((ndc_y * 0.5f + 0.5f) * (top - bottom));
  return true;
}

int VdgsEditor::gizmo_mouse_callback(XPLMWindowID, int x, int y,
                                     XPLMMouseStatus status, void* refcon) {
  return static_cast<VdgsEditor*>(refcon)->gizmo_mouse(x, y, status);
}

int VdgsEditor::gizmo_mouse(int x, int y, XPLMMouseStatus status) {
  if (state_ != VdgsEditorState::Placing || !gizmo_window_) return 0;
  int left{}, top{}, right{}, bottom{};
  XPLMGetWindowGeometry(gizmo_window_, &left, &top, &right, &bottom);
  const int cx = (left + right) / 2;
  const int cy = (top + bottom) / 2;
  if (status == xplm_MouseDown) {
    if (x >= cx - 95 && x <= cx + 95 && y >= cy - 18 && y <= cy + 24)
      gizmo_drag_ = GizmoDrag::MoveX;
    else if (x >= cx - 40 && x <= cx + 40 && y >= cy + 30 && y <= cy + 105)
      gizmo_drag_ = GizmoDrag::MoveY;
    else if (x >= cx - 95 && x <= cx + 95 && y >= cy - 68 && y <= cy - 25)
      gizmo_drag_ = GizmoDrag::MoveZ;
    else if (x >= cx - 75 && x <= cx + 75 && y >= cy - 115 && y <= cy - 70)
      gizmo_drag_ = GizmoDrag::Rotate;
    else
      return 0;
    drag_last_x_ = x;
    drag_last_y_ = y;
    return 1;
  }
  if (status == xplm_MouseUp) {
    const bool was_dragging = gizmo_drag_ != GizmoDrag::None;
    gizmo_drag_ = GizmoDrag::None;
    return was_dragging ? 1 : 0;
  }
  if (status != xplm_MouseDrag || gizmo_drag_ == GizmoDrag::None) return 0;
  const float dx = static_cast<float>(x - drag_last_x_);
  const float dy = static_cast<float>(y - drag_last_y_);
  drag_last_x_ = x;
  drag_last_y_ = y;
  if (gizmo_drag_ == GizmoDrag::MoveX) move_world_x(dx * 0.05f);
  else if (gizmo_drag_ == GizmoDrag::MoveY) adjust_altitude(dy * 0.02f);
  else if (gizmo_drag_ == GizmoDrag::MoveZ) move_world_z(dx * 0.05f);
  else if (gizmo_drag_ == GizmoDrag::Rotate) turn(-dx * 0.5f);
  return 1;
}

XPLMCursorStatus VdgsEditor::gizmo_cursor_callback(XPLMWindowID, int, int,
                                                   void*) {
  return xplm_CursorArrow;
}

void VdgsEditor::move_forward(float metres) {
  if (state_ != VdgsEditorState::Placing) return;
  const float angle = heading_ * pi / 180.0f;
  x_ += std::sin(angle) * metres;
  z_ -= std::cos(angle) * metres;
  ground_y_ = terrain_y(x_, z_, ground_y_);
  y_ = ground_y_ + altitude_offset_m_;
  show_preview();
}

void VdgsEditor::move_side(float metres) {
  if (state_ != VdgsEditorState::Placing) return;
  const float angle = heading_ * pi / 180.0f;
  x_ += std::cos(angle) * metres;
  z_ += std::sin(angle) * metres;
  ground_y_ = terrain_y(x_, z_, ground_y_);
  y_ = ground_y_ + altitude_offset_m_;
  show_preview();
}

void VdgsEditor::move_world_x(float metres) {
  if (state_ != VdgsEditorState::Placing) return;
  x_ += metres;
  ground_y_ = terrain_y(x_, z_, ground_y_);
  y_ = ground_y_ + altitude_offset_m_;
  show_preview();
}

void VdgsEditor::move_world_z(float metres) {
  if (state_ != VdgsEditorState::Placing) return;
  z_ += metres;
  ground_y_ = terrain_y(x_, z_, ground_y_);
  y_ = ground_y_ + altitude_offset_m_;
  show_preview();
}

void VdgsEditor::turn(float degrees) {
  if (state_ != VdgsEditorState::Placing) return;
  heading_ = normalize_heading(heading_ + degrees);
  show_preview();
}

void VdgsEditor::adjust_altitude(float metres) {
  if (state_ != VdgsEditorState::Placing) return;
  altitude_offset_m_ = std::clamp(altitude_offset_m_ + metres, -10.0f, 50.0f);
  y_ = ground_y_ + altitude_offset_m_;
  show_preview();
}

void VdgsEditor::show_preview() {
  if (!preview_instance_ || state_ != VdgsEditorState::Placing) return;
  XPLMDrawInfo_t draw{};
  draw.structSize = sizeof(draw);
  draw.x = x_;
  draw.y = y_;
  draw.z = z_;
  draw.heading = heading_;
  const float data[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.5f, 0.5f};
  XPLMInstanceSetPosition(preview_instance_, &draw, data);
  double ignored{};
  XPLMLocalToWorld(x_, y_, z_, &latitude_, &longitude_, &ignored);
  altitude_m_ = ignored;
}

void VdgsEditor::cancel() {
  if (state_ != VdgsEditorState::Placing) return;
  XPLMDrawInfo_t hidden{};
  hidden.structSize = sizeof(hidden);
  hidden.y = -10000.0f;
  const float data[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f};
  XPLMInstanceSetPosition(preview_instance_, &hidden, data);
  close_gizmo();
  state_ = VdgsEditorState::Idle;
  status_ = "VDGS placement cancelled";
}

bool VdgsEditor::save() {
  if (state_ != VdgsEditorState::Placing || config_path_.empty()) return false;
  try {
    std::ifstream input(config_path_);
    json root = json::parse(input);
    if (!root.contains("objects") || !root.at("objects").is_array())
      root["objects"] = json::array();
    size_t number = 1;
    std::string id;
    for (;;) {
      id = "vdgs_" + (number < 10 ? std::string("0") : std::string()) +
           std::to_string(number);
      bool used = false;
      for (const auto& item : root.at("objects"))
        if (item.value("id", std::string()) == id) used = true;
      if (!used) break;
      ++number;
    }
    json item = {
        {"id", id}, {"label", "VDGS " + std::to_string(number)},
        {"type", "parking_display"}, {"object", relative_object_path_},
        {"latitude", latitude_}, {"longitude", longitude_},
        {"altitude_m", altitude_m_}, {"heading", heading_}, {"radius_m", 90.0f},
        {"dataref", "boldstudio31/ssa/animation/vdgs/" + id + "/state"}};
    item["vdgs"] = {{"object_heading_deg", heading_}, {"stop_distance_m", 18.0f},
                     {"use_aircraft_length", true}, {"nose_clearance_m", 2.5f},
                     {"acquisition_distance_m", 80.0f},
                     {"corridor_half_width_m", 8.0f}, {"slow_distance_m", 12.0f},
                     {"stop_tolerance_m", 0.5f}, {"lateral_full_scale_m", 3.0f},
                     {"lateral_deadband_m", 0.15f},
                     {"lateral_stop_tolerance_m", 0.35f},
                     {"lateral_multiplier", -1.0f}};
    root["objects"].push_back(item);
    std::ofstream output(config_path_);
    output << root.dump(2) << '\n';
    if (!output) throw std::runtime_error("Cannot write ssa.json");

    VdgsPlacement placement;
    placement.id = id;
    placement.object_path = absolute_object_path_;
    placement.latitude = latitude_;
    placement.longitude = longitude_;
    placement.altitude_m = altitude_m_;
    placement.heading = heading_;
    placement.data[6] = 0.5f;
    placement.data[7] = 1.0f;
    if (create_saved_instance(placement)) placements_.push_back(std::move(placement));
    cancel();
    status_ = "Saved " + id + " to ssa.json";
    return true;
  } catch (const std::exception& e) {
    status_ = e.what();
    return false;
  }
}

void VdgsEditor::clear_guidance() {
  for (auto& placement : placements_) {
    placement.data.fill(0.0f);
    placement.data[6] = 0.5f;
    placement.data[7] = 1.0f;
  }
}

void VdgsEditor::set_guidance(const std::string& id,
                              const std::array<float, 8>& data) {
  const auto found = std::find_if(placements_.begin(), placements_.end(),
                                  [&](const auto& placement) { return placement.id == id; });
  if (found != placements_.end()) found->data = data;
}

void VdgsEditor::update() {
  for (auto& placement : placements_) {
    if (!placement.instance) continue;
    double x{}, y{}, z{};
    XPLMWorldToLocal(placement.latitude, placement.longitude, placement.altitude_m,
                     &x, &y, &z);
    XPLMDrawInfo_t draw{};
    draw.structSize = sizeof(draw);
    draw.x = static_cast<float>(x);
    draw.y = static_cast<float>(y);
    draw.z = static_cast<float>(z);
    draw.heading = placement.heading;
    XPLMInstanceSetPosition(placement.instance, &draw, placement.data.data());
  }
  if (state_ == VdgsEditorState::Placing) show_preview();
}

} // namespace ssa
