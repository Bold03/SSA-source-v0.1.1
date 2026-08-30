#include "ssa/datarefs.hpp"
#include "ssa/scenery.hpp"
#include "ssa/tablet.hpp"
#include "ssa/route_editor.hpp"
#include "ssa/vdgs_editor.hpp"
#include "ssa/announcement.hpp"
#include "ssa/announcement_editor.hpp"
#include <XPLMMenus.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMGraphics.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <memory>
#include <string>

namespace {
ssa::DataRefRegistry refs;
std::unique_ptr<ssa::SceneryManager> scenery;
std::unique_ptr<ssa::Tablet> tablet;
std::unique_ptr<ssa::RouteEditor> route_editor;
std::unique_ptr<ssa::VdgsEditor> vdgs_editor;
std::unique_ptr<ssa::AnnouncementManager> announcements;
std::unique_ptr<ssa::AnnouncementEditor> announcement_editor;
ssa::FloatDataRef* automatic_ref{};
ssa::FloatDataRef* vehicle_spin_ref{};
ssa::FloatDataRef* vehicle_steering_ref{};
ssa::FloatDataRef* vdgs_active_ref{};
ssa::FloatDataRef* vdgs_left_ref{};
ssa::FloatDataRef* vdgs_right_ref{};
ssa::FloatDataRef* vdgs_center_ref{};
ssa::FloatDataRef* vdgs_slow_ref{};
ssa::FloatDataRef* vdgs_stop_ref{};
ssa::FloatDataRef* vdgs_lateral_ref{};
ssa::FloatDataRef* vdgs_distance_ref{};
XPLMDataRef lat_ref{}, lon_ref{}, heading_ref{}, icao_ref{}, door_open_ref{}, prop_ref{},
    onground_ref{}, groundspeed_ref{}, aircraft_length_ref{};
XPLMCommandRef tablet_command{};
XPLMCommandRef reload_command{};
XPLMCommandRef hangar_toggle_command{};
XPLMCommandRef hangar_open_command{};
XPLMCommandRef hangar_close_command{};
XPLMCommandRef jetway_toggle_command{};
XPLMCommandRef jetway_connect_command{};
XPLMCommandRef jetway_disconnect_command{};
XPLMCommandRef developer_mode_command{};
XPLMMenuID menu{};
bool realops_detected{};
bool vehicle_spin_test{};
float compatibility_timer{};
int action_toggle{}, action_open{1}, action_close{2};
std::string armed_vdgs_id;

void log(const std::string& message);

void select_vdgs(ssa::ServiceObject* display) {
  if (!display || display->type != ssa::ServiceType::ParkingDisplay ||
      armed_vdgs_id == display->id) {
    armed_vdgs_id.clear();
    log("VDGS selection: AUTO CORRIDOR");
    return;
  }
  armed_vdgs_id = display->id;
  log("VDGS armed: " + display->label);
}

void clear_vdgs_datarefs() {
  if (vdgs_editor) vdgs_editor->clear_guidance();
  if (vdgs_active_ref) vdgs_active_ref->set(0.0f);
  if (vdgs_left_ref) vdgs_left_ref->set(0.0f);
  if (vdgs_right_ref) vdgs_right_ref->set(0.0f);
  if (vdgs_center_ref) vdgs_center_ref->set(0.0f);
  if (vdgs_slow_ref) vdgs_slow_ref->set(0.0f);
  if (vdgs_stop_ref) vdgs_stop_ref->set(0.0f);
  if (vdgs_lateral_ref) vdgs_lateral_ref->set(0.5f);
  if (vdgs_distance_ref) vdgs_distance_ref->set(1.0f);
}

void update_vdgs(double aircraft_lat, double aircraft_lon) {
  clear_vdgs_datarefs();
  if (!scenery) return;
  for (auto& object : scenery->objects()) {
    if (object.type != ssa::ServiceType::ParkingDisplay) continue;
    object.vdgs_selected = false;
    object.vdgs_armed = !armed_vdgs_id.empty() && object.id == armed_vdgs_id;
    object.vdgs_state = ssa::VdgsState::Idle;
  }

  double aircraft_x{}, aircraft_y{}, aircraft_z{};
  XPLMWorldToLocal(aircraft_lat, aircraft_lon, 0.0,
                   &aircraft_x, &aircraft_y, &aircraft_z);
  constexpr double radians = 3.14159265358979323846 / 180.0;
  auto candidates = scenery->nearby(ssa::ServiceType::ParkingDisplay,
                                    aircraft_lat, aircraft_lon, 250.0);
  if (!armed_vdgs_id.empty()) {
    candidates.clear();
    const auto found = std::find_if(
        scenery->objects().begin(), scenery->objects().end(), [&](const auto& object) {
          return object.type == ssa::ServiceType::ParkingDisplay &&
                 object.id == armed_vdgs_id;
        });
    if (found != scenery->objects().end()) candidates.push_back(&*found);
    else armed_vdgs_id.clear();
  }

  ssa::ServiceObject* display = nullptr;
  float lateral{};
  float forward{};
  for (auto* candidate : candidates) {
    if (!candidate->vdgs.enabled) continue;
    double display_x{}, display_y{}, display_z{};
    XPLMWorldToLocal(candidate->latitude, candidate->longitude, 0.0,
                     &display_x, &display_y, &display_z);
    const double angle = static_cast<double>(candidate->vdgs.object_heading_deg) * radians;
    const double east = aircraft_x - display_x;
    const double north = -(aircraft_z - display_z);
    const float candidate_lateral =
        static_cast<float>(east * std::cos(angle) - north * std::sin(angle));
    const float candidate_forward =
        static_cast<float>(east * std::sin(angle) + north * std::cos(angle));
    const bool inside_corridor =
        candidate_forward > 0.0f &&
        candidate_forward <= candidate->vdgs.acquisition_distance_m &&
        std::abs(candidate_lateral) <= candidate->vdgs.corridor_half_width_m;
    if (!inside_corridor) continue;
    display = candidate;
    lateral = candidate_lateral;
    forward = candidate_forward;
    break;
  }
  if (!display) {
    // A manually selected gate shows a centred standby marker so pilots can
    // identify the armed stand before entering its approach corridor. This is
    // applied per instance; every other saved VDGS remains dark.
    if (!armed_vdgs_id.empty() && vdgs_editor)
      vdgs_editor->set_guidance(armed_vdgs_id,
                                {1.0f, 0.0f, 0.0f, 1.0f,
                                 0.0f, 0.0f, 0.5f, 1.0f});
    return;
  }

  float effective_stop_m = display->vdgs.stop_distance_m;
  if (display->vdgs.use_aircraft_length && aircraft_length_ref) {
    const float aircraft_length_m = XPLMGetDataf(aircraft_length_ref);
    // X-Plane publishes the aircraft length in metres. Half its length is a
    // robust initial nose offset for arbitrary aircraft; the configurable
    // clearance keeps the nose/cockpit safely in front of the VDGS panel.
    if (aircraft_length_m >= 8.0f && aircraft_length_m <= 100.0f)
      effective_stop_m = std::max(effective_stop_m,
                                  aircraft_length_m * 0.5f +
                                      display->vdgs.nose_clearance_m);
  }
  const float distance_error = forward - effective_stop_m;
  display->vdgs_lateral_error_m = lateral;
  display->vdgs_distance_error_m = distance_error;
  display->vdgs_effective_stop_m = effective_stop_m;

  display->vdgs_selected = true;
  const float distance_span = std::max(1.0f, display->vdgs.acquisition_distance_m -
                                               effective_stop_m);
  const float distance_ratio = std::clamp(std::max(0.0f, distance_error) / distance_span,
                                          0.0f, 1.0f);
  const float lateral_ratio = std::clamp(
      0.5f + display->vdgs.lateral_multiplier *
                 (lateral / display->vdgs.lateral_full_scale_m) * 0.5f,
      0.0f, 1.0f);
  const bool too_right = lateral > display->vdgs.lateral_deadband_m;
  const bool too_left = lateral < -display->vdgs.lateral_deadband_m;
  const bool stopped = std::abs(distance_error) <= display->vdgs.stop_tolerance_m &&
                       std::abs(lateral) <= display->vdgs.lateral_stop_tolerance_m;
  const bool overshot = distance_error < -display->vdgs.stop_tolerance_m;
  const bool slow = !stopped && !overshot && distance_error <= display->vdgs.slow_distance_m;

  vdgs_active_ref->set(1.0f);
  vdgs_center_ref->set(1.0f);
  vdgs_lateral_ref->set(lateral_ratio);
  vdgs_distance_ref->set(distance_ratio);
  vdgs_left_ref->set(!stopped && too_right ? 1.0f : 0.0f);
  vdgs_right_ref->set(!stopped && too_left ? 1.0f : 0.0f);
  vdgs_slow_ref->set(slow ? 1.0f : 0.0f);
  vdgs_stop_ref->set((stopped || overshot) ? 1.0f : 0.0f);
  if (vdgs_editor) {
    vdgs_editor->set_guidance(
        display->id,
        {1.0f, !stopped && too_right ? 1.0f : 0.0f,
         !stopped && too_left ? 1.0f : 0.0f, 1.0f,
         slow ? 1.0f : 0.0f, (stopped || overshot) ? 1.0f : 0.0f,
         lateral_ratio, distance_ratio});
  }

  if (overshot) display->vdgs_state = ssa::VdgsState::Overshoot;
  else if (stopped) display->vdgs_state = ssa::VdgsState::Stop;
  else if (slow) display->vdgs_state = ssa::VdgsState::Slow;
  else if (too_left || too_right) display->vdgs_state = ssa::VdgsState::Guiding;
  else display->vdgs_state = ssa::VdgsState::Acquired;
}

struct DoorProfile {
  float forward_m;
  float right_m;
  float sill_height_m;
};

struct Vec3 {
  float x{};
  float y{};
  float z{};
};

struct JetwaySolution {
  std::array<float, 4> ratios{}; // rotunda, extension, height, cabin yaw
  float position_error_m{1000000.0f};
};

void log(const std::string& message) { XPLMDebugString(("[SSA] " + message + "\n").c_str()); }

void toggle_auto() {
  automatic_ref->set(automatic_ref->value() > 0.5f ? 0.0f : 1.0f);
  tablet->set_auto(automatic_ref->value() > 0.5f);
}

void toggle_vehicle_spin_test() { vehicle_spin_test = !vehicle_spin_test; }

void set_vehicle_steering_test(float value) {
  if (vehicle_steering_ref) vehicle_steering_ref->set(value);
}

std::string aircraft_icao() {
  char buffer[16]{};
  if (icao_ref) XPLMGetDatab(icao_ref, buffer, 0, sizeof(buffer) - 1);
  return buffer;
}

DoorProfile door_profile(const std::string& icao) {
  // Offsets are measured from the aircraft reference point: forward, right,
  // and passenger-door sill height above the ground. B738 is calibrated first.
  // SAM-style aircraft profiles store L1 separately from the door-state
  // dataref because X-Plane does not publish a universal door XYZ dataref.
  if (icao == "B738") return {12.8f, -1.85f, 2.75f};
  if (icao == "B737") return {11.5f, -1.85f, 2.70f};
  if (icao == "B739") return {13.4f, -1.85f, 2.75f};
  if (icao == "A319") return {11.2f, -1.95f, 2.85f};
  if (icao == "A320") return {12.0f, -1.95f, 2.85f};
  if (icao == "A321") return {14.5f, -1.95f, 2.90f};
  return {11.5f, -1.8f, 2.8f};
}

Vec3 rotate_y(Vec3 value, float degrees) {
  constexpr float radians = 3.14159265358979323846f / 180.0f;
  const float angle = degrees * radians;
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {cosine * value.x + sine * value.z, value.y,
          -sine * value.x + cosine * value.z};
}

Vec3 rotate_z(Vec3 value, float degrees) {
  constexpr float radians = 3.14159265358979323846f / 180.0f;
  const float angle = degrees * radians;
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {cosine * value.x - sine * value.y,
          sine * value.x + cosine * value.y, value.z};
}

Vec3 head_local(const ssa::JetwayKinematics& k, const std::array<float, 4>& ratios) {
  Vec3 point{k.head_x_m, k.head_y_m, k.head_z_m};
  point = rotate_y(point, k.cabin_degrees * ratios[3]);
  point.x += k.tunnel_parked_x_m + k.extension_x_m * ratios[1];
  point = rotate_z(point, k.height_degrees * ratios[2]);
  point.x += k.height_pivot_x_m;
  point.y += k.height_pivot_y_m;
  point.z += k.height_pivot_z_m;
  point = rotate_y(point, k.rotunda_degrees * ratios[0]);
  point.y += k.root_height_m;
  return point;
}

Vec3 door_local(const ssa::ServiceObject& jetway, float& aircraft_heading) {
  const double aircraft_lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
  const double aircraft_lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
  aircraft_heading = heading_ref ? XPLMGetDataf(heading_ref) : 0.0f;
  double aircraft_x{}, aircraft_y{}, aircraft_z{};
  double base_x{}, base_y{}, base_z{};
  XPLMWorldToLocal(aircraft_lat, aircraft_lon, 0.0, &aircraft_x, &aircraft_y, &aircraft_z);
  XPLMWorldToLocal(jetway.latitude, jetway.longitude, 0.0, &base_x, &base_y, &base_z);

  constexpr double radians = 3.14159265358979323846 / 180.0;
  const double aircraft_angle = static_cast<double>(aircraft_heading) * radians;
  DoorProfile door = door_profile(aircraft_icao());
  if (jetway.kinematics.door_override) {
    door.forward_m = jetway.kinematics.door_forward_m;
    door.right_m = jetway.kinematics.door_right_m;
    door.sill_height_m = jetway.kinematics.door_sill_height_m;
  }
  const double door_x = aircraft_x + std::sin(aircraft_angle) * door.forward_m +
                        std::cos(aircraft_angle) * door.right_m;
  const double door_z = aircraft_z - std::cos(aircraft_angle) * door.forward_m +
                        std::sin(aircraft_angle) * door.right_m;
  const double dx = door_x - base_x;
  const double dz = door_z - base_z;

  const double object_angle = static_cast<double>(jetway.kinematics.object_heading_deg) * radians;
  return {static_cast<float>(std::cos(object_angle) * dx + std::sin(object_angle) * dz),
          door.sill_height_m,
          static_cast<float>(-std::sin(object_angle) * dx + std::cos(object_angle) * dz)};
}

float position_error(const ssa::JetwayKinematics& k, const Vec3& door,
                     const std::array<float, 4>& ratios) {
  const Vec3 head = head_local(k, ratios);
  return std::sqrt((head.x - door.x) * (head.x - door.x) +
                   (head.y - door.y) * (head.y - door.y) +
                   (head.z - door.z) * (head.z - door.z));
}

JetwaySolution solve_jetway(const ssa::ServiceObject& jetway, const Vec3& door) {
  JetwaySolution best;
  float best_score = 1.0e30f;
  for (int r = 0; r <= 5; ++r) {
    for (int e = 0; e <= 5; ++e) {
      for (int h = 0; h <= 5; ++h) {
        std::array<float, 4> candidate{r / 5.0f, e / 5.0f, h / 5.0f,
                                       jetway.kinematics.cabin_pre_align_ratio};
        const float error = position_error(jetway.kinematics, door, candidate);
        const float score = error * error;
        if (score < best_score) {
          best_score = score;
          best.ratios = candidate;
        }
      }
    }
  }
  float step = 0.1f;
  for (int iteration = 0; iteration < 9; ++iteration, step *= 0.5f) {
    for (size_t axis = 0; axis < 3; ++axis) {
      const auto center = best.ratios;
      for (int direction : {-1, 1}) {
        auto candidate = center;
        candidate[axis] = std::clamp(candidate[axis] + direction * step, 0.0f, 1.0f);
        const float error = position_error(jetway.kinematics, door, candidate);
        const float score = error * error;
        if (score < best_score) {
          best_score = score;
          best.ratios = candidate;
        }
      }
    }
  }
  best.position_error_m = position_error(jetway.kinematics, door, best.ratios);
  return best;
}

float channel_progress(const ssa::ServiceObject& object, const char* name) {
  const auto found = std::find_if(object.channels.begin(), object.channels.end(),
                                  [&](const auto& channel) { return channel.name == name; });
  return found == object.channels.end() ? 0.0f : found->progress;
}

std::array<float, 4> current_ratios(const ssa::ServiceObject& object) {
  return {channel_progress(object, "rotunda"), channel_progress(object, "extension"),
          channel_progress(object, "height"), channel_progress(object, "cabin_yaw")};
}

bool docking_channels_at_target(const ssa::ServiceObject& object) {
  for (const char* name : {"rotunda", "extension", "height", "cabin_yaw"}) {
    const auto found = std::find_if(object.channels.begin(), object.channels.end(),
                                    [&](const auto& channel) { return channel.name == name; });
    if (found == object.channels.end() || std::abs(found->progress - found->target) > 0.001f)
      return false;
  }
  return true;
}

bool apply_door_target(ssa::ServiceObject& jetway) {
  if (!scenery) return false;
  if (!jetway.kinematics.enabled) {
    scenery->set_uniform_target(jetway, 1.0f);
    return true;
  }

  float aircraft_heading{};
  const Vec3 door = door_local(jetway, aircraft_heading);
  const JetwaySolution solution = solve_jetway(jetway, door);
  jetway.solution_error_m = solution.position_error_m;
  char diagnostic[256];
  std::snprintf(diagnostic, sizeof(diagnostic),
                "Jetway '%s' door local=(%.2f, %.2f, %.2f), aircraft heading=%.1f, solution=(%.3f, %.3f, %.3f, %.3f), error=%.2f m",
                jetway.label.c_str(), door.x, door.y, door.z, aircraft_heading, solution.ratios[0],
                solution.ratios[1], solution.ratios[2], solution.ratios[3],
                solution.position_error_m);
  log(diagnostic);
  if (solution.position_error_m > jetway.kinematics.max_solution_error_m) {
    scenery->set_uniform_target(jetway, 0.0f);
    jetway.jetway_state = ssa::JetwayState::OutOfRange;
    jetway.head_error_m = solution.position_error_m;
    return false;
  }
  jetway.target = 1.0f;
  jetway.solution_targets = solution.ratios;
  jetway.solution_ready = true;
  jetway.jetway_state = ssa::JetwayState::WheelAligning;
  // SAM-like sequence: steer the bogie before any bridge motion. This keeps
  // the wheels pointing along the upcoming path instead of visually drifting.
  scenery->set_channel_target(jetway, "rotunda", 0.0f);
  scenery->set_channel_target(jetway, "extension", 0.0f);
  scenery->set_channel_target(jetway, "height", 0.0f);
  scenery->set_channel_target(jetway, "cabin_yaw", 0.0f);
  scenery->set_channel_target(jetway, "wheel_steer", solution.ratios[0]);
  scenery->set_channel_target(jetway, "wheel_rotation", 0.0f);
  return true;
}

bool channel_at_target(const ssa::ServiceObject& object, const char* name) {
  const auto found = std::find_if(object.channels.begin(), object.channels.end(),
                                  [&](const auto& channel) { return channel.name == name; });
  return found != object.channels.end() && std::abs(found->progress - found->target) <= 0.001f;
}

void advance_jetway_docking(ssa::ServiceObject& jetway) {
  if (!jetway.kinematics.enabled || !jetway.solution_ready || jetway.target < 0.5f ||
      jetway.jetway_state == ssa::JetwayState::OutOfRange) return;
  float aircraft_heading{};
  const Vec3 door = door_local(jetway, aircraft_heading);
  const auto ratios = current_ratios(jetway);
  jetway.head_error_m = position_error(jetway.kinematics, door, ratios);

  if (jetway.jetway_state == ssa::JetwayState::WheelAligning &&
      channel_at_target(jetway, "wheel_steer")) {
    scenery->set_channel_target(jetway, "cabin_yaw",
                                jetway.kinematics.cabin_pre_align_ratio);
    jetway.jetway_state = ssa::JetwayState::HeadPreAligning;
    return;
  }
  if (jetway.jetway_state == ssa::JetwayState::HeadPreAligning &&
      channel_at_target(jetway, "cabin_yaw")) {
    scenery->set_channel_target(jetway, "rotunda", jetway.solution_targets[0]);
    scenery->set_channel_target(jetway, "height", jetway.solution_targets[2]);
    jetway.jetway_state = ssa::JetwayState::Aligning;
    return;
  }
  if (jetway.jetway_state == ssa::JetwayState::Aligning &&
      channel_at_target(jetway, "rotunda") && channel_at_target(jetway, "height")) {
    const float clearance_ratio = jetway.kinematics.pre_dock_clearance_m /
                                  std::max(0.1f, std::abs(jetway.kinematics.extension_x_m));
    const float pre_dock_extension =
        std::max(0.0f, jetway.solution_targets[1] - clearance_ratio);
    scenery->set_channel_target(jetway, "extension", pre_dock_extension);
    scenery->set_channel_target(jetway, "wheel_rotation", pre_dock_extension);
    jetway.jetway_state = ssa::JetwayState::Approaching;
    return;
  }
  if (jetway.jetway_state == ssa::JetwayState::Approaching &&
      channel_at_target(jetway, "extension")) {
    scenery->set_channel_target(jetway, "extension", jetway.solution_targets[1]);
    scenery->set_channel_target(jetway, "wheel_rotation", jetway.solution_targets[1]);
    jetway.jetway_state = ssa::JetwayState::Sealing;
    return;
  }
  if (jetway.jetway_state != ssa::JetwayState::Sealing &&
      jetway.jetway_state != ssa::JetwayState::Connected) return;

  if (jetway.head_error_m <= jetway.kinematics.connect_tolerance_m) {
    for (auto& channel : jetway.channels) channel.target = channel.progress;
    jetway.jetway_state = ssa::JetwayState::Connected;
    jetway.progress = 1.0f;
  } else if (jetway.jetway_state == ssa::JetwayState::Connected &&
             jetway.head_error_m > 0.30f) {
    // If the aircraft moves after docking, retract instead of stretching the
    // scenery object through the fuselage.
    scenery->set_uniform_target(jetway, 0.0f);
  } else if (docking_channels_at_target(jetway)) {
    // The requested pose has been reached, but the cabin head is still too far
    // from the door. Never report a false CONNECTED state.
    jetway.jetway_state = ssa::JetwayState::OutOfRange;
  }
}

int command_handler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
  if (phase == xplm_CommandBegin && tablet) tablet->toggle();
  return 1;
}

int developer_mode_handler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
  if (phase == xplm_CommandBegin && tablet) tablet->toggle_developer_mode();
  return 1;
}

void reload_scenery() {
  if (!scenery) return;
  char root[2048]{};
  XPLMGetSystemPath(root);
  const bool loaded = scenery->load(root);
  if (route_editor) {
    route_editor->load(root);
    route_editor->schedule_saved_route_load();
  }
  if (vdgs_editor) vdgs_editor->load(root);
  if (announcements) announcements->load(root);
  if (announcement_editor) announcement_editor->load(root);
  // A reloaded jetway must be planned again from a known parked state; old
  // channel targets belong to the previous configuration geometry.
  for (auto& object : scenery->objects())
    if (object.type == ssa::ServiceType::Jetway) scenery->set_uniform_target(object, 0.0f);
  log("Configuration reloaded: " + std::to_string(scenery->objects().size()) +
      " object(s)" + (loaded ? "" : " (none found)"));
}

int reload_handler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
  if (phase == xplm_CommandBegin) reload_scenery();
  return 1;
}

bool control_nearest_hangar(int action) {
  if (!scenery) return false;
  const double lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
  const double lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
  auto nearby = scenery->nearby(ssa::ServiceType::Hangar, lat, lon, 2000.0);
  if (nearby.empty()) {
    log("No hangar found within 2 km");
    return false;
  }
  auto* hangar = nearby.front();
  if (action == action_open) hangar->target = 1.0f;
  else if (action == action_close) hangar->target = 0.0f;
  else hangar->target = hangar->target > 0.5f ? 0.0f : 1.0f;
  log("Nearest hangar '" + hangar->label + "' target: " +
      (hangar->target > 0.5f ? "OPEN" : "CLOSED"));
  return true;
}

bool control_nearest_jetway(int action) {
  if (!scenery) return false;
  const double lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
  const double lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
  auto nearby = scenery->nearby(ssa::ServiceType::Jetway, lat, lon, 35.0);
  if (nearby.empty()) {
    log("No jetway found within 35 m");
    return false;
  }
  auto* jetway = nearby.front();
  if (action == action_close || (action == action_toggle && jetway->target > 0.5f)) {
    scenery->set_uniform_target(*jetway, 0.0f);
  } else {
    apply_door_target(*jetway);
  }
  log("Nearest jetway '" + jetway->label + "' target: " +
      (jetway->jetway_state == ssa::JetwayState::OutOfRange ? "OUT OF RANGE" :
       jetway->target > 0.5f ? "WHEEL ALIGNING" : "PARKED"));
  return true;
}

void toggle_tablet_object(ssa::ServiceObject& object) {
  if (!scenery) return;
  if (object.type != ssa::ServiceType::Jetway) {
    object.target = object.target > 0.5f ? 0.0f : 1.0f;
    return;
  }
  if (object.target > 0.5f)
    scenery->set_uniform_target(object, 0.0f);
  else
    apply_door_target(object);
}

int hangar_handler(XPLMCommandRef, XPLMCommandPhase phase, void* refcon) {
  if (phase == xplm_CommandBegin) control_nearest_hangar(*static_cast<int*>(refcon));
  return 1;
}

int jetway_handler(XPLMCommandRef, XPLMCommandPhase phase, void* refcon) {
  if (phase == xplm_CommandBegin) control_nearest_jetway(*static_cast<int*>(refcon));
  return 1;
}

bool matches_icao(const std::string& icao, const std::initializer_list<const char*>& values) {
  return std::any_of(values.begin(), values.end(), [&](const char* value) { return icao == value; });
}

bool is_turboprop(const std::string& icao, int engine_type) {
  return matches_icao(icao, {"AT43", "AT45", "AT72", "AT73", "AT75", "AT76",
                             "DH8A", "DH8B", "DH8C", "DH8D", "SF34", "F50",
                             "C208", "BE20", "BE30", "JS41"}) || engine_type == 1;
}

bool is_widebody(const std::string& icao) {
  return matches_icao(icao, {"A306", "A30B", "A310", "A332", "A333", "A338",
                             "A339", "A342", "A343", "A345", "A346", "A359",
                             "A35K", "B741", "B742", "B743", "B744", "B748",
                             "B762", "B763", "B764", "B772", "B773", "B77L",
                             "B77W", "B788", "B789", "B78X", "DC10", "L101", "MD11"});
}

void menu_handler(void*, void* item_ref) {
  if (item_ref == reinterpret_cast<void*>(1)) {
    if (tablet) tablet->toggle();
  } else if (item_ref == reinterpret_cast<void*>(2)) {
    reload_scenery();
  } else if (item_ref == reinterpret_cast<void*>(3)) {
    control_nearest_hangar(action_toggle);
  } else if (item_ref == reinterpret_cast<void*>(4)) {
    control_nearest_jetway(action_toggle);
  }
}

float flight_loop(float elapsed, float, int, void*) {
  if (!scenery || !tablet) return 1.0f;
  const float frame_elapsed = std::clamp(elapsed, 0.0f, 0.10f);
  const double lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
  const double lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
  const float aircraft_heading = heading_ref ? XPLMGetDataf(heading_ref) : 0.0f;
  tablet->set_position(lat, lon, aircraft_heading);
  update_vdgs(lat, lon);
  if (vdgs_editor) vdgs_editor->update();
  if (announcements) announcements->update(frame_elapsed);
  if (vehicle_spin_test && vehicle_spin_ref) {
    float spin = vehicle_spin_ref->value() + frame_elapsed * 0.65f;
    if (spin >= 1.0f) spin -= std::floor(spin);
    vehicle_spin_ref->set(spin);
  }
  tablet->set_vehicle_test(vehicle_spin_test,
                           vehicle_steering_ref ? vehicle_steering_ref->value() : 0.0f);
  compatibility_timer += frame_elapsed;
  if (compatibility_timer >= 1.0f) {
    compatibility_timer = 0.0f;
    const bool detected = XPLMFindPluginBySignature("realops") != XPLM_NO_PLUGIN_ID;
    if (detected != realops_detected) {
      realops_detected = detected;
      log(std::string("RealOps compatibility ") + (detected ? "active" : "inactive"));
    }
    tablet->set_realops(realops_detected);
  }
  // Automatic docking calculates an independent target for every animation
  // channel instead of sending every part to its Blender value of 1.0.
  if (automatic_ref->value() > 0.5f) {
    const std::string icao = aircraft_icao();
    int prop_type = 0;
    if (prop_ref) XPLMGetDatavi(prop_ref, &prop_type, 0, 1);
    const bool turboprop = is_turboprop(icao, prop_type);
    const bool parked = (!onground_ref || XPLMGetDatai(onground_ref) != 0) &&
                        (!groundspeed_ref || std::abs(XPLMGetDataf(groundspeed_ref)) < 0.5f);
    auto nearby = scenery->nearby(ssa::ServiceType::Jetway, lat, lon, 35.0);
    const size_t connect_count = parked && !turboprop
                                     ? (is_widebody(icao) ? std::min<size_t>(2, nearby.size())
                                                          : std::min<size_t>(1, nearby.size()))
                                     : 0;
    for (auto& object : scenery->objects()) {
      if (object.type != ssa::ServiceType::Jetway) continue;
      const bool selected = std::find(nearby.begin(), nearby.begin() + connect_count, &object) !=
                            nearby.begin() + connect_count;
      if (selected) {
        if (object.jetway_state == ssa::JetwayState::Parked && object.target < 0.5f)
          apply_door_target(object);
      } else if (object.target > 0.5f) {
        scenery->set_uniform_target(object, 0.0f);
      }
    }
  }
  scenery->update(frame_elapsed, realops_detected);
  if (route_editor) route_editor->update(frame_elapsed);
  for (auto& object : scenery->objects())
    if (object.type == ssa::ServiceType::Jetway) advance_jetway_docking(object);
  return -1.0f;
}
} // namespace

PLUGIN_API int XPluginStart(char* name, char* signature, char* description) {
  std::snprintf(name, 256, "%s", "SSA");
  std::snprintf(signature, 256, "%s", "boldstudio31.scenery-service-animation");
  std::snprintf(description, 256, "%s", "Scenery Service Animation by BoldStudio31");
  try {
    // Start safely in manual mode so a newly-authored jetway cannot deploy
    // before its kinematic limits have been tested at the airport.
    automatic_ref = &refs.create("boldstudio31/ssa/jetway/automatic", 0.0f, true);
    vehicle_spin_ref = &refs.create("boldstudio31/ssa/vehicle/wheel_spin", 0.0f, true);
    vehicle_steering_ref = &refs.create("boldstudio31/ssa/vehicle/steering", 0.0f, true,
                                        -1.0f, 1.0f);
    vdgs_active_ref = &refs.create("boldstudio31/ssa/vdgs/active", 0.0f, false);
    vdgs_left_ref = &refs.create("boldstudio31/ssa/vdgs/left", 0.0f, false);
    vdgs_right_ref = &refs.create("boldstudio31/ssa/vdgs/right", 0.0f, false);
    vdgs_center_ref = &refs.create("boldstudio31/ssa/vdgs/center", 0.0f, false);
    vdgs_slow_ref = &refs.create("boldstudio31/ssa/vdgs/slow", 0.0f, false);
    vdgs_stop_ref = &refs.create("boldstudio31/ssa/vdgs/stop", 0.0f, false);
    vdgs_lateral_ref = &refs.create("boldstudio31/ssa/vdgs/lateral", 0.5f, false);
    vdgs_distance_ref = &refs.create("boldstudio31/ssa/vdgs/distance_ratio", 1.0f, false);
    scenery = std::make_unique<ssa::SceneryManager>(refs);
    char root[2048]{};
    XPLMGetSystemPath(root);
    scenery->load(root);
    route_editor = std::make_unique<ssa::RouteEditor>();
    route_editor->load(root);
    vdgs_editor = std::make_unique<ssa::VdgsEditor>();
    vdgs_editor->load(root);
    announcements = std::make_unique<ssa::AnnouncementManager>();
    announcements->load(root);
    announcement_editor = std::make_unique<ssa::AnnouncementEditor>();
    announcement_editor->load(root);
    tablet = std::make_unique<ssa::Tablet>(*scenery, *route_editor, *vdgs_editor,
                                           *announcement_editor,
                                           toggle_auto, reload_scenery,
                                           toggle_tablet_object, select_vdgs,
                                           toggle_vehicle_spin_test,
                                           set_vehicle_steering_test);

    lat_ref = XPLMFindDataRef("sim/flightmodel/position/latitude");
    lon_ref = XPLMFindDataRef("sim/flightmodel/position/longitude");
    heading_ref = XPLMFindDataRef("sim/flightmodel/position/psi");
    icao_ref = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    door_open_ref = XPLMFindDataRef("sim/cockpit2/switches/door_open");
    prop_ref = XPLMFindDataRef("sim/aircraft/prop/acf_en_type");
    onground_ref = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    groundspeed_ref = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    aircraft_length_ref = XPLMFindDataRef("sim/aircraft/view/acf_length");

    tablet_command = XPLMCreateCommand("boldstudio31/ssa/tablet/toggle", "Toggle SSA tablet");
    XPLMRegisterCommandHandler(tablet_command, command_handler, 1, nullptr);
    reload_command = XPLMCreateCommand("boldstudio31/ssa/config/reload", "Reload SSA scenery configuration");
    XPLMRegisterCommandHandler(reload_command, reload_handler, 1, nullptr);
    hangar_toggle_command = XPLMCreateCommand("boldstudio31/ssa/hangar/nearest_toggle", "Toggle nearest SSA hangar");
    hangar_open_command = XPLMCreateCommand("boldstudio31/ssa/hangar/nearest_open", "Open nearest SSA hangar");
    hangar_close_command = XPLMCreateCommand("boldstudio31/ssa/hangar/nearest_close", "Close nearest SSA hangar");
    XPLMRegisterCommandHandler(hangar_toggle_command, hangar_handler, 1, &action_toggle);
    XPLMRegisterCommandHandler(hangar_open_command, hangar_handler, 1, &action_open);
    XPLMRegisterCommandHandler(hangar_close_command, hangar_handler, 1, &action_close);
    jetway_toggle_command = XPLMCreateCommand("boldstudio31/ssa/jetway/nearest_toggle", "Toggle nearest SSA jetway");
    jetway_connect_command = XPLMCreateCommand("boldstudio31/ssa/jetway/nearest_connect", "Connect nearest SSA jetway");
    jetway_disconnect_command = XPLMCreateCommand("boldstudio31/ssa/jetway/nearest_disconnect", "Disconnect nearest SSA jetway");
    developer_mode_command = XPLMCreateCommand("boldstudio31/ssa/developer/toggle", "Toggle SSA Developer Mode");
    XPLMRegisterCommandHandler(jetway_toggle_command, jetway_handler, 1, &action_toggle);
    XPLMRegisterCommandHandler(jetway_connect_command, jetway_handler, 1, &action_open);
    XPLMRegisterCommandHandler(jetway_disconnect_command, jetway_handler, 1, &action_close);
    XPLMRegisterCommandHandler(developer_mode_command, developer_mode_handler, 1, nullptr);
    const int parent = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "SSA", nullptr, 0);
    menu = XPLMCreateMenu("SSA", XPLMFindPluginsMenu(), parent, menu_handler, nullptr);
    XPLMAppendMenuItem(menu, "Open tablet", reinterpret_cast<void*>(1), 0);
    XPLMAppendMenuItem(menu, "Reload scenery configuration", reinterpret_cast<void*>(2), 0);
    XPLMAppendMenuItem(menu, "Toggle nearest hangar", reinterpret_cast<void*>(3), 0);
    XPLMAppendMenuItem(menu, "Toggle nearest jetway", reinterpret_cast<void*>(4), 0);
    XPLMRegisterFlightLoopCallback(flight_loop, -1.0f, nullptr);
    log("SSA 0.22.1 started: " + std::to_string(scenery->objects().size()) +
        " object(s), " + std::to_string(announcements->source_count()) +
        " 3D announcement(s), L1 door dataref " +
        (door_open_ref ? "detected" : "not found"));
    return 1;
  } catch (const std::exception& e) {
    log(std::string("Start failed: ") + e.what());
    return 0;
  }
}

PLUGIN_API void XPluginStop() {
  XPLMUnregisterFlightLoopCallback(flight_loop, nullptr);
  if (tablet_command) XPLMUnregisterCommandHandler(tablet_command, command_handler, 1, nullptr);
  if (reload_command) XPLMUnregisterCommandHandler(reload_command, reload_handler, 1, nullptr);
  if (hangar_toggle_command) XPLMUnregisterCommandHandler(hangar_toggle_command, hangar_handler, 1, &action_toggle);
  if (hangar_open_command) XPLMUnregisterCommandHandler(hangar_open_command, hangar_handler, 1, &action_open);
  if (hangar_close_command) XPLMUnregisterCommandHandler(hangar_close_command, hangar_handler, 1, &action_close);
  if (jetway_toggle_command) XPLMUnregisterCommandHandler(jetway_toggle_command, jetway_handler, 1, &action_toggle);
  if (jetway_connect_command) XPLMUnregisterCommandHandler(jetway_connect_command, jetway_handler, 1, &action_open);
  if (jetway_disconnect_command) XPLMUnregisterCommandHandler(jetway_disconnect_command, jetway_handler, 1, &action_close);
  if (developer_mode_command) XPLMUnregisterCommandHandler(developer_mode_command, developer_mode_handler, 1, nullptr);
  if (menu) XPLMDestroyMenu(menu);
  tablet.reset();
  announcements.reset();
  announcement_editor.reset();
  vdgs_editor.reset();
  route_editor.reset();
  scenery.reset();
  refs.clear();
}

PLUGIN_API int XPluginEnable() { return 1; }
PLUGIN_API void XPluginDisable() {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int message, void*) {
  if (message == XPLM_MSG_SCENERY_LOADED) reload_scenery();
}
