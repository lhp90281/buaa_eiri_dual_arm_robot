#include "eiriarm_controllers/dm_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <thread>

#include "yaml-cpp/yaml.h"

namespace eiriarm_controllers
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

bool parse_double(const std::string & s, double & out)
{
  try { out = std::stod(s); return true; } catch (...) { return false; }
}

bool parse_int(const std::string & s, int & out)
{
  try { out = std::stoi(s); return true; } catch (...) { return false; }
}

double sanitize(double v) { return std::isfinite(v) ? v : 0.0; }
}  // namespace

DMHardwareInterface::DMHardwareInterface() = default;

double DMHardwareInterface::wrap_to_window(double x, double center)
{
  // Wrap x into the half-open interval (center - pi, center + pi].
  double y = std::fmod(x - center + M_PI, kTwoPi);
  if (y <= 0.0) {
    y += kTwoPi;
  }
  return y - M_PI + center;
}

double DMHardwareInterface::raw_to_urdf_pos(const JointCfg & j, double raw) const
{
  const double linear = j.axis_sign * (raw - j.zero_offset);
  return j.wrap_safe ? wrap_to_window(linear, j.range_center) : linear;
}

double DMHardwareInterface::urdf_to_raw_pos(const JointCfg & j, double urdf_pos) const
{
  // Inverse of raw_to_urdf_pos (linear branch). We do not re-wrap on the raw
  // side because the motor itself accumulates the multi-turn position; the
  // commanded raw position must agree with the motor's current revolution.
  // axis_sign is +/-1 so its own inverse.
  return j.axis_sign * urdf_pos + j.zero_offset;
}

double DMHardwareInterface::urdf_to_raw_pos_near(
  const JointCfg & j, double urdf_pos, double current_raw) const
{
  const double linear = j.axis_sign * urdf_pos + j.zero_offset;
  if (!j.wrap_safe) return linear;
  // Choose the integer k such that linear + k*2pi is closest to current_raw.
  // Without this, when state_pos_ has been wrapped from a multi-turn raw
  // value the round-trip through urdf_to_raw_pos() always lands in the
  // canonical revolution (k=0), so commanding the motor to "hold its
  // current position" would force it to spin a full 2 pi to reach a
  // target on the wrong side of the wrap.
  const double diff = current_raw - linear;
  const int k = static_cast<int>(std::round(diff / kTwoPi));
  return linear + k * kTwoPi;
}

bool DMHardwareInterface::load_offsets_yaml(const std::string & path)
{
  if (path.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("DMHardwareInterface"),
                "No offsets_yaml hardware_parameter set; using zero_offset=0, axis_sign=1 for all joints");
    return true;
  }

  std::ifstream f(path);
  if (!f.good()) {
    RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                 "offsets_yaml '%s' not readable", path.c_str());
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                 "Failed to parse offsets_yaml '%s': %s", path.c_str(), e.what());
    return false;
  }

  if (!root["offsets"] || !root["offsets"].IsSequence()) {
    RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                 "offsets_yaml '%s' has no 'offsets' sequence", path.c_str());
    return false;
  }

  std::unordered_map<std::string, std::pair<double, double>> by_name;  // name -> (zero_offset, axis_sign)
  for (const auto & e : root["offsets"]) {
    if (!e["name"]) continue;
    std::string name = e["name"].as<std::string>();
    double zo = e["zero_offset"] ? e["zero_offset"].as<double>() : 0.0;
    double sign = e["axis_sign"] ? e["axis_sign"].as<double>() : 1.0;
    by_name[name] = {zo, sign};
  }

  size_t hit = 0;
  for (auto & j : joints_) {
    auto it = by_name.find(j.name);
    if (it != by_name.end()) {
      j.zero_offset = it->second.first;
      j.axis_sign = it->second.second;
      ++hit;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("DMHardwareInterface"),
                  "Joint '%s' not found in offsets_yaml; using zero_offset=0, axis_sign=1",
                  j.name.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("DMHardwareInterface"),
              "Loaded calibration for %zu/%zu joints from %s",
              hit, joints_.size(), path.c_str());
  return true;
}

hardware_interface::CallbackReturn DMHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- hardware-level params ----
  if (auto it = info_.hardware_parameters.find("offsets_yaml");
      it != info_.hardware_parameters.end()) {
    offsets_yaml_path_ = it->second;
  }
  if (auto it = info_.hardware_parameters.find("motor_topic_ns");
      it != info_.hardware_parameters.end()) {
    motor_topic_ns_ = it->second;
  }
  if (auto it = info_.hardware_parameters.find("auto_enable");
      it != info_.hardware_parameters.end()) {
    auto_enable_ = (it->second == "true" || it->second == "1");
  }

  // ---- per-joint params ----
  joints_.clear();
  joints_.reserve(info_.joints.size());
  joint_name_to_index_.clear();
  joints_by_channel_.clear();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & jinfo = info_.joints[i];
    JointCfg j;
    j.name = jinfo.name;

    auto get = [&jinfo](const std::string & k) -> std::string {
      auto it = jinfo.parameters.find(k);
      return it != jinfo.parameters.end() ? it->second : std::string{};
    };

    int ch = 1, slot = 0;
    if (!parse_int(get("channel"), ch)) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Joint '%s' missing/invalid <param name=\"channel\">", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!parse_int(get("slot"), slot)) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Joint '%s' missing/invalid <param name=\"slot\">", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (slot < 0 || slot > 7) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Joint '%s' slot=%d out of range [0,7]", j.name.c_str(), slot);
      return hardware_interface::CallbackReturn::ERROR;
    }
    j.channel = ch;
    j.slot = slot;
    j.motor_type = get("motor_type");

    double v;
    if (parse_double(get("urdf_lower"), v)) j.urdf_lower = v;
    if (parse_double(get("urdf_upper"), v)) j.urdf_upper = v;
    if (parse_double(get("pos_max"), v)) j.pos_max = v;
    if (parse_double(get("vel_max"), v)) j.vel_max = v;
    if (parse_double(get("tor_max"), v)) j.tor_max = v;

    if (j.urdf_upper <= j.urdf_lower) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Joint '%s' urdf_upper (%.3f) <= urdf_lower (%.3f)",
                   j.name.c_str(), j.urdf_upper, j.urdf_lower);
      return hardware_interface::CallbackReturn::ERROR;
    }
    j.range_center = 0.5 * (j.urdf_lower + j.urdf_upper);
    j.wrap_safe = (j.urdf_upper - j.urdf_lower) <= kTwoPi - 1e-6;

    joint_name_to_index_[j.name] = i;
    joints_by_channel_[j.channel].push_back(i);
    joints_.push_back(std::move(j));
  }

  unique_channels_.clear();
  for (const auto & kv : joints_by_channel_) {
    unique_channels_.push_back(kv.first);
  }

  // ---- storage allocation ----
  const size_t n = joints_.size();
  state_pos_.assign(n, 0.0);
  state_vel_.assign(n, 0.0);
  state_eff_.assign(n, 0.0);
  cmd_pos_.assign(n, 0.0);
  cmd_vel_.assign(n, 0.0);
  cmd_eff_.assign(n, 0.0);
  cmd_kp_.assign(n, 0.0);
  cmd_kd_.assign(n, 0.0);
  last_err_.assign(n, -1);
  last_pos_raw_.assign(n, 0.0);

  // ---- load calibration ----
  if (!load_offsets_yaml(offsets_yaml_path_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- log summary ----
  for (const auto & j : joints_) {
    RCLCPP_INFO(rclcpp::get_logger("DMHardwareInterface"),
                "  %-12s ch%d.slot%d  type=%-8s offset=%+.4f sign=%+.0f urdf=[%+.3f,%+.3f] center=%+.3f wrap_safe=%s",
                j.name.c_str(), j.channel, j.slot, j.motor_type.c_str(),
                j.zero_offset, j.axis_sign, j.urdf_lower, j.urdf_upper,
                j.range_center, j.wrap_safe ? "true" : "false");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DMHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>("dm_hardware_interface");

  for (int ch : unique_channels_) {
    const std::string state_t  = motor_topic_ns_ + "/ch" + std::to_string(ch) + "/state";
    const std::string cmd_t    = motor_topic_ns_ + "/ch" + std::to_string(ch) + "/cmd";
    const std::string enable_t = motor_topic_ns_ + "/ch" + std::to_string(ch) + "/motor_enable";

    state_subs_[ch] = node_->create_subscription<usb2can::msg::MotorStateArray>(
      state_t, rclcpp::SensorDataQoS(),
      [this, ch](usb2can::msg::MotorStateArray::SharedPtr msg) {
        on_motor_state(ch, msg);
      });
    cmd_pubs_[ch] = node_->create_publisher<usb2can::msg::MotorCommandArray>(cmd_t, 10);
    enable_pubs_[ch] = node_->create_publisher<usb2can::msg::MotorEnableArray>(enable_t, 10);
    state_seen_[ch] = false;

    RCLCPP_INFO(node_->get_logger(),
                "ch%d:  sub %s   pub %s   pub %s",
                ch, state_t.c_str(), cmd_t.c_str(), enable_t.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DMHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(joints_.size() * 3);
  for (size_t i = 0; i < joints_.size(); ++i) {
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &state_pos_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &state_vel_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_EFFORT,   &state_eff_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
DMHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(joints_.size() * 5);
  for (size_t i = 0; i < joints_.size(); ++i) {
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_EFFORT,   &cmd_eff_[i]);
    out.emplace_back(joints_[i].name, "stiffness", &cmd_kp_[i]);
    out.emplace_back(joints_[i].name, "damping",   &cmd_kd_[i]);
  }
  return out;
}

void DMHardwareInterface::publish_enable_all(bool enable)
{
  for (int ch : unique_channels_) {
    usb2can::msg::MotorEnableArray msg;
    msg.header.stamp = node_->now();
    msg.channel = static_cast<uint8_t>(ch);
    for (size_t idx : joints_by_channel_[ch]) {
      usb2can::msg::MotorEnable m;
      m.id = static_cast<uint8_t>(joints_[idx].slot);
      m.enable = enable;
      msg.motors.push_back(m);
    }
    enable_pubs_[ch]->publish(msg);
  }
}

void DMHardwareInterface::publish_zero_command_all()
{
  for (int ch : unique_channels_) {
    usb2can::msg::MotorCommandArray msg;
    msg.header.stamp = node_->now();
    msg.channel = static_cast<uint8_t>(ch);
    for (size_t idx : joints_by_channel_[ch]) {
      usb2can::msg::MotorCommand m;
      m.id = static_cast<uint8_t>(joints_[idx].slot);
      m.position = 0.0f;
      m.velocity = 0.0f;
      m.kp = 0.0f;
      m.kd = 0.0f;
      m.torque_ff = 0.0f;
      msg.motors.push_back(m);
    }
    cmd_pubs_[ch]->publish(msg);
  }
}

hardware_interface::CallbackReturn DMHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands to safe defaults so an effort-only controller does not
  // accidentally generate spring/damping torques via stale kp/kd.
  std::fill(cmd_pos_.begin(), cmd_pos_.end(), 0.0);
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0.0);
  std::fill(cmd_eff_.begin(), cmd_eff_.end(), 0.0);
  std::fill(cmd_kp_.begin(), cmd_kp_.end(), 0.0);
  std::fill(cmd_kd_.begin(), cmd_kd_.end(), 0.0);

  quiescing_ = false;

  if (!auto_enable_) {
    RCLCPP_INFO(node_->get_logger(),
                "auto_enable=false; user must publish enable manually");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ---- synchronous ENABLE: every 20 ms (50 Hz) we publish BOTH:
  //   - a zero-impedance MotorCommandArray (cmd byte = slot mask, payload = MIT zero)
  //   - a MotorEnableArray with enable=true (cmd byte = slot mask, payload = DM_ENABLE)
  // Reasons:
  //   1) The STM32 watchdog disables every motor in prev_mask if no
  //      DcuCommand arrives for >100 ms (see USB2CAN README §10.2).
  //      During on_activate ros2_control's update() is not running yet,
  //      so write() never runs. 20 ms cadence << 100 ms watchdog -> safe.
  //   2) Each MotorEnable publish becomes one DcuCommand carrying the
  //      DM_ENABLE special-byte sequence to every tracked motor. A DM
  //      motor in disabled state needs to actually receive this sequence
  //      to transition to MIT. Empirically (e.g. left ch1 odd slots) some
  //      motors miss the transition window when ENABLE is published at
  //      only 5 Hz; sending it 10x more often (50 Hz) resolves the flake.
  //   3) DM motors are request-response: each cmd frame triggers exactly
  //      one state frame. Two 50 Hz publishes therefore give us a steady
  //      ~100 Hz feed of err codes to poll.
  // Both frame types are safe pre-MIT: a disabled motor still echoes a
  // state frame for a zero-cmd but does not move; DM_ENABLE in MIT is a
  // no-op (or re-asserts MIT) per the DM datasheet.
  static constexpr int DM_ERR_ENABLED = 1;
  const double timeout_s = 5.0;
  const auto loop_period = std::chrono::milliseconds(20);  // 50 Hz both
  const auto t0 = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(),
              "ENABLE: 50Hz zero-cmd + 50Hz motor_enable publish until all "
              "%zu joint(s) report err=%d (timeout %.1fs)...",
              joints_.size(), DM_ERR_ENABLED, timeout_s);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now - t0).count();
    if (elapsed_s > timeout_s) break;
    // Both at 50 Hz: zero-cmd keeps STM32 watchdog quiet AND ENABLE
    // keeps hammering the DM_ENABLE special-byte sequence at every motor
    // until each one transitions to MIT.
    publish_zero_command_all();
    publish_enable_all(true);
    bool all_enabled = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (last_err_[i] != DM_ERR_ENABLED) {
        all_enabled = false;
        break;
      }
    }
    if (all_enabled) {
      RCLCPP_INFO(node_->get_logger(),
                  "ENABLED (all %zu motor(s) report err=%d)",
                  joints_.size(), DM_ERR_ENABLED);
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(loop_period);
  }
  // Timeout: list per-joint err for diagnosis and fail activation so
  // ros2_control surfaces the error to the user.
  std::string offenders;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (last_err_[i] != DM_ERR_ENABLED) {
      if (!offenders.empty()) offenders += ", ";
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s(err=%d)",
                    joints_[i].name.c_str(), last_err_[i]);
      offenders += buf;
    }
  }
  RCLCPP_ERROR(node_->get_logger(),
               "ENABLE timed out after %.1fs; not enabled: %s",
               timeout_s, offenders.c_str());
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn DMHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Force write() into zero-cmd mode for the rest of this transition --
  // ros2_control's update loop may still race with this callback.
  quiescing_ = true;

  // Burst zero-cmd so motors are passive when DISABLE lands.
  for (int i = 0; i < 3; ++i) {
    publish_zero_command_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!auto_enable_) {
    RCLCPP_INFO(node_->get_logger(),
                "Deactivated; commands zeroed (auto_enable=false, no DISABLE sent)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ---- synchronous DISABLE: 5 Hz publish until every joint reports
  // err == DM_ERR_DISABLED (0). We *keep* sending cmd via write() (still
  // being called by CM until on_deactivate returns) -- DM is
  // request-response, so without cmd we'd never see a fresh err code.
  // quiescing_ above forces those cmds to zero so they can't inject torque.
  //
  // Note: we explicitly poll for err==0 (disabled) rather than err!=1
  // (anything-but-enabled). Error codes 8..14 (over-volt / over-current /
  // over-temp / etc.) also satisfy err!=1 but they do NOT mean the motor
  // has cleanly disabled -- treating them as "DISABLED" would silently
  // swallow a real fault on shutdown.
  static constexpr int DM_ERR_DISABLED = 0;
  const double timeout_s = 5.0;
  const double publish_period_s = 0.05;
  const auto t0 = std::chrono::steady_clock::now();
  auto last_pub = t0 - std::chrono::seconds(1);
  RCLCPP_INFO(node_->get_logger(),
              "DISABLE: %.0fHz FD pulse until all %zu joint(s) report err=%d "
              "(timeout %.1fs)...",
              1.0 / publish_period_s, joints_.size(), DM_ERR_DISABLED, timeout_s);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now - t0).count();
    if (elapsed_s > timeout_s) break;
    if (std::chrono::duration<double>(now - last_pub).count() >= publish_period_s) {
      publish_enable_all(false);
      last_pub = now;
    }
    bool all_off = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (last_err_[i] != DM_ERR_DISABLED) {
        all_off = false;
        break;
      }
    }
    if (all_off) {
      RCLCPP_INFO(node_->get_logger(),
                  "DISABLED (all %zu motor(s) report err=%d)",
                  joints_.size(), DM_ERR_DISABLED);
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  // Timeout: list per-joint err for diagnosis. Distinguish "still enabled"
  // (err==1) from "stuck in fault" (err in 8..14) so the user knows
  // whether to power-cycle the affected motor.
  std::string offenders;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (last_err_[i] != DM_ERR_DISABLED) {
      if (!offenders.empty()) offenders += ", ";
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s(err=%d)",
                    joints_[i].name.c_str(), last_err_[i]);
      offenders += buf;
    }
  }
  RCLCPP_WARN(node_->get_logger(),
              "DISABLE timed out after %.1fs; not disabled: %s",
              timeout_s, offenders.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

void DMHardwareInterface::on_motor_state(int ch,
  const usb2can::msg::MotorStateArray::SharedPtr msg)
{
  latest_state_[ch] = msg;
  state_seen_[ch] = true;

  // Per-joint sentinel filter + full state update. dm_motor_bridge
  // publishes -pos_max/-vel_max/-tor_max for slots whose motor did not
  // respond to the last STM32 frame; in that case b[0] is also 0 so the
  // unpacked err==0 is meaningless. Treating the sentinel as real motor
  // data would corrupt state_pos_ / state_vel_ / state_eff_ by several
  // rad, which then trips the joint_position_controller: on_activate()
  // captures the corrupt value as hold_pos_, and the next write() commands
  // a setpoint roughly that far from the actual joint, slamming the motor
  // into whichever physical limit lies on the kp side. Likewise it would
  // corrupt last_err_ (the on_activate / on_deactivate signal) and
  // last_pos_raw_ (the bumpless mirror in write()).
  //
  // We therefore make on_motor_state the *single* source of truth for
  // state_*_ / last_err_ / last_pos_raw_; read() just drains the
  // subscription queue. State stays at its previous value through any
  // sentinel transient, which is the correct behaviour: a brief drop is
  // hidden, and a permanent silence freezes state at the last known good
  // value (downstream controllers will see no change rather than a slam).
  constexpr double EPS = 1e-3;
  for (size_t idx : joints_by_channel_[ch]) {
    const auto & j = joints_[idx];
    if (j.slot >= static_cast<int>(msg->motors.size())) continue;
    const auto & m = msg->motors[j.slot];
    const bool is_sentinel =
      std::abs(m.position + j.pos_max) < EPS &&
      std::abs(m.velocity + j.vel_max) < EPS &&
      std::abs(m.torque   + j.tor_max) < EPS;
    if (is_sentinel) continue;
    last_err_[idx]     = static_cast<int>(m.err);
    last_pos_raw_[idx] = static_cast<double>(m.position);
    state_pos_[idx] = raw_to_urdf_pos(j, static_cast<double>(m.position));
    state_vel_[idx] = j.axis_sign * static_cast<double>(m.velocity);
    state_eff_[idx] = j.axis_sign * static_cast<double>(m.torque);
  }
}

hardware_interface::return_type DMHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // State interfaces are populated in on_motor_state (sentinel-filtered);
  // here we just dispatch any pending state messages so the cache is
  // up-to-date before controllers' update() runs.
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DMHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // For every channel that has joints, emit one MotorCommandArray with
  // exactly those slots filled. Slots not listed are auto-zeroed by
  // dm_motor_bridge per its message contract.
  //
  // While quiescing_ is true (set by on_deactivate), force every field to
  // zero so a still-active controller cannot inject torques into a motor
  // that we are simultaneously trying to disable.
  for (int ch : unique_channels_) {
    usb2can::msg::MotorCommandArray msg;
    msg.header.stamp = node_->now();
    msg.channel = static_cast<uint8_t>(ch);
    for (size_t idx : joints_by_channel_[ch]) {
      const auto & j = joints_[idx];
      double pos_urdf, vel_urdf, eff_urdf, kp, kd;
      if (quiescing_) {
        pos_urdf = vel_urdf = eff_urdf = kp = kd = 0.0;
      } else {
        pos_urdf = sanitize(cmd_pos_[idx]);
        vel_urdf = sanitize(cmd_vel_[idx]);
        eff_urdf = sanitize(cmd_eff_[idx]);
        kp       = std::max(0.0, sanitize(cmd_kp_[idx]));
        kd       = std::max(0.0, sanitize(cmd_kd_[idx]));
      }

      // Safety mirror for pure-torque mode (kp == kd == 0):
      //   Set pos_cmd = last raw motor position, vel_cmd = 0. The STM32
      //   watchdog falls back to (kp=1, kd=1, last pos_cmd, last vel_cmd)
      //   if the upstream link drops for >100 ms. If pos_cmd were left at
      //   its stale value (e.g. zero_offset from on_activate), that
      //   fallback would snap the motor toward a target several rad away
      //   from where the joint actually is right now, producing a violent
      //   torque spike on a comm-drop -- exactly the failure mode this
      //   block prevents.
      //
      //   We sidestep cmd_pos_ entirely in this branch: a pure-torque
      //   controller leaves cmd_pos_ at whatever was last written (0 by
      //   default), so deriving pos_raw from cmd_pos_ would not track the
      //   actual joint. last_pos_raw_ stays in raw frame and reflects the
      //   live motor position, so it is the correct mirror target.
      //
      //   The branch is skipped while quiescing_ is true -- in that path
      //   pos_cmd=0 is paired with an active DISABLE on the same channel,
      //   so the motor will not act on it.
      double pos_raw, vel_raw;
      if (!quiescing_ && kp == 0.0 && kd == 0.0) {
        pos_raw = last_pos_raw_[idx];
        vel_raw = 0.0;
      } else {
        // Use the multi-turn-aware inverse so the commanded raw position
        // tracks the motor's current revolution. urdf_to_raw_pos() is the
        // canonical (k=0) inverse, which is incorrect when state_pos_ was
        // wrapped from a multi-turn raw value: the controller's hold_pos_
        // would round-trip into the wrong revolution and command a 2 pi
        // slam toward a hard stop.
        pos_raw = urdf_to_raw_pos_near(j, pos_urdf, last_pos_raw_[idx]);
        vel_raw = j.axis_sign * vel_urdf;
      }
      const double tau_raw = j.axis_sign * eff_urdf;

      usb2can::msg::MotorCommand m;
      m.id        = static_cast<uint8_t>(j.slot);
      m.position  = static_cast<float>(pos_raw);
      m.velocity  = static_cast<float>(vel_raw);
      m.kp        = static_cast<float>(kp);
      m.kd        = static_cast<float>(kd);
      m.torque_ff = static_cast<float>(tau_raw);
      msg.motors.push_back(m);
    }
    cmd_pubs_[ch]->publish(msg);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace eiriarm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  eiriarm_controllers::DMHardwareInterface,
  hardware_interface::SystemInterface)
