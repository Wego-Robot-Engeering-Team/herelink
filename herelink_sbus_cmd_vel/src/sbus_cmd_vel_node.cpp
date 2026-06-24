#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr uint8_t kMavlinkV1Magic = 0xFE;
constexpr uint8_t kMavlinkV2Magic = 0xFD;
constexpr uint32_t kHeartbeatMsgId = 0;
constexpr uint32_t kRcChannelsRawMsgId = 35;
constexpr uint32_t kRcChannelsMsgId = 65;
constexpr uint8_t kMavDataStreamRcChannels = 3;
constexpr uint16_t kMavCmdSetMessageInterval = 511;

uint16_t crc_x25(const std::vector<uint8_t> & data)
{
  uint16_t crc = 0xFFFF;
  for (const auto byte : data) {
    uint8_t tmp = byte ^ (crc & 0xFF);
    tmp ^= static_cast<uint8_t>(tmp << 4);
    crc = static_cast<uint16_t>(
      (crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^ (static_cast<uint16_t>(tmp) << 3) ^
      (static_cast<uint16_t>(tmp) >> 4));
  }
  return crc;
}

void append_le_u16(std::vector<uint8_t> & data, uint16_t value)
{
  data.push_back(static_cast<uint8_t>(value & 0xFF));
  data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void append_le_float(std::vector<uint8_t> & data, float value)
{
  uint8_t bytes[sizeof(float)];
  std::memcpy(bytes, &value, sizeof(float));
  data.insert(data.end(), bytes, bytes + sizeof(float));
}

uint16_t le_u16(const std::vector<uint8_t> & data, size_t offset)
{
  return static_cast<uint16_t>(data[offset] | (data[offset + 1] << 8));
}

speed_t baud_to_constant(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}
}  // namespace

class SbusCmdVelNode : public rclcpp::Node
{
public:
  SbusCmdVelNode()
  : Node("sbus_cmd_vel_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    baud_ = declare_parameter<int>("baud", 115200);
    linear_channel_ = declare_parameter<int>("linear_channel", 2);
    lateral_channel_ = declare_parameter<int>("lateral_channel", 4);
    angular_channel_ = declare_parameter<int>("angular_channel", 1);
    mode_channel_ = declare_parameter<int>("mode_channel", 7);
    center_pwm_ = declare_parameter<int>("center_pwm", 1524);
    min_pwm_ = declare_parameter<int>("min_pwm", 1102);
    max_pwm_ = declare_parameter<int>("max_pwm", 1927);
    deadband_pwm_ = declare_parameter<int>("deadband_pwm", 30);
    mode_threshold_pwm_ = declare_parameter<int>("mode_threshold_pwm", 1600);
    max_linear_ = declare_parameter<double>("max_linear", 1.0);
    max_lateral_ = declare_parameter<double>("max_lateral", 1.0);
    max_angular_ = declare_parameter<double>("max_angular", 1.0);
    linear_direction_ = declare_parameter<double>("linear_direction", -1.0);
    lateral_direction_ = declare_parameter<double>("lateral_direction", -1.0);
    angular_direction_ = declare_parameter<double>("angular_direction", -1.0);
    ackermann_min_linear_x_ =
      declare_parameter<double>("ackermann_min_linear_x", 0.05);
    mode_toggle_latch_ = declare_parameter<bool>("mode_toggle_latch", true);
    mode_toggle_debounce_sec_ =
      declare_parameter<double>("mode_toggle_debounce_sec", 0.3);
    initial_ackermann_mode_ =
      declare_parameter<bool>("initial_ackermann_mode", false);
    rc_timeout_sec_ = declare_parameter<double>("rc_timeout_sec", 0.5);
    publish_zero_on_timeout_ = declare_parameter<bool>("publish_zero_on_timeout", true);
    request_rate_hz_ = declare_parameter<int>("request_rate_hz", 20);
    debug_output_ = declare_parameter<bool>("debug_output", false);
    debug_log_interval_sec_ = declare_parameter<double>("debug_log_interval_sec", 1.0);
    debug_cmd_change_threshold_ = declare_parameter<double>("debug_cmd_change_threshold", 0.02);
    debug_pwm_change_threshold_ = declare_parameter<int>("debug_pwm_change_threshold", 5);
    latched_ackermann_mode_ = initial_ackermann_mode_;

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&SbusCmdVelNode::spin_once, this));

    RCLCPP_INFO(
      get_logger(),
      "Reading %s, mapping ch%d -> %s linear.x, ch%d -> linear.y, ch%d -> angular.z, ch%d mode toggle",
      port_.c_str(), linear_channel_, cmd_vel_topic_.c_str(), lateral_channel_,
      angular_channel_, mode_channel_);
  }

  ~SbusCmdVelNode() override
  {
    close_serial();
  }

private:
  struct MavlinkFrame
  {
    uint32_t msgid = 0;
    uint8_t sysid = 0;
    uint8_t compid = 0;
    std::vector<uint8_t> payload;
  };

  void spin_once()
  {
    if (fd_ < 0) {
      try_open_serial();
      return;
    }

    read_serial();
    parse_buffer();
    send_periodic_requests();
    publish_zero_if_timed_out();
  }

  void try_open_serial()
  {
    const auto now = this->now();
    if (last_open_attempt_.nanoseconds() != 0 &&
      (now - last_open_attempt_).seconds() < 1.0)
    {
      return;
    }
    last_open_attempt_ = now;

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for serial port %s: %s", port_.c_str(), std::strerror(errno));
      return;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      close_serial();
      return;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baud_to_constant(baud_));
    cfsetospeed(&tty, baud_to_constant(baud_));
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      close_serial();
      return;
    }

    buffer_.clear();
    RCLCPP_INFO(get_logger(), "Opened serial port %s", port_.c_str());
  }

  void close_serial()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void read_serial()
  {
    std::array<uint8_t, 4096> chunk{};
    while (true) {
      const auto received = ::read(fd_, chunk.data(), chunk.size());
      if (received > 0) {
        buffer_.insert(buffer_.end(), chunk.begin(), chunk.begin() + received);
        continue;
      }
      if (received == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      RCLCPP_WARN(get_logger(), "Serial read failed: %s", std::strerror(errno));
      close_serial();
      return;
    }
  }

  void parse_buffer()
  {
    size_t index = 0;
    while (index < buffer_.size()) {
      const auto magic = buffer_[index];
      if (magic == kMavlinkV1Magic) {
        if (index + 6 > buffer_.size()) {
          break;
        }
        const size_t frame_len = 6 + buffer_[index + 1] + 2;
        if (index + frame_len > buffer_.size()) {
          break;
        }
      } else if (magic == kMavlinkV2Magic) {
        if (index + 10 > buffer_.size()) {
          break;
        }
        const bool signed_packet = (buffer_[index + 2] & 0x01) != 0;
        const size_t frame_len = 10 + buffer_[index + 1] + 2 + (signed_packet ? 13 : 0);
        if (index + frame_len > buffer_.size()) {
          break;
        }
      }

      MavlinkFrame frame;
      size_t frame_len = 0;
      if (!try_parse_frame(index, frame, frame_len)) {
        ++index;
        continue;
      }

      handle_frame(frame);
      index += frame_len;
    }

    if (index > 0) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(index));
    }

    constexpr size_t kMaxBufferSize = 8192;
    if (buffer_.size() > kMaxBufferSize) {
      buffer_.erase(buffer_.begin(), buffer_.end() - kMaxBufferSize);
    }
  }

  bool try_parse_frame(size_t index, MavlinkFrame & frame, size_t & frame_len)
  {
    const auto magic = buffer_[index];
    if (magic == kMavlinkV1Magic) {
      if (index + 6 > buffer_.size()) {
        return false;
      }
      const size_t payload_len = buffer_[index + 1];
      frame_len = 6 + payload_len + 2;
      if (index + frame_len > buffer_.size()) {
        return false;
      }
      frame.sysid = buffer_[index + 3];
      frame.compid = buffer_[index + 4];
      frame.msgid = buffer_[index + 5];
      frame.payload.assign(
        buffer_.begin() + static_cast<std::ptrdiff_t>(index + 6),
        buffer_.begin() + static_cast<std::ptrdiff_t>(index + 6 + payload_len));
      return true;
    }

    if (magic == kMavlinkV2Magic) {
      if (index + 10 > buffer_.size()) {
        return false;
      }
      const size_t payload_len = buffer_[index + 1];
      const bool signed_packet = (buffer_[index + 2] & 0x01) != 0;
      frame_len = 10 + payload_len + 2 + (signed_packet ? 13 : 0);
      if (index + frame_len > buffer_.size()) {
        return false;
      }
      frame.sysid = buffer_[index + 5];
      frame.compid = buffer_[index + 6];
      frame.msgid = static_cast<uint32_t>(
        buffer_[index + 7] | (buffer_[index + 8] << 8) | (buffer_[index + 9] << 16));
      frame.payload.assign(
        buffer_.begin() + static_cast<std::ptrdiff_t>(index + 10),
        buffer_.begin() + static_cast<std::ptrdiff_t>(index + 10 + payload_len));
      return true;
    }

    return false;
  }

  void handle_frame(const MavlinkFrame & frame)
  {
    if (frame.msgid == kHeartbeatMsgId) {
      target_system_ = frame.sysid;
      target_component_ = frame.compid;
      return;
    }

    if (frame.msgid == kRcChannelsMsgId) {
      handle_rc_channels(frame.payload);
      return;
    }

    if (frame.msgid == kRcChannelsRawMsgId) {
      handle_rc_channels_raw(frame.payload);
    }
  }

  void handle_rc_channels(const std::vector<uint8_t> & payload)
  {
    if (payload.size() < 42) {
      return;
    }

    std::array<uint16_t, 18> channels{};
    for (size_t i = 0; i < channels.size(); ++i) {
      channels[i] = le_u16(payload, 4 + i * 2);
    }

    publish_cmd_vel(channels);
  }

  void handle_rc_channels_raw(const std::vector<uint8_t> & payload)
  {
    if (payload.size() < 22) {
      return;
    }

    std::array<uint16_t, 18> channels{};
    channels.fill(0xFFFF);
    for (size_t i = 0; i < 8; ++i) {
      channels[i] = le_u16(payload, 4 + i * 2);
    }

    publish_cmd_vel(channels);
  }

  void publish_cmd_vel(const std::array<uint16_t, 18> & channels)
  {
    const auto linear_pwm = channel_value(channels, linear_channel_);
    const auto lateral_pwm = channel_value(channels, lateral_channel_);
    const auto angular_pwm = channel_value(channels, angular_channel_);
    const auto mode_pwm = channel_value(channels, mode_channel_);
    if (linear_pwm == 0 || lateral_pwm == 0 || angular_pwm == 0) {
      return;
    }

    const bool ackermann_mode = current_drive_mode(mode_pwm);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = axis_from_pwm(linear_pwm) * max_linear_ * linear_direction_;
    cmd.linear.y = axis_from_pwm(lateral_pwm) * max_lateral_ * lateral_direction_;
    cmd.angular.z = -1 * axis_from_pwm(angular_pwm) * max_angular_ * angular_direction_;

    cmd.linear.x = (std::abs(cmd.linear.x) < 0.1) ? 0.0 : cmd.linear.x;
    cmd.linear.y = (std::abs(cmd.linear.y) < 0.1) ? 0.0 : cmd.linear.y;
    cmd.angular.z = (std::abs(cmd.angular.z) < 0.1) ? 0.0 : cmd.angular.z;

    if (ackermann_mode) {
      // Ackermann mode should not use lateral motion, and should not spin in place.
      cmd.linear.y = 0.0;
      if (std::abs(cmd.linear.x) < ackermann_min_linear_x_) {
        cmd.angular.z = 0.0;
      }
    }

    cmd_vel_pub_->publish(cmd);

    last_rc_time_ = this->now();
    zero_published_after_timeout_ = false;
    log_drive_mode_if_changed(ackermann_mode, mode_pwm);

    if (debug_output_ && should_print_debug(linear_pwm, lateral_pwm, angular_pwm, cmd)) {
      RCLCPP_INFO(
        get_logger(),
        "RC mode=%s ch%d=%u ch%d=%u ch%d=%u ch%d=%u -> %s linear.x=%.3f linear.y=%.3f angular.z=%.3f",
        ackermann_mode ? "ackermann" : "spin",
        linear_channel_, linear_pwm, lateral_channel_, lateral_pwm, angular_channel_, angular_pwm,
        mode_channel_, mode_pwm, cmd_vel_topic_.c_str(), cmd.linear.x, cmd.linear.y, cmd.angular.z);
    }

    RCLCPP_DEBUG(
      get_logger(), "RC ch%d=%u ch%d=%u ch%d=%u -> linear.x=%.3f linear.y=%.3f angular.z=%.3f",
      linear_channel_, linear_pwm, lateral_channel_, lateral_pwm, angular_channel_, angular_pwm,
      cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  bool should_print_debug(
    uint16_t linear_pwm,
    uint16_t lateral_pwm,
    uint16_t angular_pwm,
    const geometry_msgs::msg::Twist & cmd)
  {
    const auto now = this->now();

    const bool is_first_log = last_debug_time_.nanoseconds() == 0;
    const bool pwm_changed =
      std::abs(static_cast<int>(linear_pwm) - static_cast<int>(last_logged_linear_pwm_)) >=
        debug_pwm_change_threshold_ ||
      std::abs(static_cast<int>(lateral_pwm) - static_cast<int>(last_logged_lateral_pwm_)) >=
        debug_pwm_change_threshold_ ||
      std::abs(static_cast<int>(angular_pwm) - static_cast<int>(last_logged_angular_pwm_)) >=
        debug_pwm_change_threshold_;
    const bool cmd_changed =
      std::abs(cmd.linear.x - last_logged_linear_x_) >= debug_cmd_change_threshold_ ||
      std::abs(cmd.linear.y - last_logged_linear_y_) >= debug_cmd_change_threshold_ ||
      std::abs(cmd.angular.z - last_logged_angular_z_) >= debug_cmd_change_threshold_;
    const bool interval_elapsed =
      last_debug_time_.nanoseconds() != 0 &&
      (now - last_debug_time_).seconds() >= debug_log_interval_sec_;

    if (!is_first_log && !pwm_changed && !cmd_changed && !interval_elapsed) {
      return false;
    }

    last_debug_time_ = now;
    last_logged_linear_pwm_ = linear_pwm;
    last_logged_lateral_pwm_ = lateral_pwm;
    last_logged_angular_pwm_ = angular_pwm;
    last_logged_linear_x_ = cmd.linear.x;
    last_logged_linear_y_ = cmd.linear.y;
    last_logged_angular_z_ = cmd.angular.z;
    return true;
  }

  bool is_ackermann_mode(uint16_t mode_pwm) const
  {
    if (mode_channel_ <= 0 || mode_pwm == 0) {
      return false;
    }
    return mode_pwm >= static_cast<uint16_t>(mode_threshold_pwm_);
  }

  bool current_drive_mode(uint16_t mode_pwm)
  {
    const bool switch_high = is_ackermann_mode(mode_pwm);
    if (!mode_toggle_latch_) {
      return switch_high;
    }

    const auto now = this->now();
    if (!mode_input_initialized_) {
      mode_input_initialized_ = true;
      last_mode_switch_high_ = switch_high;
      return latched_ackermann_mode_;
    }

    const bool rising_edge = switch_high && !last_mode_switch_high_;
    const bool debounce_ok =
      last_mode_toggle_time_.nanoseconds() == 0 ||
      (now - last_mode_toggle_time_).seconds() >= mode_toggle_debounce_sec_;

    if (rising_edge && debounce_ok) {
      latched_ackermann_mode_ = !latched_ackermann_mode_;
      last_mode_toggle_time_ = now;
    }

    last_mode_switch_high_ = switch_high;
    return latched_ackermann_mode_;
  }

  void log_drive_mode_if_changed(bool ackermann_mode, uint16_t mode_pwm)
  {
    if (!last_mode_initialized_ || last_ackermann_mode_ != ackermann_mode) {
      RCLCPP_INFO(
        get_logger(), "Drive mode switched to %s (ch%d=%u, threshold=%d)",
        ackermann_mode ? "ACKERMANN" : "SPIN", mode_channel_, mode_pwm, mode_threshold_pwm_);
      last_mode_initialized_ = true;
      last_ackermann_mode_ = ackermann_mode;
    }
  }

  uint16_t channel_value(const std::array<uint16_t, 18> & channels, int channel) const
  {
    if (channel < 1 || channel > static_cast<int>(channels.size())) {
      return 0;
    }
    const auto value = channels[static_cast<size_t>(channel - 1)];
    return value == 0xFFFF ? 0 : value;
  }

  double axis_from_pwm(uint16_t pwm) const
  {
    const int delta = static_cast<int>(pwm) - center_pwm_;
    if (std::abs(delta) <= deadband_pwm_) {
      return 0.0;
    }

    const double span = delta > 0 ? (max_pwm_ - center_pwm_) : (center_pwm_ - min_pwm_);
    if (span <= 0.0) {
      return 0.0;
    }

    return std::clamp(static_cast<double>(delta) / span, -1.0, 1.0);
  }

  void publish_zero_if_timed_out()
  {
    if (!publish_zero_on_timeout_ || zero_published_after_timeout_) {
      return;
    }
    if (last_rc_time_.nanoseconds() == 0) {
      return;
    }
    if ((this->now() - last_rc_time_).seconds() < rc_timeout_sec_) {
      return;
    }

    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    zero_published_after_timeout_ = true;
    RCLCPP_WARN(get_logger(), "RC timeout. Published zero /cmd_vel.");
  }

  void send_periodic_requests()
  {
    const auto now = this->now();
    if (last_request_time_.nanoseconds() != 0 &&
      (now - last_request_time_).seconds() < 1.0)
    {
      return;
    }
    last_request_time_ = now;

    send_packet(build_heartbeat());
    send_packet(build_request_data_stream());
    send_packet(build_set_message_interval(kRcChannelsMsgId));
    send_packet(build_set_message_interval(kRcChannelsRawMsgId));
  }

  std::vector<uint8_t> build_v1_packet(uint8_t msgid, const std::vector<uint8_t> & payload, uint8_t crc_extra)
  {
    std::vector<uint8_t> packet{
      kMavlinkV1Magic,
      static_cast<uint8_t>(payload.size()),
      sequence_++,
      255,
      190,
      msgid};
    packet.insert(packet.end(), payload.begin(), payload.end());

    std::vector<uint8_t> crc_input(packet.begin() + 1, packet.end());
    crc_input.push_back(crc_extra);
    append_le_u16(packet, crc_x25(crc_input));
    return packet;
  }

  std::vector<uint8_t> build_heartbeat()
  {
    std::vector<uint8_t> payload;
    payload.resize(4, 0);
    payload.push_back(6);
    payload.push_back(8);
    payload.push_back(0);
    payload.push_back(4);
    payload.push_back(3);
    return build_v1_packet(0, payload, 50);
  }

  std::vector<uint8_t> build_request_data_stream()
  {
    std::vector<uint8_t> payload;
    append_le_u16(payload, static_cast<uint16_t>(request_rate_hz_));
    payload.push_back(target_system_);
    payload.push_back(target_component_);
    payload.push_back(kMavDataStreamRcChannels);
    payload.push_back(1);
    return build_v1_packet(66, payload, 148);
  }

  std::vector<uint8_t> build_set_message_interval(uint32_t msgid)
  {
    std::vector<uint8_t> payload;
    const float interval_us = 1'000'000.0f / static_cast<float>(std::max(1, request_rate_hz_));
    append_le_float(payload, static_cast<float>(msgid));
    append_le_float(payload, interval_us);
    for (int i = 0; i < 5; ++i) {
      append_le_float(payload, 0.0f);
    }
    append_le_u16(payload, kMavCmdSetMessageInterval);
    payload.push_back(target_system_);
    payload.push_back(target_component_);
    payload.push_back(0);
    return build_v1_packet(76, payload, 152);
  }

  void send_packet(const std::vector<uint8_t> & packet)
  {
    if (fd_ < 0) {
      return;
    }
    const auto written = ::write(fd_, packet.data(), packet.size());
    if (written < 0) {
      RCLCPP_WARN(get_logger(), "Serial write failed: %s", std::strerror(errno));
      close_serial();
    }
  }

  std::string port_;
  std::string cmd_vel_topic_;
  int baud_ = 115200;
  int linear_channel_ = 2;
  int lateral_channel_ = 4;
  int angular_channel_ = 1;
  int mode_channel_ = 7;
  int center_pwm_ = 1524;
  int min_pwm_ = 1102;
  int max_pwm_ = 1927;
  int deadband_pwm_ = 30;
  int mode_threshold_pwm_ = 1600;
  double max_linear_ = 1.0;
  double max_lateral_ = 1.0;
  double max_angular_ = 1.0;
  double linear_direction_ = -1.0;
  double lateral_direction_ = -1.0;
  double angular_direction_ = -1.0;
  double ackermann_min_linear_x_ = 0.05;
  double mode_toggle_debounce_sec_ = 0.3;
  double rc_timeout_sec_ = 0.5;
  double debug_log_interval_sec_ = 1.0;
  double debug_cmd_change_threshold_ = 0.02;
  bool mode_toggle_latch_ = true;
  bool initial_ackermann_mode_ = false;
  bool publish_zero_on_timeout_ = true;
  bool debug_output_ = false;
  int debug_pwm_change_threshold_ = 5;
  int request_rate_hz_ = 20;

  int fd_ = -1;
  uint8_t sequence_ = 0;
  uint8_t target_system_ = 1;
  uint8_t target_component_ = 0;
  std::vector<uint8_t> buffer_;
  bool zero_published_after_timeout_ = false;
  bool mode_input_initialized_ = false;
  bool last_mode_switch_high_ = false;
  bool last_mode_initialized_ = false;
  bool last_ackermann_mode_ = false;
  bool latched_ackermann_mode_ = false;
  uint16_t last_logged_linear_pwm_ = 0;
  uint16_t last_logged_lateral_pwm_ = 0;
  uint16_t last_logged_angular_pwm_ = 0;
  double last_logged_linear_x_ = 0.0;
  double last_logged_linear_y_ = 0.0;
  double last_logged_angular_z_ = 0.0;
  rclcpp::Time last_open_attempt_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_rc_time_;
  rclcpp::Time last_debug_time_;
  rclcpp::Time last_mode_toggle_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SbusCmdVelNode>());
  rclcpp::shutdown();
  return 0;
}
