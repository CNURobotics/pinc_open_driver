// Copyright 2025 Christopher Newport University - CNU Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pinc_open_driver/gripper.hpp"

#include <sys/ioctl.h>

#include <chrono>
#include <limits>
#include <numeric>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pinc_open_driver
{

hardware_interface::CallbackReturn PincOpenDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  RCLCPP_INFO(get_logger(), "on_init() called");

  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("serial_port")) {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  } else {
    RCLCPP_FATAL(get_logger(), "Serial port not specified!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Serial port = %s", serial_port_.c_str());

  servo_id_ = info_.hardware_parameters.count("servo_id")
                ? static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("servo_id")))
                : 1;
  RCLCPP_INFO(get_logger(), "Servo ID = %d", servo_id_);

  // Speed conversion (1 rev / 4096 steps) * (2pi rad/rev)
  rad_per_step_ = (2.0 * M_PI) / 4096;  // radians/step

  // make parameters
  baud_rate_termios_ = B1000000;  // Set for FeeTech servo
  if (info_.hardware_parameters.count("baud_rate")) {
    uint32_t baud_rate =
      static_cast<uint32_t>(std::stoi(info_.hardware_parameters.at("baud_rate")));
    RCLCPP_INFO(get_logger(), "Request serial baud rate = %u", baud_rate);
    switch (baud_rate) {
      case 9600:
        baud_rate_termios_ = B9600;
        break;
      case 57600:
        baud_rate_termios_ = B57600;
        break;
      case 115200:
        baud_rate_termios_ = B115200;
        break;
      case 1000000:
        baud_rate_termios_ = B1000000;
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown serial baud rate = %u", baud_rate);
        throw std::runtime_error("Unsupported baud for termios: " + std::to_string(baud_rate));
    }
  } else {
    RCLCPP_FATAL(get_logger(), "Baud rate not specified - using default 1000000!");
  }

  comm_config_delay_ms_ = 100;
  if (info_.hardware_parameters.count("comm_config_delay_ms")) {
    comm_config_delay_ms_ =
      static_cast<uint32_t>(std::stoi(info_.hardware_parameters.at("comm_config_delay_ms")));
  }
  RCLCPP_INFO(get_logger(), "Use %u for comm_config_delay_ms", comm_config_delay_ms_);

  read_timeout_ms_ = std::chrono::milliseconds(20);  // 50 Hz default rate
  if (info_.hardware_parameters.count("serial_timeout")) {
    read_timeout_ms_ = std::chrono::milliseconds(
      static_cast<uint32_t>(std::stoi(info_.hardware_parameters.at("serial_timeout"))));
  } else {
    RCLCPP_FATAL(get_logger(), "Timeout not specified!");
  }
  RCLCPP_INFO(get_logger(), "Serial timeout = %ld ms", read_timeout_ms_.count());

  hw_commands_pos.resize(info_.joints.size(), 0.0);
  hw_states_pos.resize(info_.joints.size(), 0.0);
  hw_states_vel.resize(info_.joints.size(), 0.0);

  joint_lower_.clear();
  joint_upper_.clear();

  for (const auto & joint : info_.joints) {
    RCLCPP_INFO(get_logger(), "Joint detected: %s", joint.name.c_str());

    bool has_position_cmd = false;
    bool has_position_state = false;
    bool has_velocity_state = false;

    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) has_position_cmd = true;
    }
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) has_position_state = true;
      if (si.name == hardware_interface::HW_IF_VELOCITY) has_velocity_state = true;
    }

    if (!has_position_cmd || !has_position_state) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' must have position command/state interfaces at minimum.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!has_velocity_state) {
      RCLCPP_INFO(get_logger(), "Only using position interface for %s", joint.name.c_str());
    }

    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();

    if (joint.command_interfaces[0].parameters.count("min"))
      lower = std::stod(joint.command_interfaces[0].parameters.at("min"));

    if (joint.command_interfaces[0].parameters.count("max"))
      upper = std::stod(joint.command_interfaces[0].parameters.at("max"));

    RCLCPP_INFO(get_logger(), "Joint '%s' limits: [%f, %f]", joint.name.c_str(), lower, upper);

    joint_lower_[joint.name] = lower;
    joint_upper_[joint.name] = upper;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PincOpenDriver::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring hardware...");
  std::fill(hw_states_pos.begin(), hw_states_pos.end(), 0.0);
  std::fill(hw_commands_pos.begin(), hw_commands_pos.end(), 0.0);
  std::fill(hw_states_vel.begin(), hw_states_vel.end(), 0.0);
  initialize_command_ = true;

  // Set up read request of position and velocity as 4-bytes from position address
  std::vector<uint8_t> posn_request = {0xFF, 0xFF, servo_id_, 0x04, 0x02, 0x38, 0x04};
  posn_request.push_back(~std::accumulate(posn_request.begin() + 2, posn_request.end(), 0) & 0xFF);
  std::vector<uint8_t> vel_request = {0xFF, 0xFF, servo_id_, 0x04, 0x02, 0x3a, 0x04};
  vel_request.push_back(~std::accumulate(vel_request.begin() + 2, vel_request.end(), 0) & 0xFF);
  tx_posn_request.clear();
  tx_posn_request.insert(tx_posn_request.end(), posn_request.begin(), posn_request.end());

  RCLCPP_INFO(get_logger(), "Opening serial port");

  SerialPort = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY);  // not using O_SYNC);
  if (SerialPort < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  int status = 0;
  int rc = ioctl(SerialPort, TIOCMGET, &status);
  if (rc != 0) {
    // This is not fatal because some interfaces (e.g. socat) do not support this action
    RCLCPP_WARN(
      get_logger(), "Unable to check serial port '%s' status: rc=%d err=%d (%s)!",
      serial_port_.c_str(), rc, errno, strerror(errno));
  } else {
    RCLCPP_INFO(
      get_logger(), "Initial TIOCM status for '%s' mask: 0x%08X", serial_port_.c_str(), status);
    RCLCPP_INFO(
      get_logger(), "    CTS=%d DSR=%d DCD=%d RI=%d DTR=%d RTS=%d", !!(status & TIOCM_CTS),
      !!(status & TIOCM_DSR), !!(status & TIOCM_CAR), !!(status & TIOCM_RI), !!(status & TIOCM_DTR),
      !!(status & TIOCM_RTS));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(comm_config_delay_ms_));

  termios tio{};
  if (tcgetattr(SerialPort, &tio) != 0) {
    RCLCPP_FATAL(get_logger(), "tcgetattr failed!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Start from raw to avoid hidden transformations
  cfmakeraw(&tio);

  // 8N1, no flow control
  tio.c_cflag &= ~CRTSCTS;            // no HW flow control
  tio.c_cflag |= (CLOCAL | CREAD);    // enable receiver, ignore modem ctrl
  tio.c_cflag &= ~(PARENB | CSTOPB);  // 8N1 (parity off, 1 stop)
  tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;

  // Make sure all input translations are off (cfmakeraw does most of this, but be explicit)
  tio.c_iflag &=
    ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR | PARMRK | ISTRIP | BRKINT | IGNBRK);

  // Output/local flags already cleared by cfmakeraw, but you can keep these for clarity
  tio.c_oflag = 0;
  tio.c_lflag = 0;

  // Set baud rate
  if (cfsetispeed(&tio, baud_rate_termios_) != 0 || cfsetospeed(&tio, baud_rate_termios_) != 0) {
    RCLCPP_FATAL(get_logger(), "cfset*speed failed errno=%d (%s)", errno, strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // READ behavior:
  // nonblocking-ish; read returns immediately if no data,
  // with VTIME=1, the kernel will wait up to 0.1s while assembling a packet
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;  // non-blocking zero wait
  tio.c_cflag &= ~HUPCL;
  if (tcsetattr(SerialPort, TCSAFLUSH, &tio) != 0) {
    RCLCPP_FATAL(get_logger(), "tcsetattr failed:");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(comm_config_delay_ms_));
  rc = ioctl(SerialPort, TIOCMGET, &status);
  if (rc != 0) {
    // This is not fatal because some interfaces (e.g. socat) do not support this action
    RCLCPP_WARN(
      get_logger(), "Unable to check serial port '%s' status rc=%d err=%d (%s)!",
      serial_port_.c_str(), rc, errno, strerror(errno));
  } else {
    RCLCPP_INFO(get_logger(), "TIOCM status for '%s' mask: 0x%08X", serial_port_.c_str(), status);
    RCLCPP_INFO(
      get_logger(), "    CTS=%d DSR=%d DCD=%d RI=%d DTR=%d RTS=%d", !!(status & TIOCM_CTS),
      !!(status & TIOCM_DSR), !!(status & TIOCM_CAR), !!(status & TIOCM_RI), !!(status & TIOCM_DTR),
      !!(status & TIOCM_RTS));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(comm_config_delay_ms_));

  // Start clean: drop any stale RX/TX bytes
  rc = tcflush(SerialPort, TCIOFLUSH);
  if (rc < 0) {
    RCLCPP_ERROR(get_logger(), "configure: tcflush rc=%d errno=%d %s", rc, errno, strerror(errno));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(comm_config_delay_ms_));

  setTorque(false);  // Disable torque until activated
  std::this_thread::sleep_for(std::chrono::milliseconds(comm_config_delay_ms_));

  // Tell servo not to respond to every command
  rc = sendInstruction(0x03, {0x08, 0});
  if (rc < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to turn off responses!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(comm_config_delay_ms_));

  // clean up any stray bits in buffer from prior echos
  rc = tcflush(SerialPort, TCIOFLUSH);
  if (rc < 0) {
    RCLCPP_ERROR(get_logger(), "configure: tcflush rc=%d errno=%d %s", rc, errno, strerror(errno));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(comm_config_delay_ms_));

  polling.fd = SerialPort;
  polling.events = POLLIN | POLLERR | POLLHUP | POLLNVAL;

  RCLCPP_INFO(get_logger(), "Successfully configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PincOpenDriver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating gripper hardware...");
  setTorque(true);  // Enable torque
  RCLCPP_INFO(get_logger(), "Gripper - torque mode activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PincOpenDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating gripper hardware  ...");
  if (SerialPort >= 0) {
    setTorque(false);
    initialize_command_ = true;
    RCLCPP_INFO(get_logger(), "Deactivated gripper torque mode");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PincOpenDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up hardware...");
  if (SerialPort >= 0) {
    RCLCPP_INFO(get_logger(), "Disabling torque...");
    setTorque(false);
    ::close(SerialPort);
    RCLCPP_INFO(get_logger(), "Closed gripper serial port");
  }
  SerialPort = -1;
  std::fill(hw_states_pos.begin(), hw_states_pos.end(), 0.0);
  std::fill(hw_states_vel.begin(), hw_states_vel.end(), 0.0);
  std::fill(hw_commands_pos.begin(), hw_commands_pos.end(), 0.0);
  tx_posn_request.clear();
  initialize_command_ = false;
  RCLCPP_INFO(get_logger(), "Hardware cleanup complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PincOpenDriver::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

// --- Interfaces ---
std::vector<hardware_interface::StateInterface> PincOpenDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    states.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos[i]);
    states.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> PincOpenDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> commands;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    commands.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_pos[i]);
  }
  return commands;
}
// --- Serial Commands ---
void PincOpenDriver::setTorque(bool on)
{
  if (sendInstruction(0x03, {0x28, static_cast<uint8_t>(on ? 1 : 0)}) < 0) {
    RCLCPP_WARN(get_logger(), "Failed to setTorque control %d!", on);
  }
}

int32_t PincOpenDriver::write_all(const std::vector<uint8_t> & packet)
{
  int written = 0;
  while (written < static_cast<int>(packet.size())) {
    int rc = ::write(SerialPort, packet.data() + written, packet.size() - written);
    if (rc == 0) {
      RCLCPP_WARN(get_logger(), "write_all unexpected rc=%d - delay and retry!", rc);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } else if (rc > 0) {
      written += rc;
    } else {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_WARN(get_logger(), "write_all error rc=%d errno=%d (%s)!", rc, errno, strerror(errno));
      return -1;
    }
  }
  return written;
}

int32_t PincOpenDriver::get_data(
  const std::vector<uint8_t> & tx_request, uint8_t & error, int32_t & raw_val)
{
  using clk = std::chrono::steady_clock;

  uint8_t tmp[64];
  bool request = true;
  auto elapsed = clk::now() - frame_start_;
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
  int32_t rc = 0;
  while (elapsed_ms < read_timeout_ms_) {
    if (request) {
      rx_buffer.clear();                   // clear existing data buffer before first request
      rc = tcflush(SerialPort, TCIFLUSH);  // flush receive buffers if error
      if (rc < 0) {
        RCLCPP_ERROR(
          get_logger(), "Flush serial before request error to 0x%02X  rc=%d", servo_id_, rc);
        return -1;
      }

      // Send request to read data
      rc = write_all(tx_request);
      if (rc < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to write read request to 0x%02X  rc=%d", servo_id_, rc);
        return -1;
      }
      rc = tcdrain(SerialPort);
      if (rc < 0) {
        RCLCPP_ERROR(
          get_logger(), "sendInstruction: tcdrain rc=%d errno=%d %s", rc, errno, strerror(errno));
      }
      request = false;
    }

    polling.revents = 0;          // clear prior
    rc = ::poll(&polling, 1, 1);  // 1 ms max
    if (rc == 0) {
      elapsed = clk::now() - frame_start_;
      elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
      if (elapsed_ms > read_timeout_ms_) {
        rc = tcflush(SerialPort, TCIOFLUSH);  // flush buffers if error
        RCLCPP_ERROR(
          get_logger(), "Serial port timeout with %d ms rx=%ld",
          static_cast<int>(elapsed_ms.count()), rx_buffer.size());
        return -1;
      }
      continue;  // normal condition while waiting
    } else if (rc < 0) {
      if (errno == EINTR) continue;  // system interrupt, try again
      RCLCPP_ERROR(get_logger(), "Polling error on serial port %d (%s)", errno, strerror(errno));
      return -1;  // real poll error
    }

    // polling event - double check for error from poll() before reading
    if (polling.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      RCLCPP_ERROR(get_logger(), "Polling revents error on serial port 0x%x", polling.revents);
      return -1;
    }
    if (!(polling.revents & POLLIN)) {
      continue;
    }

    // Read data
    rc = ::read(SerialPort, tmp, sizeof(tmp));
    if (rc > 0) {
      rx_buffer.insert(rx_buffer.end(), tmp, tmp + rc);
      if (rx_buffer.size() > 1024) {
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + (rx_buffer.size() - 512));
        RCLCPP_WARN(get_logger(), "RX buffer overflow, trimming...");
      }
    } else if (rc == 0) {
      elapsed = clk::now() - frame_start_;
      elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
      if (elapsed_ms > read_timeout_ms_) {
        rc = tcflush(SerialPort, TCIOFLUSH);  // flush buffers if error
        RCLCPP_ERROR(
          get_logger(), "Serial port timeout with %d ms rx=%ld",
          static_cast<int>(elapsed_ms.count()), rx_buffer.size());
        return -1;
      }
      continue;
    } else {
      RCLCPP_WARN(get_logger(), "RX buffer read=%d errno=%d rx=%ld", rc, errno, rx_buffer.size());
      if (errno == EIO || errno == ENODEV) {
        RCLCPP_ERROR(get_logger(), "Serial port – remote closed connection");
      } else {
        RCLCPP_ERROR(get_logger(), "Serial port read error: errno=%d (%s)", errno, strerror(errno));
      }
      return -1;
    }

    while (rx_buffer.size() >= 6 && read_state_ != static_cast<uint8_t>(ReadState::DONE)) {
      // minimum packet length: header(2)+id+len+error+2data+checksum
      if (!(rx_buffer[0] == 0xFF && rx_buffer[1] == 0xFF)) {
        // Find beginning of header
        rx_buffer.erase(rx_buffer.begin());
        continue;  // keep going one at a time until we find a header start
      }

      // Extract a 4-byte packet for all requested data
      uint8_t id = rx_buffer[2];
      uint8_t length = rx_buffer[3];
      if (length < 2 || length > 32) {  // pick a sane bound for length
        request = true;                 // request our data again as it messed up in transit
        break;
      }
      if (rx_buffer.size() < 4UL + length) {
        break;  // wait for full packet, so loop some more
      }
      std::vector<uint8_t> packet(rx_buffer.begin(), rx_buffer.begin() + 4 + length);

      uint8_t checksum_calc =
        ~(id + length + std::accumulate(packet.begin() + 4, packet.end() - 1, 0)) & 0xFF;
      if (checksum_calc != packet.back()) {
        rx_buffer.erase(rx_buffer.begin());
        if (length > 4) {
          request = true;  // request our data again as it was messed up in transit
          break;
        }
        continue;
      }
      rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 4 + length);

      if (packet[4] != 0) {
        error |= packet[4];
        RCLCPP_WARN(get_logger(), "Servo error byte: 0x%x", packet[4]);
        // Dynamixel protocol v.1 error codes
        // Bit 6 Instruction Error   In case of sending an undefined instruction or
        // delivering the action instruction without the
        // Reg Write instruction, it is set as 1
        // Bit 5 Overload Error      When the current load
        // cannot be controlled by the set Torque, it is set as 1
        // Bit 4 Checksum Error      When the Checksum of the transmitted I
        // nstruction Packet is incorrect, it is set as 1
        // Bit 3 Range Error         When an instruction is out of
        // the range for use, it is set as 1
        // Bit 2 Overheating Error   When internal temperature of DYNAMIXEL is
        // out of the range of operating temperature set in
        // the Control table, it is set as 1
        // Bit 1 Angle Limit Error   When Goal Position is written out of
        // the range from CW Angle Limit to CCW Angle Limit , it is set as 1
        // Bit 0 Input Voltage Error When the applied voltage is out of the
        // range of operating voltage set in the Control table, it is as 1
      }

      if (length < 4) {
        RCLCPP_INFO(get_logger(), "In read, rcv not a data message, length = %d ", length);
        continue;  // Not a data message
      }

      raw_val = (packet[8] << 24) | (packet[7] << 16) | (packet[6] << 8) | packet[5];

      return 0;
    }

    // We read something but not enough
    elapsed = clk::now() - frame_start_;
    elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
    RCLCPP_INFO(
      get_logger(), "Servo %d ms r=%3d size=%ld error=0x%x - loop for more data",
      static_cast<int>(elapsed_ms.count()), rc, rx_buffer.size(), error);
  }

  tcflush(SerialPort, TCIFLUSH);  // flush receive buffers before request

  RCLCPP_INFO(
    get_logger(), "Servo %d ms size=%ld error=0x%x - failed to read data properly",
    static_cast<int>(elapsed_ms.count()), rx_buffer.size(), error);
  return -1;
}

hardware_interface::return_type PincOpenDriver::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (hw_states_pos.empty() || hw_states_vel.empty()) return hardware_interface::return_type::OK;

  uint8_t error = 0;
  int32_t raw_value = 0;

  read_state_ = static_cast<uint8_t>(ReadState::EMPTY);
  using clk = std::chrono::steady_clock;
  frame_start_ = clk::now();
  hardware_interface::return_type rc = hardware_interface::return_type::OK;  // assume OK
  int32_t success = get_data(tx_posn_request, error, raw_value);
  if (success < 0) {
    rc = hardware_interface::return_type::ERROR;
  } else {
    // Valid frame

    // Access position data
    double radians = ((max_angle_ * static_cast<float>(raw_value & 0x0000FFFF)) / (max_position_)) -
                     (max_angle_ / 2);
    hw_states_pos[0] = radians;
    read_state_ |= static_cast<uint8_t>(ReadState::GOT_POS);
    if (initialize_command_) {
      // Initialize command to match current position on startup
      hw_commands_pos[0] = radians;
      initialize_command_ = false;
    }

    raw_value = (raw_value >> 16) & 0xFFFF;  // shift to access velocity data
    // Valid frame
    int direction = (raw_value & 0x8000) ? -1 : 1;  // bit 16 = direction
    int speed = raw_value & 0x7FFF;                 // lower 15 bits
    double rad_per_sec = direction * (speed * rad_per_step_);
    hw_states_vel[0] = rad_per_sec;
    read_state_ |= static_cast<uint8_t>(ReadState::GOT_VEL);
  }

  if (error) {
    RCLCPP_WARN(get_logger(), "Final Servo reading error byte: 0x%x !", error);
    // Dynamixel protocol v.1 error codes
    // Bit 6 Instruction Error   In case of sending an undefined instruction or
    // delivering the action instruction without the Reg Write instruction, it is set as 1
    // Bit 5 Overload Error      When the current load
    // cannot be controlled by the set Torque, it is set as 1
    // Bit 4 Checksum Error      When the Checksum of the transmitted I
    // nstruction Packet is incorrect, it is set as 1
    // Bit 3 Range Error         When an instruction is out of
    // the range for use, it is set as 1
    // Bit 2 Overheating Error   When internal temperature of DYNAMIXEL is
    // out of the range of operating temperature set in the Control table, it is set as 1
    // Bit 1 Angle Limit Error   When Goal Position is written out of
    // the range from CW Angle Limit to CCW Angle Limit , it is set as 1
    // Bit 0 Input Voltage Error When the applied voltage is out of the
    // range of operating voltage set in the Control table, it is as 1
  }
  auto elapsed = clk::now() - frame_start_;
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

  return rc;
}

hardware_interface::return_type PincOpenDriver::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (hw_commands_pos.empty()) return hardware_interface::return_type::ERROR;

  // --- Clamp and convert position command ---
  double target_rads = std::clamp(
    hw_commands_pos[0], joint_lower_[info_.joints[0].name], joint_upper_[info_.joints[0].name]);
  int raw_pos = 4095 * ((M_PI + target_rads) / (2 * M_PI));
  raw_pos = std::clamp(raw_pos, 0, 4095);

  // --- Build Dynamixel packet for position write ---
  std::vector<uint8_t> pos_packet = {
    0xFF,
    0xFF,
    servo_id_,
    0x05,
    0x03,
    0x2A,
    static_cast<uint8_t>(raw_pos & 0xFF),
    static_cast<uint8_t>((raw_pos >> 8) & 0xFF)};
  pos_packet.push_back(
    static_cast<uint8_t>(~std::accumulate(pos_packet.begin() + 2, pos_packet.end(), 0) & 0xFF));
  int rc = write_all(pos_packet);
  if (rc < 0) {
    RCLCPP_WARN(get_logger(), "Servo write error rc=%d!", rc);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

int32_t PincOpenDriver::sendInstruction(uint8_t instr, const std::vector<uint8_t> & params)
{
  errno = 0;
  std::vector<uint8_t> packet = {
    0xFF, 0xFF, servo_id_, static_cast<uint8_t>(params.size() + 2), instr};
  packet.insert(packet.end(), params.begin(), params.end());
  packet.push_back(
    static_cast<uint8_t>(~std::accumulate(packet.begin() + 2, packet.end(), 0) & 0xFF));

  int rc = write_all(packet);
  if (rc < 0) {
    RCLCPP_WARN(get_logger(), "sendInstruction write error rc=%d!", rc);
  }
  return rc;
}

}  // namespace pinc_open_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pinc_open_driver::PincOpenDriver, hardware_interface::SystemInterface)
