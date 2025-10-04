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

#ifndef PINC_OPEN_DRIVER__GRIPPER_HPP_
#define PINC_OPEN_DRIVER__GRIPPER_HPP_
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <memory>
#include <string>
#include <vector>
#include <numeric>
#include <cmath>
#include <unordered_map>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pinc_open_driver
{

class PincOpenDriver : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PincOpenDriver)

  // Lifecycle callbacks
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Additional functions
  void setTorque(bool on);  // Enable/disable servo torque

  void sendInstruction(uint8_t instr, const std::vector<uint8_t> & params);


private:
  // Get 2-byte data packet and return raw value
  // or -1 if invalid
  int32_t get_data(const std::vector<uint8_t>& tx_request, uint8_t& error);
  // Hardware parameters
  std::vector<double> hw_states_pos;  // Joint positions (radians)
  std::vector<double> hw_states_vel;
  std::vector<double> hw_states_eff;
  std::vector<double> hw_commands_pos;    // Joint commands (radians)
  bool initialize_command_;
  // Comms setup
  uint8_t id_ = 6;                    // Servo ID
  std::string port_;
  int SerialPort{-1};
  std::vector<uint8_t> tx_posn_request;
  std::vector<uint8_t> tx_vel_request;
  std::vector<uint8_t> tx_load_request;
  std::vector<uint8_t> rx_buffer;
  speed_t baud_rate_termios_;  // Set for FeeTech servo
  std::chrono::milliseconds read_timeout_ms_;
  std::chrono::time_point<std::chrono::steady_clock> frame_start_;
  bool sim_mode_ = false;
  double max_angle_ = 2*M_PI;
  double max_position_ = 4095.;
  double rad_per_step_ = (2*M_PI/4096.);
  double max_speed_ = 32767.;
  std::unordered_map<std::string, double> joint_lower_;
  std::unordered_map<std::string, double> joint_upper_;
  bool pos_read_done_ = false;
  bool second_request_done_ = false;
  enum class ReadState {EMPTY = 0, GOT_POS = 0x1, GOT_VEL = 0x2, DONE = 0xF };
  uint8_t read_state_ = static_cast<uint8_t>(ReadState::EMPTY);
};

}  // namespace pinc_open_driver
#endif  // PINC_OPEN_DRIVER__GRIPPER_HPP_
