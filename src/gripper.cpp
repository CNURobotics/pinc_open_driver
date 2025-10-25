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

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <numbers>
#include <chrono>
#include <numeric>
#include <limits>
#include "pinc_open_driver/gripper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace pinc_open_driver
{

hardware_interface::CallbackReturn PincOpenDriver::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    RCLCPP_INFO(get_logger(), "on_init() called");

    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS){
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.hardware_parameters.count("port")){
        port_ = info_.hardware_parameters.at("port");
    }else{
        RCLCPP_FATAL(get_logger(), "Serial port not specified!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Serial port = %s", port_.c_str());

    id_ = info_.hardware_parameters.count("id") ?
          static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("id"))) : 1;
    RCLCPP_INFO(get_logger(), "Motor ID = %d", id_);

    // Speed conversion (1 rev / 4096 steps) * (2pi rad/rev)
    rad_per_step_ = (2.0*M_PI)/4096;  // radians/step

    // make parameters
    baud_rate_termios_ = B1000000;  // Set for FeeTech servo
    read_timeout_ms_ = std::chrono::milliseconds(50);

    hw_commands_pos.resize(info_.joints.size(), 0.0);
    hw_states_pos.resize(info_.joints.size(), 0.0);
    hw_states_vel.resize(info_.joints.size(), 0.0);

    joint_lower_.clear();
    joint_upper_.clear();

    for (const auto & joint : info_.joints) {
        RCLCPP_INFO(get_logger(), "Joint detected: %s", joint.name.c_str());

        bool has_position_cmd = false;
        bool has_position_state = false;

        for (const auto & ci : joint.command_interfaces) {
            if (ci.name == hardware_interface::HW_IF_POSITION) has_position_cmd = true;
        }
        for (const auto & si : joint.state_interfaces) {
            if (si.name == hardware_interface::HW_IF_POSITION) has_position_state = true;
            if (si.name == hardware_interface::HW_IF_VELOCITY) has_position_state = true;
        }

        if (!has_position_cmd || !has_position_state) {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' must have position + velocity command/state interfaces.",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
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


    // Set up read requests
    std::vector<uint8_t> posn_request = {0xFF, 0xFF, id_, 0x04, 0x02, 0x38, 0x04};
    posn_request.push_back(~std::accumulate(posn_request.begin() + 2,
        posn_request.end(), 0) & 0xFF);
    std::vector<uint8_t> vel_request = {0xFF, 0xFF, id_, 0x04, 0x02, 0x3a, 0x04};
    vel_request.push_back(~std::accumulate(vel_request.begin() + 2, vel_request.end(), 0) & 0xFF);
    tx_posn_request.clear();
    tx_vel_request.clear();
    tx_posn_request.insert(tx_posn_request.end(), posn_request.begin(), posn_request.end());
    tx_vel_request.insert(tx_vel_request.end(), vel_request.begin(),  vel_request.end());

    RCLCPP_INFO(get_logger(), "Successfully configured");

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn PincOpenDriver::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating hardware...");


    SerialPort = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);  // not using O_SYNC);
    if (SerialPort < 0) {
        RCLCPP_FATAL(get_logger(), "Failed to open serial port %s", port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    termios tio{};
    if (tcgetattr(SerialPort, &tio) != 0) {
        RCLCPP_FATAL(get_logger(), "tcgetattr failed!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Start from raw to avoid hidden transformations
    cfmakeraw(&tio);

    // 8N1, no flow control
    tio.c_cflag &= ~CRTSCTS;               // no HW flow control
    tio.c_cflag |= (CLOCAL | CREAD);       // enable receiver, ignore modem ctrl
    tio.c_cflag &= ~(PARENB | CSTOPB);     // 8N1 (parity off, 1 stop)
    tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;

    // Make sure all input translations are off (cfmakeraw does most of this, but be explicit)
    tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR
        | IGNCR | PARMRK | ISTRIP | BRKINT | IGNBRK);

    // Output/local flags already cleared by cfmakeraw, but you can keep these for clarity
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    // Baud
    cfsetispeed(&tio, baud_rate_termios_);
    cfsetospeed(&tio, baud_rate_termios_);

    // READ behavior:
    // nonblocking-ish; read returns immediately if no data,
    // with VTIME=1, the kernel will wait up to 0.1s while
    // assembling a packet: good for state machines.
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;  // non-blocking zero wait

    if (tcsetattr(SerialPort, TCSANOW, &tio) != 0) {
        RCLCPP_FATAL(get_logger(), "tcsetattr failed:");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Start clean: drop any stale RX/TX bytes
    tcflush(SerialPort, TCIOFLUSH);

    // Enable torque
    setTorque(true);
    tcdrain(SerialPort);  // flush write

    // Ensure servo responds
    sendInstruction(0x03, {0x08, 0});
    tcdrain(SerialPort);

    RCLCPP_INFO(get_logger(), "Hardware activated successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PincOpenDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating hardware...");
    if (SerialPort >= 0) {
        setTorque(false);
        tcdrain(SerialPort);
        close(SerialPort);
        SerialPort = -1;
        initialize_command_ = true;
        RCLCPP_INFO(get_logger(), "Serial port closed, torque disabled");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

// --- Interfaces ---
std::vector<hardware_interface::StateInterface> PincOpenDriver::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> states;
    for (size_t i = 0; i < info_.joints.size(); ++i){
        states.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_pos[i]);
        states.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_vel[i]);
    }
    return states;
}

std::vector<hardware_interface::CommandInterface> PincOpenDriver::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> commands;
    for (size_t i = 0; i < info_.joints.size(); ++i){
        commands.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_pos[i]);
    }
    return commands;
}
// --- Serial Commands ---
void PincOpenDriver::setTorque(bool on)
{
    sendInstruction(0x03, {0x28, static_cast<uint8_t>(on ? 1 : 0)});
    tcdrain(SerialPort);
}

int32_t PincOpenDriver::get_data(const std::vector<uint8_t>& tx_request, uint8_t& error) {
    using clk = std::chrono::steady_clock;

    uint8_t tmp[64];
    bool request = true;
    auto elapsed = clk::now() - frame_start_;
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
    while (elapsed_ms < read_timeout_ms_ ) {
        if (request) {
            // Send request to read data
            if (::write(SerialPort, tx_request.data(), tx_request.size())
                != (ssize_t)tx_request.size()) {
                RCLCPP_ERROR(get_logger(), "Failed to write read request to 0x%02X", id_);
                return -1;
            }
            tcdrain(SerialPort);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            request = false;
        }


        int r = ::read(SerialPort, tmp, sizeof(tmp));
        if (r > 0) {
            rx_buffer.insert(rx_buffer.end(), tmp, tmp + r);
            if (rx_buffer.size() > 1024) {
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + (rx_buffer.size() - 512));
                RCLCPP_WARN(get_logger(), "RX buffer overflow, trimming...");
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            elapsed = clk::now() - frame_start_;
            elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
            if ( elapsed_ms > read_timeout_ms_) {
                tcflush(SerialPort, TCIOFLUSH);  // flush buffers if error
                RCLCPP_WARN(get_logger(), "Timeout with %d ms rx=%ld",
                    static_cast<int>(elapsed_ms.count()), rx_buffer.size());
                return -1;
            }
            continue;
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
            uint8_t length = std::min<uint8_t>(6, rx_buffer[3]);
            if (rx_buffer.size() < 4UL + length) {
                break;  // wait for full packet, so loop some more
            }
            std::vector<uint8_t> packet(rx_buffer.begin(), rx_buffer.begin() + 4 + length);


            uint8_t checksum_calc = ~(id + length + std::accumulate(packet.begin() + 4,
                packet.end() - 1, 0)) & 0xFF;
            if (checksum_calc != packet.back()) {
                rx_buffer.erase(rx_buffer.begin());
                if (length > 4) {
                    request = true;  // request our data again as it was messed up in transit
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
                continue;  // Not a data message
            }

            int32_t raw_value = (packet[6] << 8) | packet[5];

            return  raw_value;
        }

        // We read something but not enough
        elapsed = clk::now() - frame_start_;
        elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        RCLCPP_INFO(get_logger(), "Servo %d ms r=%3d size=%ld error=0x%x - loop for more data",
                            static_cast<int>(elapsed_ms.count()),
                            r, rx_buffer.size(), error);
    }

    tcflush(SerialPort, TCIOFLUSH);  // flush buffers before request

    RCLCPP_INFO(get_logger(), "Servo %d ms size=%ld error=0x%x - failed to read data properly",
                        static_cast<int>(elapsed_ms.count()),
                        rx_buffer.size(), error);
    return -1;
}
hardware_interface::return_type PincOpenDriver::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    if (hw_states_pos.empty() || hw_states_vel.empty())
        return hardware_interface::return_type::OK;

    uint8_t error = 0;


    read_state_ = static_cast<uint8_t>(ReadState::EMPTY);
    using clk = std::chrono::steady_clock;
    frame_start_ = clk::now();
    hardware_interface::return_type rc = hardware_interface::return_type::OK;  // assume OK
    int32_t raw_value = get_data(tx_posn_request, error);
    if (raw_value < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to get position data ...");
        rc = hardware_interface::return_type::ERROR;
    } else {
        // Valid frame
        double radians =((max_angle_ * static_cast<float>(raw_value))
            / (max_position_)) - (max_angle_/ 2);
        hw_states_pos[0] = radians;
        read_state_ |= static_cast<uint8_t>(ReadState::GOT_POS);
        if (initialize_command_) {
            // Initialize command to match current position on startup
            hw_commands_pos[0] = radians;
            initialize_command_ = false;
        }
   }
    raw_value = get_data(tx_vel_request, error);
    if (raw_value < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to get velocity data ...");
        rc = hardware_interface::return_type::ERROR;
    } else {
        // Valid frame
        int direction = (raw_value & 0x8000) ? -1 : 1;  // bit 10 = direction
        int speed = raw_value & 0x7FFF;                // lower 10 bits
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
    if (hw_commands_pos.empty())
        return hardware_interface::return_type::ERROR;

    // --- Clamp and convert position command ---
    double target_rads = std::clamp(hw_commands_pos[0],
                                    joint_lower_[info_.joints[0].name], 
                                    joint_upper_[info_.joints[0].name]);
    int raw_pos = 4095 * ((M_PI + target_rads) / (2*M_PI));
    raw_pos = std::clamp(raw_pos, 0, 4095);

    // --- Build Dynamixel packet for position write ---
    std::vector<uint8_t> pos_packet = {
        0xFF, 0xFF, id_, 0x05, 0x03, 0x2A,
        static_cast<uint8_t>(raw_pos & 0xFF),
        static_cast<uint8_t>((raw_pos >> 8) & 0xFF)
    };
    pos_packet.push_back(static_cast<uint8_t>(~std::accumulate(pos_packet.begin()+2,
        pos_packet.end(), 0) & 0xFF));
    if (::write(SerialPort, pos_packet.data(), pos_packet.size()) != (ssize_t)pos_packet.size())
        return hardware_interface::return_type::ERROR;
    tcdrain(SerialPort);
    return hardware_interface::return_type::OK;
}



void PincOpenDriver::sendInstruction(uint8_t instr, const std::vector<uint8_t>& params)
{
    std::vector<uint8_t> packet = {0xFF, 0xFF, id_, static_cast<uint8_t>(params.size() + 2), instr};
    packet.insert(packet.end(), params.begin(), params.end());
    packet.push_back(static_cast<uint8_t>(~std::accumulate(packet.begin() + 2,
        packet.end(), 0) & 0xFF));

    ::write(SerialPort, packet.data(), packet.size());
    tcdrain(SerialPort);
}

}  // namespace pinc_open_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    pinc_open_driver::PincOpenDriver,
    hardware_interface::SystemInterface)
