
#include "meca500_hardware/meca500_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
 
// Linux socket headers (for TCP communication with robot)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
 
#include <cmath>
#include <sstream>
#include <string>

namespace meca500_hardware
{
//================================================================================
// Helper function to connect a TCP socket to ip:port, returns fd or -1 on failure
//================================================================================
    int Meca500System::connect_socket(
        const std::string & ip, int port, int timeout_sec){
        // Create socket of type IPv4 (AF_INET) and stream (SOCK_STREAM)
        // 0 is the default protocol for this type of connecion (TCP for stream sockets)
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0){
            RCLCPP_WARN(rclcpp::get_logger("Meca500System"),
            "Could not create socket. Running in fake mode.");
            return -1;              
        }
        // Set connection timeout
        struct timeval timeout;
        timeout.tv_sec = timeout_sec; // seconds
        timeout.tv_usec = 0; // microseconds
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        // Try to connect
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port); //htons => host to network byte order for short (port)
        inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr); // Convert IP string to binary form
        int result = connect(fd, (struct sockaddr *)&server_addr, sizeof(server_addr));

        if (result < 0){
            RCLCPP_WARN(rclcpp::get_logger("Meca500System"),
              "Could not connect to %s:%d. Running in fake mode.",
                ip.c_str(), port);
                close(fd);
                return -1;
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("Meca500System"),
              "Connected to robot at %s:%d",
              ip.c_str(), port);
                return fd;
        }

    }
//================================================================================
// Export state and command interfaces to the ros2_control framework.
// These give the framework direct pointers into hw_positions_, hw_velocities_,
// and hw_commands_ so joint_state_broadcaster can publish real values.
//================================================================================
    std::vector<hardware_interface::StateInterface>
    Meca500System::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;
        for (size_t i = 0; i < NUM_JOINTS; ++i) {
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_positions_[i]);
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_velocities_[i]);
        }
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    Meca500System::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (size_t i = 0; i < NUM_JOINTS; ++i) {
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_commands_[i]);
        }
        return interfaces;
    }

//================================================================================
//                            On Init
//================================================================================
  
    hardware_interface::CallbackReturn Meca500System::on_init(
      const hardware_interface::HardwareInfo & info){

        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
          return CallbackReturn::ERROR;
        }

        // Read params
        robot_ip = info_.hardware_parameters.at("robot_ip");
        control_port = std::stoi(info_.hardware_parameters.at("control_port"));
        monitoring_port = std::stoi(info_.hardware_parameters.at("monitoring_port"));

        // Check that we have 6 joints
        if (info_.joints.size() != 6){
          RCLCPP_FATAL(get_logger(), "Expected 6 joints, got %zu", info_.joints.size());
          return CallbackReturn::ERROR;
        }

        // Validate each joint has position command + position/velocity state
        for (const auto & joint : info_.joints)
        {
          if (joint.command_interfaces.size() != 1 ||
              joint.command_interfaces[0].name != "position"){
            RCLCPP_FATAL(get_logger(),
              "Joint '%s' needs 1 position command interface",
              joint.name.c_str());
            return CallbackReturn::ERROR;
          }

          if (joint.state_interfaces.size() != 2){
            RCLCPP_FATAL(get_logger(),
              "Joint '%s' needs 2 state interfaces (position, velocity)",
              joint.name.c_str());
            return CallbackReturn::ERROR;
          }
        }

        // Resize vectors for 6 joints
        hw_positions_.resize(6, 0.0);
        hw_velocities_.resize(6, 0.0);
        hw_commands_.resize(6, 0.0);

        // Load joint offsets from hardware parameters (set via joint_offsets.yaml)
        const std::string keys[NUM_JOINTS] = {
            "joint1_offset_deg", "joint2_offset_deg", "joint3_offset_deg",
            "joint4_offset_deg", "joint5_offset_deg", "joint6_offset_deg"
        };
        for (size_t i = 0; i < NUM_JOINTS; ++i) {
            auto it = info_.hardware_parameters.find(keys[i]);
            if (it != info_.hardware_parameters.end()) {
                joint_offsets_deg_.at(i) = std::stod(it->second);
            } else {
                joint_offsets_deg_.at(i) = 0.0;
                RCLCPP_WARN(get_logger(), "%s not found in hardware params, defaulting to 0", keys[i].c_str());
            }
        }
        RCLCPP_INFO(get_logger(), "Joint offsets (deg): %.3f %.3f %.3f %.3f %.3f %.3f",
            joint_offsets_deg_.at(0), joint_offsets_deg_.at(1), joint_offsets_deg_.at(2),
            joint_offsets_deg_.at(3), joint_offsets_deg_.at(4), joint_offsets_deg_.at(5));

        return CallbackReturn::SUCCESS;
    }

    //================================================================================
    //                            On Configure
    //================================================================================

    hardware_interface::CallbackReturn Meca500System::on_configure(
      const rclcpp_lifecycle::State & previous_state){

        RCLCPP_INFO(rclcpp::get_logger("Meca500System"),
            "on_configure: attempting connection...");

        control_fd = connect_socket(robot_ip, control_port, 5);
        monitoring_fd = connect_socket(robot_ip, monitoring_port, 5);

        if (control_fd < 0 || monitoring_fd < 0){
            use_fake_ = true;
            if (control_fd >= 0) close(control_fd);
            if (monitoring_fd >= 0) close(monitoring_fd);
            control_fd = -1;
            monitoring_fd = -1;
        }
        else{
            use_fake_ = false;
                
          // Robot sends: [3000][Connected to Meca500 R3 v8.x.x.] message once connected. Read and print it.
          // the below code is to display that

          char buffer[256] = {0};
          recv(control_fd, buffer, sizeof(buffer) - 1, 0);
          RCLCPP_INFO(rclcpp::get_logger("Meca500System"),
            "Robot says: %s", buffer);

          // Enable real-time joint position monitoring at 15 ms intervals.
          // Firmware v11+ uses SetMonitoringInterval(seconds); older used SetRTCMonitoringInterval(ms).
          std::string rtc_cmd = "SetMonitoringInterval(0.015)";
          send(control_fd, rtc_cmd.c_str(), rtc_cmd.size() + 1, 0);
          char rtc_buf[256] = {0};
          recv(control_fd, rtc_buf, sizeof(rtc_buf) - 1, 0);
          RCLCPP_INFO(rclcpp::get_logger("Meca500System"),
            "SetMonitoringInterval response: %s", rtc_buf);
        }
     
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    //================================================================================
    //                            On Activate
    //================================================================================

    hardware_interface::CallbackReturn Meca500System::on_activate(
        const rclcpp_lifecycle::State & previous_state){

            RCLCPP_INFO(rclcpp::get_logger("Meca500System"), "on_activate called");
 
            // Set command to current position so robot doesn't jump on startup
            hw_commands_ = hw_positions_;
 
            if (use_fake_){
              RCLCPP_INFO(rclcpp::get_logger("Meca500System"),
                "Fake mode: robot 'activated' and 'homed'");
              return hardware_interface::CallbackReturn::SUCCESS;
            }
           
            // Send ActivateRobot, wait for [2000][Motors activated.]
            std::string cmd = "ActivateRobot";
            send(control_fd, cmd.c_str(), cmd.size() + 1, 0);
            char buf[256] = {0};
            recv(control_fd, buf, sizeof(buf) - 1, 0);
            RCLCPP_INFO(rclcpp::get_logger("Meca500System"), 
              "ActivateRobot response: %s", buf);
           
            // Send Home, wait for [2002][Homing done.]
            cmd = "Home";
            memset(buf, 0, sizeof(buf));
            send(control_fd, cmd.c_str(), cmd.size() + 1, 0);
            recv(control_fd, buf, sizeof(buf) - 1, 0);
            RCLCPP_INFO(rclcpp::get_logger("Meca500System"),
              "Home response: %s", buf);

            // Drain any stale messages before GetJoints.
            {
                char drain[256];
                while (recv(control_fd, drain, sizeof(drain), MSG_DONTWAIT) > 0) {}
            }

            // GetJoints: seed hw_positions_ with actual robot position so the
            // controller's first write() cycle doesn't command an extreme position.
            cmd = "GetJoints";
            send(control_fd, cmd.c_str(), cmd.size() + 1, 0);
            char jbuf[512] = {0};
            //jresp accumulates partial socket reads.
            std::string jresp;
            // attempt 5 reads
            for (int attempt = 0; attempt < 5; ++attempt) {
                // memset basically sets all bytes of the data to a given value
                // in this case: 0, to clear the buffer before each recv call
                memset(jbuf, 0, sizeof(jbuf));
                int n = recv(control_fd, jbuf, sizeof(jbuf) - 1, 0);
                if (n > 0) {
                    jresp += std::string(jbuf, n);
                    if (jresp.find("[2026]") != std::string::npos) break;
                }
            }
            size_t jpos = jresp.rfind("[2026]");
            if (jpos != std::string::npos) {
                size_t ds = jresp.find('[', jpos + 6);
                size_t de = jresp.find(']', ds);
                if (ds != std::string::npos && de != std::string::npos) {
                    std::string data = jresp.substr(ds + 1, de - ds - 1);
                    try {
                        std::vector<std::string> fields;
                        std::stringstream ss(data);
                        std::string token;
                        while (std::getline(ss, token, ',')) fields.push_back(token);
                        size_t start = (fields.size() >= NUM_JOINTS + 1) ? 1 : 0;
                        if (fields.size() >= start + NUM_JOINTS) {
                            for (size_t i = 0; i < NUM_JOINTS; ++i) {
                                hw_positions_.at(i) =
                                    (joint_offsets_deg_[i] - std::stod(fields[start + i])) * M_PI / 180.0;
                            }
                            hw_commands_ = hw_positions_;
                            RCLCPP_INFO(rclcpp::get_logger("Meca500System"),
                                "Initial joints (deg): %.3f %.3f %.3f %.3f %.3f %.3f",
                                hw_positions_.at(0)*180/M_PI, hw_positions_.at(1)*180/M_PI,
                                hw_positions_.at(2)*180/M_PI, hw_positions_.at(3)*180/M_PI,
                                hw_positions_.at(4)*180/M_PI, hw_positions_.at(5)*180/M_PI);
                        }
                    } catch (const std::exception & e) {
                        RCLCPP_WARN(rclcpp::get_logger("Meca500System"),
                            "GetJoints parse failed: %s — starting at zero", e.what());
                    }
                }
            }

            return hardware_interface::CallbackReturn::SUCCESS;
    }

    //================================================================================
    //                            Read
    //================================================================================

    
    hardware_interface::return_type Meca500System::read(
      const rclcpp::Time & time, const rclcpp::Duration & period){
        
        if (use_fake_){
            std::vector<double> old_positions = hw_positions_;
            hw_positions_ = hw_commands_;

            if (period.seconds() > 0.0){
                for (size_t i = 0; i < hw_positions_.size(); ++i){
                    hw_velocities_.at(i) = (hw_positions_.at(i) - old_positions.at(i)) / period.seconds();
                }
            }
            return hardware_interface::return_type::OK;
        }

        // Real robot: read joint positions from monitoring port
        // The Meca500 monitoring port (10001) sends joint data every ~15ms
        // Format: [2026][timestamp, j1, j2, j3, j4, j5, j6] in degrees

        std::vector<double> old_positions = hw_positions_;

        char buf[512] = {0};
        int bytes = recv(monitoring_fd, buf, sizeof(buf) - 1, MSG_DONTWAIT);
        
        if (bytes > 0) {
            std::string response(buf, bytes);
        
            // Find LAST [2026]
            size_t pos = std::string::npos;
            size_t search_from = 0;
        
            while (true) {
                size_t found = response.find("[2026]", search_from);
                if (found == std::string::npos) break;
                pos = found;
                search_from = found + 1;
            }
        
            if (pos != std::string::npos) {
        
                size_t data_start = response.find('[', pos + 6);
                size_t data_end   = response.find(']', data_start);
        
                if (data_start != std::string::npos && data_end != std::string::npos) {
        
                    std::string data = response.substr(
                        data_start + 1,
                        data_end - data_start - 1
                    );
        
                    try {
                        // Split all comma-separated fields
                        std::vector<std::string> fields;
                        std::stringstream ss(data);
                        std::string token;
                        while (std::getline(ss, token, ',')) {
                            fields.push_back(token);
                        }

                        // Accept 6 fields (no timestamp) or 7+ fields (timestamp first)
                        size_t start = (fields.size() >= NUM_JOINTS + 1) ? 1 : 0;
                        if (fields.size() < start + NUM_JOINTS) {
                            throw std::runtime_error(
                                "expected " + std::to_string(NUM_JOINTS) +
                                " joint fields, got raw data: [" + data + "]");
                        }

                        for (size_t i = 0; i < NUM_JOINTS; ++i) {
                            hw_positions_.at(i) =
                                (joint_offsets_deg_[i] - std::stod(fields[start + i])) * M_PI / 180.0;
                        }

                    } catch (const std::exception & e) {
                        RCLCPP_WARN(
                            rclcpp::get_logger("Meca500System"),
                            "Failed to parse [2026] joint positions: %s",
                            e.what()
                        );
                    }
                }
            }
        }
        // Compute velocities from position change
        if (period.seconds() > 0.0) {
            for (size_t i = 0; i < 6; ++i) {
                hw_velocities_.at(i) = 
                (hw_positions_.at(i) - old_positions.at(i)) / period.seconds();
            }
        }

        return hardware_interface::return_type::OK;
    }

    //================================================================================
    //                            Write
    //================================================================================

    hardware_interface::return_type Meca500System::write(
      const rclcpp::Time & time, const rclcpp::Duration & period){

        if (use_fake_){
            return hardware_interface::return_type::OK;
        }
 
        std::vector<double> joints_deg(6, 0.0);
        for(size_t i = 0; i < hw_commands_.size(); ++i){
            joints_deg.at(i) = joint_offsets_deg_[i] - hw_commands_.at(i) * 180.0 / M_PI;
        }
        
       
        // For 6 joints, command format is: MoveJoints(j1,j2,j3,j4,j5,j6) in degrees
        std::string cmd = "MoveJoints("
          + std::to_string(joints_deg.at(0)) + ","
          + std::to_string(joints_deg.at(1)) + ","
          + std::to_string(joints_deg.at(2)) + ","
          + std::to_string(joints_deg.at(3)) + ","
          + std::to_string(joints_deg.at(4)) + ","
          + std::to_string(joints_deg.at(5)) + ")";
       
        send(control_fd, cmd.c_str(), cmd.size() + 1, 0);
       
        return hardware_interface::return_type::OK;
    }

    //================================================================================
    //                            On Deactivate
    //================================================================================

    hardware_interface::CallbackReturn Meca500System::on_deactivate(
      const rclcpp_lifecycle::State & previous_state){ 
        RCLCPP_INFO(rclcpp::get_logger("Meca500System"), "on_deactivate called");

        if (!use_fake_){
            std::string cmd = "DeactivateRobot";
            send(control_fd, cmd.c_str(), cmd.size() + 1, 0);
            char buf[256] = {0};
            recv(control_fd, buf, sizeof(buf) - 1, 0);
            RCLCPP_INFO(rclcpp::get_logger("Meca500System"),
              "DeactivateRobot: %s", buf);
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    //================================================================================
    //                            On Cleanup
    //================================================================================

    hardware_interface::CallbackReturn Meca500System::on_cleanup(
      const rclcpp_lifecycle::State & previous_state){
        RCLCPP_INFO(rclcpp::get_logger("Meca500System"), "on_cleanup called");

        if (control_fd >= 0){
            close(control_fd);
            control_fd = -1;
        }
 
        if (monitoring_fd >= 0){
          close(monitoring_fd);
          monitoring_fd = -1;
        }
 
        return hardware_interface::CallbackReturn::SUCCESS;
}



}  // namespace meca500_hardware

PLUGINLIB_EXPORT_CLASS(
  meca500_hardware::Meca500System,
  hardware_interface::SystemInterface
)