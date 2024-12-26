"""
Copyright 2024 Lorenzo Grandi

GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007

Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
Everyone is permitted to copy and distribute verbatim copies
of this license document, but changing it is not allowed.
"""

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

class PX4DroneController : public rclcpp::Node
{
public:
    PX4DroneController() : Node("px4_drone_control")
    {
        // Setup for Drone 1 (system ID 2)
        setup_drone("/px4_1", px4_1_, 2);
        px4_1_.waypoints = {
            {0.0, 0.0, -3.5}, 
            {-10.0, 10.0, -3.5}, 
            {-10.0, -10.0, -3.5}, 
            {-8.0, -10.0, -3.5}, 
            {-8.0, 10.0, -3.5},
            {-6.0, 10.0, -3.5},
            {-6.0, -10.0, -3.5},
            {-4.0, -10.0, -3.5},
            {-4.0, 10.0, -3.5},
            {-2.0, 10.0, -3.5},
            {-2.0, -10.0, -3.5}
        };

        // Setup for Drone 2 (system ID 3)
        setup_drone("/px4_2", px4_2_, 3);
        px4_2_.waypoints = {
            {0.0, 0.0, -4.0}, 
            {-3.0, -13.0, -4.0}, // Considering a (-3,-3) offset
            {-3.0, 7.0, -4.0},
            {-1.0, 7.0, -4.0},
            {-1.0, -13.0, -4.0},
            {1.0, -13.0, -4.0},
            {1.0, 7.0, -4.0},
            {3.0, 7.0, -4.0},
            {3.0, -13.0, -4.0},
            {5.0, -13.0, -4.0},
            {5.0, 7.0, -4.0}
        };

        // Start timer to control the drones at intervals (10 Hz) -> heartbeat function
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PX4DroneController::control_loop, this)); // Control loop every 1/10 of a sec

        RCLCPP_INFO(this->get_logger(), "PX4 Drone Controller Node Initialized.");
    }

private:
    float fixed_velocity = 1; // Fixed velocity magnitude (in m/s)
    
    struct Drone
    {        
        // Publishers
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;

        // Subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub;
        rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub;

        // State variables
        uint64_t timestamp;
        bool armed;
        bool offboard_mode;
        uint8_t nav_state;      // Navigation state
        uint8_t arming_state;   // Arming state
        uint8_t system_id;
        std::vector<std::array<float, 3>> waypoints; // List of waypoints
        size_t current_waypoint_index = 0;          // Index of the current waypoint
        bool timestamp_synced = false;
        std::array<float, 3> current_position = {0.0, 0.0, 0.0}; // Current position
    };

    // Drones' instancies
    Drone px4_1_;
    Drone px4_2_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    void setup_drone(const std::string &namespace_, Drone &drone, uint8_t system_id)
    {
        // Initialize publishers
        drone.offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            namespace_ + "/fmu/in/offboard_control_mode", 10);
        drone.trajectory_setpoint_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            namespace_ + "/fmu/in/trajectory_setpoint", 10);
        drone.vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            namespace_ + "/fmu/in/vehicle_command", 10);

        // Define QoS profile matching the PX4 publishers
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        // Initialize subscribers with the matching QoS profile
        drone.timesync_sub = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
            namespace_ + "/fmu/out/timesync_status",
            qos_profile,
            [&drone](const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
                drone.timestamp = msg->timestamp;
                drone.timestamp_synced = true;
            });

        drone.vehicle_status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            namespace_ + "/fmu/out/vehicle_status",
            qos_profile,
            [&drone](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                drone.nav_state = msg->nav_state;
                drone.arming_state = msg->arming_state;
            });
            
        drone.local_position_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            namespace_ + "/fmu/out/vehicle_local_position",
            qos_profile,
            [&drone](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                // Update the current position
                drone.current_position[0] = msg->x; // x position
                drone.current_position[1] = msg->y; // y position
                drone.current_position[2] = msg->z; // z position
            });

        // Initialize state variables
        drone.timestamp = 0;
        drone.armed = false;
        drone.offboard_mode = false;
        drone.system_id = system_id;

        RCLCPP_INFO(this->get_logger(), "Drone with system ID %d set up.", drone.system_id);
    }

    void control_loop()
    {
        control_drone(px4_1_);
        control_drone(px4_2_);
    }




    void control_drone(Drone &drone)
    {
        if (!drone.timestamp_synced) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for timestamp synchronization for drone %d...", drone.system_id);
            return;
        }
        
        publish_offboard_control_mode(drone);

        // Navigate through waypoints
        if (drone.current_waypoint_index < drone.waypoints.size()) {
            const auto &waypoint = drone.waypoints[drone.current_waypoint_index];
            publish_trajectory_setpoint(drone, waypoint);

            // Check proximity to waypoint
            if (has_reached_waypoint(drone, waypoint)) {
                drone.current_waypoint_index++;
                RCLCPP_INFO(this->get_logger(),
                            "Drone %d reached waypoint %zu. Moving to next.",
                            drone.system_id, drone.current_waypoint_index);
            }
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Drone %d has completed its waypoints.", drone.system_id);
        }

        if (drone.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            arm_drone(drone);
        } else {
            drone.armed = true;
        }

        if (drone.armed && !drone.offboard_mode) {
            enable_offboard(drone);
        }
    }




    // Functions:
    
    bool has_reached_waypoint(Drone &drone, const std::array<float, 3> &waypoint)
    {
        const auto &current_position = drone.current_position;
        //RCLCPP_INFO(this->get_logger(), "Drone with system ID %d position: (%f,%f,%f)", drone.system_id, current_position[0], current_position[1], current_position[2]);

        // Compute Euclidean distance to the waypoint
        float distance = std::sqrt(
            std::pow(current_position[0] - waypoint[0], 2) +
            std::pow(current_position[1] - waypoint[1], 2) +
            std::pow(current_position[2] - waypoint[2], 2));

        // Define a threshold for considering the waypoint as reached
        const float threshold = 0.5; // Adjust as needed

        return distance < threshold;
    }

    void publish_offboard_control_mode(Drone &drone)
    {
        auto msg = px4_msgs::msg::OffboardControlMode();
        msg.timestamp = drone.timestamp;
        msg.position = true;
        drone.offboard_control_mode_pub->publish(msg);
    }
    
    void publish_trajectory_setpoint(Drone &drone, const std::array<float, 3> &waypoint)
    {
        auto traj_msg = px4_msgs::msg::TrajectorySetpoint();
        traj_msg.timestamp = drone.timestamp;
        
        // Current position
        const auto &current_position = drone.current_position;

        // Compute direction vector to waypoint
        float dx = waypoint[0] - current_position[0];
        float dy = waypoint[1] - current_position[1];
        float dz = waypoint[2] - current_position[2];

        // Normalize direction vector
        float magnitude = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (magnitude > 0.1) // Avoid division by zero for very close waypoints
        {
            traj_msg.velocity[0] = fixed_velocity * (dx / magnitude);
            traj_msg.velocity[1] = fixed_velocity * (dy / magnitude);
            traj_msg.velocity[2] = fixed_velocity * (dz / magnitude);
        }
        else
        {
            traj_msg.velocity[0] = 0.0;
            traj_msg.velocity[1] = 0.0;
            traj_msg.velocity[2] = 0.0;
        }
        
        // Calculate desired yaw based on movement direction in the x-y plane
        traj_msg.yaw = std::atan2(dy, dx); // Yaw is in radians

        // Keep position setpoints empty to focus on velocity control
        traj_msg.position[0] = NAN;
        traj_msg.position[1] = NAN;
        traj_msg.position[2] = NAN;

        drone.trajectory_setpoint_pub->publish(traj_msg);
    }

    void enable_offboard(Drone &drone)
    {
        auto cmd_msg = px4_msgs::msg::VehicleCommand();
        cmd_msg.timestamp = drone.timestamp;
        cmd_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd_msg.param1 = 1.0;
        cmd_msg.param2 = 6.0;
        cmd_msg.target_system = drone.system_id;
        cmd_msg.target_component = 1;
        cmd_msg.source_system = drone.system_id;
        cmd_msg.source_component = 1;
        cmd_msg.from_external = true;
        drone.vehicle_command_pub->publish(cmd_msg);
    }

    void arm_drone(Drone &drone)
    {
        auto arm_cmd = px4_msgs::msg::VehicleCommand();
        arm_cmd.timestamp = drone.timestamp;
        arm_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        arm_cmd.param1 = 1.0;
        arm_cmd.target_system = drone.system_id;
        arm_cmd.target_component = 1;
        arm_cmd.source_system = drone.system_id;
        arm_cmd.source_component = 1;
        arm_cmd.from_external = true;
        drone.vehicle_command_pub->publish(arm_cmd);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4DroneController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
