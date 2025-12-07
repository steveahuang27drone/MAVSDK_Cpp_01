// takeoff -> when altitude > 1m, start rotating while climbing to 5m
// -> hover 5s -> land

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Example (SITL): " << bin_name << " udp://:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    // GroundStation connection
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    const auto connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(5.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    Telemetry telemetry{system.value()};
    Action action{system.value()};
    Offboard offboard{system.value()};

    // Set the position update rate
    const auto set_rate_result = telemetry.set_rate_position(5.0); // 5 Hz
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return 1;
    }

    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "[Telem] Altitude (rel): " << position.relative_altitude_m << " m\n";
    });

    // Wait until the vehicle is healthy
    while (!telemetry.health_all_ok()) {
        std::cout << "Vehicle is getting ready to arm...\n";
        sleep_for(seconds(1));
    }

    // Arm
    std::cout << "Arming...\n";
    const auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }

    // Takeoff
    std::cout << "Taking off...\n";
    const auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    // Wait until altitude > 1 m
    std::cout << "Waiting until altitude > 1.0 m...\n";
    while (true) {
        Telemetry::Position pos = telemetry.position();
        std::cout << "Current altitude: " << pos.relative_altitude_m << " m\n";

        if (pos.relative_altitude_m > 1.0f) {
            std::cout << "Altitude above 1.0 m, start rotate + climb.\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Send a neutral (zero) velocity setpoint before starting Offboard
    {
        Offboard::VelocityBodyYawspeed stay{};
        stay.forward_m_s = 0.0f;
        stay.right_m_s   = 0.0f;
        stay.down_m_s    = 0.0f;
        stay.yawspeed_deg_s = 0.0f;

        const auto set_result = offboard.set_velocity_body(stay);
        if (set_result != Offboard::Result::Success) {
            std::cerr << "Offboard set_velocity_body (stay) failed: "
                      << set_result << '\n';
            return 1;
        }
    }

    std::cout << "Starting Offboard...\n";
    auto offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return 1;
    }

    // Velocity setpoint for rotating while climbing:
    //   down_m_s < 0 => climb
    //   yawspeed_deg_s > 0 => clockwise (viewed from above)
    const float climb_speed_m_s   = 0.5f;  // climb at 0.5 m/s
    const float yaw_rate_deg_s    = 45.0f; // rotate at 45 deg/s
    const float target_altitude_m = 5.0f;

    Offboard::VelocityBodyYawspeed rotate_climb{};
    rotate_climb.forward_m_s = 0.0f;
    rotate_climb.right_m_s   = 0.0f;
    rotate_climb.down_m_s    = -climb_speed_m_s;
    rotate_climb.yawspeed_deg_s = yaw_rate_deg_s;

    offboard_result = offboard.set_velocity_body(rotate_climb);
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard set_velocity_body (rotate_climb) failed: "
                  << offboard_result << '\n';
        return 1;
    }

    std::cout << "Rotating while climbing to " << target_altitude_m << " m...\n";

    // Keep rotating and climbing until altitude >= 5 m
    while (true) {
        Telemetry::Position pos = telemetry.position();
        const float alt = pos.relative_altitude_m;

        std::cout << "[Rotate+Climb] Alt: " << alt << " m\n";

        if (alt >= target_altitude_m) {
            std::cout << "Reached target altitude (" << target_altitude_m
                      << " m). Stop climb & rotation.\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Stop rotating and climbing, switch to hover
    Offboard::VelocityBodyYawspeed hover{};
    hover.forward_m_s = 0.0f;
    hover.right_m_s   = 0.0f;
    hover.down_m_s    = 0.0f;
    hover.yawspeed_deg_s = 0.0f;

    offboard_result = offboard.set_velocity_body(hover);
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard set_velocity_body (hover) failed: "
                  << offboard_result << '\n';
        return 1;
    }

    // Hover for 5 seconds
    std::cout << "Hovering for 5 seconds...\n";
    sleep_for(seconds(5));

    // Stop Offboard
    std::cout << "Stopping Offboard...\n";
    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return 1;
    }

    // Land
    std::cout << "Landing...\n";
    const auto land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed: " << land_result << '\n';
        return 1;
    }

    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }

    std::cout << "Landed. Finished.\n";
    return 0;
}
