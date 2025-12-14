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

    // Set takeoff altitude
    action.set_takeoff_altitude(1.75f);
    const auto max_wait = seconds(20);        // safety timeout
    auto start = std::chrono::steady_clock::now();
    // Takeoff
    std::cout << "Taking off...\n";
    const auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }
    // Wait until we reach ~1.7 m
    while (true) {
        const auto pos = telemetry.position();
        std::cout << "Current altitude: "
                << pos.relative_altitude_m << " m\n";

        if (pos.relative_altitude_m >= 1.7f) {
            std::cout << "Altitude above 1.7 m, Hi, Monalisa and Lenna!\n";
            break;
        }
        sleep_for(seconds(1));
    }

    int hover_count = 0;
    while (true) {
        // Optional: avoid infinite wait in case something is wrong
        auto now = std::chrono::steady_clock::now();
        int duration = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        std::cout << "Timeout waiting for hover count " << duration << "\n";
                
        if (now - start > max_wait) {
            break;
        }  
        sleep_for(seconds(1));
    }

    // Hover for 5 seconds
    // std::cout << "Hovering for 5 seconds...\n";

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
