//
// Simple example to demonstrate how takeoff and land using MAVSDK.
//

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(const std::string& bin_name)
{
    std::cerr << "Connection URL format should be : udp://:14540 [Takeoff Altitude in meters Example: 4.0] [Pattern Dimensions in meters Example: 3.0]\n";
}

int main(int argc, char** argv)
{
    if (argc != 4) {
        usage(argv[0]);
        return 1;
    }

    double altitude;
    try {
        altitude = std::stod(argv[2]);

    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid floating-point value: " << argv[2] << "\n";
        return 1;

    } catch (const std::out_of_range& e) {
        std::cerr << "Floating-point value out of range:" << argv[2] << "\n";
        return 1;

    }

    double pattern_dimension;
    try {
        pattern_dimension = std::stod(argv[3]);

    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid floating-point value: " << argv[3] << "\n";
        return 1;

    } catch (const std::out_of_range& e) {
        std::cerr << "Floating-point value out of range:" << argv[3] << "\n";
        return 1;

    }

    Mavsdk::Configuration config(Mavsdk::ComponentType::GroundStation);
    Mavsdk mavsdk{config};

    // Aircraft Connection

    ConnectionResult aircraft_result = mavsdk.add_any_connection(argv[1]);
    if (aircraft_result != ConnectionResult::Success) {
        std::cerr << "Failed to connect to aircraft: " << aircraft_result << "\n";
        return 1;

    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};
    auto offboard = Offboard{system.value()};
    auto mavlink_passthrough = MavlinkPassthrough{system.value()};

    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(0.2);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return 1;
    }

//     Check until vehicle is ready to arm
     while (telemetry.health().is_local_position_ok != true) {
        std::cout << "Vehicle is getting ready to arm, poor position lock\n";
        sleep_for(seconds(1));
    }

    std::cout << "Local Position Valid...\n";
    
    // Callback to listen for manual control messages
    mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_HEARTBEAT, [](const mavlink_message_t& message){
        
        // std::cout << "Heartbeat Recieved" << "\n";
        mavlink_heartbeat_t mode_control;
        mavlink_msg_heartbeat_decode(&message, &mode_control);

        if (mode_control.custom_mode == 196608) {

            exit(0);

        }
 //       std::cerr << mode_control.custom_mode << "\n";

    });

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position_velocity_ned([](Telemetry::PositionVelocityNed position) {
      //  std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
    });

    if (telemetry.position_velocity_ned().position.down_m < -0.2f || telemetry.position_velocity_ned().position.down_m > 0.2f) {
    std::cerr << "Current local altitude is out of safe takeoff range (-0.5 to 0.5 m). Reboot Aircraft At Launch Location, and Do Not Move It Before Takeoff.\n";
    return 1;  // Exit the program or take other action to prevent takeoff
    } else {
    std::cout << "Current altitude is within safe takeoff range. Proceeding with takeoff.\n";
    }
    
    float takeoff_altitude = altitude; //Altitude in Meters

    Action:: Result set_altitude_result = action.set_takeoff_altitude(takeoff_altitude);
    if (set_altitude_result != Action::Result::Success) {

        std::cerr << "Failed to Set Takeoff Altitude" << set_altitude_result << std::endl;
        return 1;
    } else {

        std::cout << "Takeoff Altitude Set To:" << takeoff_altitude << "meters" << std::endl;

    }
    
    // Arm vehicle
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }

    // Take off

    std::cout << "Taking off...\n";
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    // Ensure we get within .5m of our desired altitude
    while (takeoff_altitude + telemetry.position_velocity_ned().position.down_m > 0.25f){
        std::cout << "Climbing... Current Altitude: " << -(telemetry.position_velocity_ned().position.down_m) << "\n";
        sleep_for(seconds(1)); 

    }

    // Hover for 5 seconds
    sleep_for(seconds(5));
    
    // Retry logic for entering Offboard mode
    int retry_offboard_count = 3;  // Number of retries
    while (retry_offboard_count > 0) {

        const Offboard::VelocityNedYaw stay{};
        offboard.set_velocity_ned(stay);
        
        std::cout << "Entering Offboard Mode...\n";
        Offboard::Result offboard_result = offboard.start();

        if (offboard_result == Offboard::Result::Success) {
            std::cout << "Offboard mode successfully started.\n";
            break;  // Exit the loop if successful
        } else {
            std::cerr << "Failed to enter Offboard mode. Retrying...\n";
            
            sleep_for(seconds(2));  // Wait for 2 seconds before retrying
            retry_offboard_count--;
        }
    }

    if (retry_offboard_count == 0) {
        std::cerr << "Unable to enter Offboard mode after multiple attempts. Exiting...\n";
        return 1;
    }

    std::cout << "Flying the "<< pattern_dimension << " meter Rectangle Pattern...\n";

    //Fly Pattern

    float ui = pattern_dimension;

    while (true) {
 

        float pa = -(takeoff_altitude);

        Offboard::PositionNedYaw point1{0.0f, 0.0f, pa, 0.0f};
        std::cout <<"Holding Over RX...\n" ;
        offboard.set_position_ned(point1);
        sleep_for(seconds(15));

        float battery_voltage = telemetry.battery().voltage_v;

        while (battery_voltage < 7.0f){

            std::cerr << "Current Battery Voltage is too Low (<7.0V) Hovering To Charge...";
            sleep_for(seconds(30));
        }

        std::cout << "Aircraft is Ready to Perform Flight Pattern... Please Press Enter \n ";

        //prompt 

        std::string user_input;
        std:getline(std::cin, user_input);

        // Define Pattern
        // Convert Takeoff Altitude to NED Altitude

        Offboard::PositionNedYaw point2{-(ui/2), 0.0f, pa, 0.0f};
        Offboard::PositionNedYaw point3{-(ui/2), (ui/2), pa, 0.0f};
        Offboard::PositionNedYaw point4{(ui/2), (ui/2), pa, 0.0f};
        Offboard::PositionNedYaw point5{(ui/2), -(ui/2), pa, 0.0f};
        Offboard::PositionNedYaw point6{-(ui/2), -(ui/2), pa, 0.0f};
        Offboard::PositionNedYaw point7{0.0f, 0.0f, pa, 0.0f};

        std::cout <<"Heading to Position 1...\n" ;
        offboard.set_position_ned(point2);
        sleep_for(seconds(10));

        std::cout <<"Heading to Position 2...\n" ;
        offboard.set_position_ned(point3);
        sleep_for(seconds(10));

        std::cout <<"Heading to Position 3...\n" ;
        offboard.set_position_ned(point4);
        sleep_for(seconds(10));

        std::cout <<"Heading to Position 4...\n" ;
        offboard.set_position_ned(point5);
        sleep_for(seconds(10));
        
        std::cout <<"Heading to Position 5...\n" ;
        offboard.set_position_ned(point6);
        sleep_for(seconds(10));

        std::cout <<"Heading to Position 6...\n" ;
        offboard.set_position_ned(point2);
        sleep_for(seconds(10));

        std::cout <<"Heading Back to RX...\n" ;
        offboard.set_position_ned(point7);
    
    }

//    std::cout << "Landing...\n";
//    const Action::Result land_result = action.land();
//    if (land_result != Action::Result::Success) {
//        std::cerr << "Land failed: " << land_result << '\n';
//        return 1;
//    }

    // Check if vehicle is still in air
//    while (telemetry.in_air()) {
//        std::cout << "Vehicle is landing...\n";
//        sleep_for(seconds(1));
//    }
//    std::cout << "Landed!\n";

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
//    sleep_for(seconds(3));
//    std::cout << "Finished...\n";

//    return 0;
}
