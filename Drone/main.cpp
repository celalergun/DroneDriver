#include <iostream>
#include "drone.h"
#include "communicator.h"
#include "beacon.h"
#include <thread>
#include <chrono>
#include <stop_token>
#include <signal.h>
#include <cmath>
#include <atomic>
#include <mutex>

using std::cout;
using std::endl;

std::stop_source stop_source;

void ctrlCPressed(int s)
{
    cout << "\nCtrl+C pressed, shutting down..." << endl;
    stop_source.request_stop();
}

// These values might have been read from a config file. I define them here for simplicity.
const uint8_t my_sysid = 1;
const uint8_t my_compid = 1;

// Map center coordinates (same map bounds as GCS)
const double MAP_CENTER_LAT = (59.430927 + 59.417272) / 2.0;  // 59.424100
const double MAP_CENTER_LON = (24.789792 + 24.836687) / 2.0;  // 24.813240

// Conversion constants at this latitude
const double METERS_PER_DEG_LAT = 111320.0;
const double METERS_PER_DEG_LON = 111320.0 * std::cos(MAP_CENTER_LAT * M_PI / 180.0);

const double DRONE_SPEED = 15.0; // meters per second

int main()
{
    std::cout << "Hello from Drone!" << std::endl;

    // catch ctrl+c
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrlCPressed;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Initialize drone at center of the map
    Drone drone(1, MAP_CENTER_LAT, MAP_CENTER_LON, 100.0, 0.0, 0.0);

    // Current position (lat, lon) — protected by mutex
    std::mutex pos_mutex;
    double current_lat = MAP_CENTER_LAT;
    double current_lon = MAP_CENTER_LON;

    // Target position — protected by mutex
    std::mutex target_mutex;
    double target_lat = MAP_CENTER_LAT;
    double target_lon = MAP_CENTER_LON;
    bool has_target = false;

    // Drone flight mode — shared with beacon thread
    std::atomic<uint8_t> drone_mode{MAV_MODE_GUIDED_ARMED};

    Communicator comm(14563);
    Beacon beacon;
    auto token = stop_source.get_token();
    cout << "Starting communicator..." << endl;
    comm.run(token);
    cout << "Communicator started. Starting beacon..." << endl;
    beacon.start(comm, token, drone_mode);
    comm.onMessage(1, [&](const mavlink_message_t &message)
                   {
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {
            mavlink_set_position_target_global_int_t target;
            mavlink_msg_set_position_target_global_int_decode(&message, &target);

            double lat = target.lat_int / 1e7;
            double lon = target.lon_int / 1e7;

            std::cout << "[TARGET] New Global Setpoint Received:" << std::endl;
            std::cout << "         Lat: " << lat 
                      << " Lon: " << lon 
                      << " Alt: " << target.alt << std::endl;

            {
                std::lock_guard lock(target_mutex);
                target_lat = lat;
                target_lon = lon;
                has_target = true;
            }
            drone_mode.store(MAV_MODE_AUTO_ARMED);

            // ACK with POSITION_TARGET_GLOBAL_INT
            {
                mavlink_message_t ack_msg;
                mavlink_msg_position_target_global_int_pack(
                    my_sysid, my_compid, &ack_msg,
                    0,                          // time_boot_ms
                    MAV_FRAME_GLOBAL_INT,
                    target.type_mask,
                    target.lat_int,
                    target.lon_int,
                    target.alt,
                    target.vx, target.vy, target.vz,
                    target.afx, target.afy, target.afz,
                    target.yaw, target.yaw_rate
                );
                comm.sendMessage(ack_msg);
                std::cout << "       Sent POSITION_TARGET_GLOBAL_INT ACK to GCS." << std::endl;
            }
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&message, &cmd);

            if (cmd.command == MAV_CMD_OVERRIDE_GOTO) {
                std::cout << "[GOTO] Received Override Command (HOLD)" << std::endl;

                {
                    std::lock_guard lock(target_mutex);
                    has_target = false;
                }
                drone_mode.store(MAV_MODE_GUIDED_ARMED);

                mavlink_message_t ack_msg;
                mavlink_msg_command_ack_pack(
                    my_sysid, my_compid, &ack_msg,
                    MAV_CMD_OVERRIDE_GOTO,
                    MAV_RESULT_ACCEPTED,
                    100, 0,
                    message.sysid,
                    message.compid
                );
                comm.sendMessage(ack_msg);
                std::cout << "       Sent COMMAND_ACK to GCS." << std::endl;
            }
            break;
        }        
        default:
            break;
        } });

    cout << "Beacon started. Entering main loop..." << endl;

    // Send initial position (center of map)
    {
        mavlink_message_t pos_msg;
        mavlink_msg_global_position_int_pack(
            my_sysid, my_compid, &pos_msg,
            0,                                              // time_boot_ms
            static_cast<int32_t>(current_lat * 1e7),        // lat
            static_cast<int32_t>(current_lon * 1e7),        // lon
            100000,                                         // alt (mm)
            100000,                                         // relative_alt (mm)
            0, 0, 0,                                        // vx, vy, vz (cm/s)
            0                                               // hdg (cdeg)
        );
        comm.sendMessage(pos_msg);
        std::cout << "[DRONE] Initial position sent: lat=" << current_lat << " lon=" << current_lon << std::endl;
    }

    const double dt = 0.1; // 100ms loop
    auto last_time = std::chrono::steady_clock::now();

    while (!token.stop_requested())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_time).count();
        last_time = now;

        // Move toward target if we have one
        {
            std::lock_guard tlock(target_mutex);
            if (has_target) {
                std::lock_guard plock(pos_mutex);

                double dlat = target_lat - current_lat;
                double dlon = target_lon - current_lon;

                // Convert to meters for distance calculation
                double dlat_m = dlat * METERS_PER_DEG_LAT;
                double dlon_m = dlon * METERS_PER_DEG_LON;
                double distance = std::sqrt(dlat_m * dlat_m + dlon_m * dlon_m);

                if (distance < 0.5) {
                    // Close enough — snap to target
                    current_lat = target_lat;
                    current_lon = target_lon;
                    has_target = false;
                    drone.updateState(current_lat, current_lon, 100.0, 0.0, 0.0);
                    drone_mode.store(MAV_MODE_GUIDED_ARMED);
                    std::cout << "[DRONE] Reached target position." << std::endl;
                } else {
                    // Move at DRONE_SPEED m/s along the direction
                    double step_m = DRONE_SPEED * elapsed;
                    double ratio = step_m / distance;
                    if (ratio > 1.0) ratio = 1.0;

                    current_lat += dlat * ratio;
                    current_lon += dlon * ratio;

                    // Calculate heading (degrees from north, clockwise)
                    double heading = std::atan2(dlon_m, dlat_m) * 180.0 / M_PI;
                    if (heading < 0) heading += 360.0;

                    drone.updateState(current_lat, current_lon, 100.0, DRONE_SPEED, heading);
                }
            }
        }

        // Send current position via GLOBAL_POSITION_INT
        {
            std::lock_guard plock(pos_mutex);
            mavlink_message_t pos_msg;

            double speed_val = 0.0;
            {
                std::lock_guard tlock(target_mutex);
                speed_val = has_target ? DRONE_SPEED : 0.0;
            }

            mavlink_msg_global_position_int_pack(
                my_sysid, my_compid, &pos_msg,
                0,
                static_cast<int32_t>(current_lat * 1e7),
                static_cast<int32_t>(current_lon * 1e7),
                100000,                                     // alt (mm)
                100000,                                     // relative_alt (mm)
                static_cast<int16_t>(speed_val * 100),      // vx (cm/s)
                0, 0,                                       // vy, vz
                0                                           // hdg
            );
            comm.sendMessage(pos_msg);
        }
    }
    return 0;
}
