#include "beacon.h"
#include <iostream>

using std::cout;
using std::endl;

void Beacon::start(Communicator &comm, std::stop_token stoken, std::atomic<uint8_t>& drone_mode)
{
    cout << "Starting beacon thread..." << endl;
    beacon_thread = std::jthread([this, &comm, &drone_mode, stoken](std::stop_token){ 
            while(!stoken.stop_requested()) {
                // Send a heartbeat message at 10 Hz to indicate the drone is alive and active
                mavlink_message_t heartbeat_msg;
                mavlink_msg_heartbeat_pack(1, 1, &heartbeat_msg,
                                   MAV_TYPE_QUADROTOR,
                                   MAV_AUTOPILOT_GENERIC,
                                   drone_mode.load(),
                                   0,
                                   MAV_STATE_ACTIVE);

                comm.sendMessage(heartbeat_msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } 
        });
}