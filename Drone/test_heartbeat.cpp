#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h> 
#include <chrono>
#include <iomanip> 
#include "mavlink/common/mavlink.h"

std::string get_msg_name(uint32_t msgid) {
    switch (msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:    return "HEARTBEAT";
        case MAVLINK_MSG_ID_COMMAND_LONG: return "COMMAND_LONG";
        case MAVLINK_MSG_ID_PING:         return "PING";
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: return "PARAM_REQUEST_LIST";
        case MAVLINK_MSG_ID_MANUAL_CONTROL: return "MANUAL_CONTROL";
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: return "MISSION_REQUEST_LIST";
        default: return "OTHER (" + std::to_string(msgid) + ")";
    }
}


void dump_message(mavlink_message_t& msg, int sock, struct sockaddr_in qgc_addr) {
    switch (msg.msgid) {
        
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            std::cout << "[RECV] QGC is requesting your full Parameter List (Handshake)." << std::endl;

            struct MyParam { std::string name; float value; MAV_PARAM_TYPE type; };
            std::vector<MyParam> my_params = {
                {"SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_UINT8},
                {"MAV_TYPE",      2.0f, MAV_PARAM_TYPE_UINT8}, // 2 = Quadrotor
                {"PILOT_SPEED",   5.5f, MAV_PARAM_TYPE_REAL32}
            };

            for (uint16_t i = 0; i < my_params.size(); ++i) {
                mavlink_message_t p_msg;
                
                mavlink_msg_param_value_pack(
                    1, 1, &p_msg,
                    my_params[i].name.c_str(),  // Param ID
                    my_params[i].value,         // Value
                    my_params[i].type,          // Type
                    my_params.size(),           // Total Count
                    i                           // Current Index
                );

                sendto(sock, &p_msg, sizeof(p_msg), 0, (struct sockaddr*)&qgc_addr, sizeof(qgc_addr));
            }
            }
            break;

        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);
            
            if (cmd.command == MAV_CMD_DO_SET_HOME) {
                // Command 179: Set Home
                // Param 5 = Latitude, Param 6 = Longitude
                std::cout << "[HOME] QGC is setting Home via COMMAND_LONG:" << std::endl;
                std::cout << "       Lat: " << std::fixed << std::setprecision(6) << cmd.param5 << std::endl;
                std::cout << "       Lon: " << cmd.param6 << std::endl;
            } else {
                std::cout << "[RECV] COMMAND_LONG (ID: " << cmd.command << ")" << std::endl;
            }
            break;
        }

        case MAVLINK_MSG_ID_SET_HOME_POSITION: {
            // Message 243: Direct Set Home Position
            mavlink_set_home_position_t home;
            mavlink_msg_set_home_position_decode(&msg, &home);
            
            // MAVLink GPS coordinates are often "int32_t" scaled by 1E7
            std::cout << "[HOME] QGC is setting Home via SET_HOME_POSITION message:" << std::endl;
            std::cout << "       Lat: " << (double)home.latitude / 1e7 << std::endl;
            std::cout << "       Lon: " << (double)home.longitude / 1e7 << std::endl;
            break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            mavlink_param_request_read_t req;
            mavlink_msg_param_request_read_decode(&msg, &req);

            if (req.param_index == -1) {
                // QGC is asking by NAME (e.g., "SYSID_THISMAV")
                std::cout << "[RECV] Request to read param by NAME: " << req.param_id << std::endl;
            } else {
                // QGC is asking by INDEX (e.g., "Give me parameter #1")
                std::cout << "[RECV] Request to read param by INDEX: " << req.param_index << std::endl;
            }
            break;
        }
        case MAVLINK_MSG_ID_SYSTEM_TIME: {
            mavlink_system_time_t t;
            mavlink_msg_system_time_decode(&msg, &t);
            std::cout << "[TIME] QGC Unix Time: " << t.time_unix_usec << " us" << std::endl;
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
            std::cout << "[MISSION] QGC asking for mission count. Sending 0..." << std::endl;
            
            mavlink_message_t p_msg;
            mavlink_msg_mission_count_pack(
                1, 
                1, 
                &p_msg, 
                msg.sysid,  // Target System (QGC)
                msg.compid, // Target Component
                0,          // We have 0 mission items
                MAV_MISSION_TYPE_MISSION, // This is a standard flight mission
                0           // Opaque ID (not used here)
            );
            sendto(sock, &p_msg, sizeof(p_msg), 0, (struct sockaddr*)&qgc_addr, sizeof(qgc_addr));
            break;
        }
        default:
            std::cout << "[RECV] Message ID: " << msg.msgid << " (No decoder implemented)" << std::endl;
            break;
    }
}

int main() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(14551); 

    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "Bind failed! Port might be in use." << std::endl;
        return 1;
    }

    fcntl(sock, F_SETFL, O_NONBLOCK);
    struct sockaddr_in qgc_addr;
    memset(&qgc_addr, 0, sizeof(qgc_addr));
    qgc_addr.sin_family = AF_INET;
    qgc_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    qgc_addr.sin_port = htons(14550);

    std::cout << "Starting MAVLink node. Talking to QGC on 14550..." << std::endl;

    mavlink_status_t status;
    mavlink_message_t msg_recv;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    auto last_heartbeat = std::chrono::steady_clock::now();

    while (true) {
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        ssize_t recvd = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr*)&from_addr, &from_len);
        
        if (recvd > 0) {
            for (ssize_t i = 0; i < recvd; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg_recv, &status)) {
                    dump_message(msg_recv, sock, qgc_addr);
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (now - last_heartbeat > std::chrono::seconds(1)) {
            mavlink_message_t msg_send;
            mavlink_msg_heartbeat_pack(1, 1, &msg_send, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
            
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_send);
            sendto(sock, buf, len, 0, (struct sockaddr*)&qgc_addr, sizeof(qgc_addr));
            
            last_heartbeat = now;
        }
        usleep(10000); 
    }

    close(sock);
    return 0;
}