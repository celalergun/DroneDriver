#include "communicator.h"
#include <cstdio>
#include <cstring>
#include <iostream>

using std::cout;
using std::endl;

Communicator::Communicator(int local_port)
{
    socket = UdpSocket();
    cout << "Communicator created, binding to port " << local_port << "..." << endl;
    socket.create(local_port, true);
    cout << "Communicator socket created and bound to port " << local_port << endl;
}

void Communicator::sendMessage(mavlink_message_t message)
{
    message_counter++;
    outgoing_queue.push(OutgoingMessage{message_counter, message, std::chrono::system_clock::now()});
}

void Communicator::onMessage(uint32_t subscriber_id, std::function<void(const mavlink_message_t &)> callback)
{
    subscribers[subscriber_id] = callback;
}

void Communicator::run(std::stop_token stoken)
{
    receive_thread = std::jthread([this, stoken](std::stop_token) { receiveLoop(stoken); });
    send_thread = std::jthread([this, stoken](std::stop_token) { sendLoop(stoken); });
}

void Communicator::sendLoop(std::stop_token stoken)
{
    while (!stoken.stop_requested()) {
        auto outgoing_opt = outgoing_queue.wait_and_pop(stoken);
        if (!outgoing_opt) break; // Stop requested

        OutgoingMessage msg = std::move(*outgoing_opt);
        uint8_t buffer[1024];
        int length = mavlink_msg_to_send_buffer(buffer, &msg.message);
        //cout << "Communicator: Sending message #" << msg.message_counter << " (" << length << " bytes) to proxy at " << proxy_ip << ":" << proxy_port << endl;
        socket.sendto(buffer, length, IpAddress(proxy_ip.c_str(), proxy_port));
    }
}    

void Communicator::receiveLoop(std::stop_token stoken)
{
    IpAddress sender_addr;
    char buffer[1024];
    mavlink_message_t message;
    while (!stoken.stop_requested()) {
        if (socket.available() > 0) {
            int receivedBytes = socket.recvfrom((void*)buffer, sizeof(buffer), sender_addr);
            cout << "Communicator: Received " << receivedBytes << " bytes from " << sender_addr.to_string() << endl;
            
            // Check if this is a discovery response (int[2])
            if (receivedBytes == static_cast<int>(sizeof(int) * 2) && !discovered) {
                int marker = 0;
                std::memcpy(&marker, buffer, sizeof(int));
                if (marker == DISCOVERY_RESPONSE_MARKER) {
                    discovered = true;
                    char ip_str[16];
                    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
                        sender_addr.addr_parts[0], sender_addr.addr_parts[1],
                        sender_addr.addr_parts[2], sender_addr.addr_parts[3]);
                    proxy_ip = std::string(ip_str);
                    cout << "Communicator: Discovered proxy at " << proxy_ip << ":" << sender_addr.port << endl;
                    continue;
                }
            }
            
            // Process the received message
            mavlink_status_t status;
            for (int i = 0; i < receivedBytes; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, static_cast<uint8_t>(buffer[i]), &message, &status)) {
                    // Handle the parsed message
                    for (const auto& [subscriber_id, callback] : subscribers) {
                        callback(message);
                    }
                }
            }
        }
    }
}

