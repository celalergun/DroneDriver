#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <mavlink/common/mavlink.h>
#include <chrono>
#include <thread>
#include <udp/simple_udp.h>
#include "threadsafe_queue.h"
#include <functional>
#include <map>

// Discovery packet markers
constexpr int DISCOVERY_REQUEST_MARKER = MAV_CMD_USER_4;
constexpr int DISCOVERY_RESPONSE_MARKER = MAV_CMD_USER_5;

struct OutgoingMessage {
    int message_counter;
    mavlink_message_t message;
    std::chrono::system_clock::time_point time;
};

class ICommunicator {
public:
    virtual ~ICommunicator() = default;
    virtual void sendMessage(mavlink_message_t message) = 0;
    virtual void onMessage(uint32_t subscriber_id, std::function<void(const mavlink_message_t&)> callback) = 0;
    virtual void run(std::stop_token stoken) = 0;
};

class Communicator : public ICommunicator {
public:
    Communicator() = delete;
    Communicator(int local_port);
    void sendMessage(mavlink_message_t message);
    void onMessage(uint32_t subscriber_id, std::function<void(const mavlink_message_t&)> callback);
    void run(std::stop_token stoken);

private:
    std::atomic<int> message_counter{0};
    UdpSocket socket;
    bool discovered = false;
    std::string proxy_ip = "127.0.0.1"; // localhost — proxy runs on the same machine
    int proxy_port = 14550; // must match Proxy::PROXY_PORT
    ThreadSafeQueue<OutgoingMessage> outgoing_queue;
    std::map<uint32_t, std::function<void(const mavlink_message_t&)>> subscribers;
    std::jthread receive_thread;
    std::jthread send_thread;
    void receiveLoop(std::stop_token stoken);
    void sendLoop(std::stop_token stoken);
};

#endif // COMMUNICATOR_H
