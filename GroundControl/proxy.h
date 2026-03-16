#ifndef PROXY_H
#define PROXY_H

#include <udp/simple_udp.h>
#include <mavlink/common/mavlink.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <vector>
#include <queue>
#include <random>
#include <functional>
#include <condition_variable>
#include <algorithm>
#include <cstdint>

// Proxy: a UDP bridge between Drone and GCS that simulates radio link conditions
// - Configurable packet drop percentage
// - Data rate limited to 32KB/s
// - Radio buffer limited to 1024 bytes (packets that don't fit are dropped)
// - Simulated delay: packetDelay = packetLen * timePerByte
class Proxy {
public:
    static constexpr uint16_t PROXY_PORT = 14550;              // Drone connects here
    static constexpr int MAX_DATA_RATE = 32768;                // 32KB/s
    static constexpr int MAX_BUFFER_SIZE = 1024;               // Radio buffer in bytes
    static constexpr double TIME_PER_BYTE = 1.0 / MAX_DATA_RATE; // ~30.5 µs/byte

    // Discovery markers (must match Drone communicator)
    static constexpr int DISCOVERY_REQUEST = MAV_CMD_USER_4;
    static constexpr int DISCOVERY_RESPONSE = MAV_CMD_USER_5;

    Proxy();
    ~Proxy();

    // Start the proxy (spawns recv + delivery threads)
    void start();
    // Stop the proxy and join threads
    void stop();

    // Thread-safe: set/get packet drop percentage (0-100)
    void setPacketDropPercent(int percent);
    int getPacketDropPercent() const;

    // Send a MAVLink message to the drone (GCS → Drone direction)
    // Applies filtering (drop, rate limit, buffer, delay)
    void sendToClient(const mavlink_message_t& msg);

    // Register callback for MAVLink messages received from drone (Drone → GCS)
    // The callback is invoked from the delivery thread — keep it fast or copy data out
    void onMessageFromClient(std::function<void(const mavlink_message_t&)> callback);

    // Check if drone has been discovered
    bool isDroneConnected() const;

    // Statistics
    uint64_t getPacketsForwarded() const;
    uint64_t getPacketsDropped() const;

private:
    // Internal packet queued for delayed delivery
    struct DelayedPacket {
        std::vector<uint8_t> data;
        std::chrono::steady_clock::time_point deliver_at;
        bool to_gcs;  // true = deliver to GCS callback, false = send to drone
        int size;      // original packet size for buffer accounting

        // Earliest-first ordering for priority queue
        bool operator>(const DelayedPacket& other) const {
            return deliver_at > other.deliver_at;
        }
    };

    UdpSocket socket;

    std::atomic<int> packet_drop_percent{0};
    std::atomic<bool> running{false};
    std::atomic<bool> drone_connected{false};

    IpAddress drone_addr;
    std::mutex drone_addr_mutex;

    // Radio buffer simulation
    struct RadioBuffer {
        std::mutex mutex;
        int used = 0; // Current bytes occupying the buffer
    } radio_buffer;

    // Delayed delivery queue (earliest first)
    std::priority_queue<DelayedPacket, std::vector<DelayedPacket>,
                        std::greater<DelayedPacket>> pending_packets;
    std::mutex pending_mutex;
    std::condition_variable_any pending_cv;

    // GCS message callback
    std::function<void(const mavlink_message_t&)> message_callback;
    std::mutex callback_mutex;

    // Statistics
    std::atomic<uint64_t> packets_forwarded{0};
    std::atomic<uint64_t> packets_dropped{0};

    // Threads
    std::jthread recv_thread;
    std::jthread delivery_thread;

    // RNG for packet drop
    std::mt19937 rng{std::random_device{}()};
    std::mutex rng_mutex;

    // Thread entry points
    void recvLoop(std::stop_token stoken);
    void deliveryLoop(std::stop_token stoken);

    // Returns true if this packet should be randomly dropped
    bool shouldDrop();

    // Try to enqueue a packet for delayed delivery.
    // Returns false if packet was dropped (rate/buffer/random).
    bool tryEnqueue(const uint8_t* data, int len, bool to_gcs);

    // Handle drone discovery request
    void handleDiscovery(const IpAddress& sender);
};

#endif // PROXY_H