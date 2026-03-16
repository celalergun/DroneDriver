#include "proxy.h"
#include <cstring>
#include <cstdio>

Proxy::Proxy() = default;

Proxy::~Proxy() {
    stop();
}

void Proxy::start() {
    if (running.exchange(true)) return; // Already running

    if (!socket.create(PROXY_PORT, false)) {
        fprintf(stderr, "[Proxy] CRITICAL: Failed to bind port %d (port in use?)\n", PROXY_PORT);
        running.store(false);
        return;
    }
    printf("[Proxy] Listening on port %d\n", PROXY_PORT);

    radio_buffer.used = 0;
    packets_forwarded.store(0);
    packets_dropped.store(0);

    recv_thread = std::jthread([this](std::stop_token st) { recvLoop(st); });
    delivery_thread = std::jthread([this](std::stop_token st) { deliveryLoop(st); });
}

void Proxy::stop() {
    if (!running.exchange(false)) return;

    recv_thread.request_stop();
    delivery_thread.request_stop();
    pending_cv.notify_all();

    // Close socket to unblock poll_read
    socket.close();

    if (recv_thread.joinable()) recv_thread.join();
    if (delivery_thread.joinable()) delivery_thread.join();

    printf("[Proxy] Stopped. Forwarded: %lu, Dropped: %lu\n",
           packets_forwarded.load(), packets_dropped.load());
}

void Proxy::setPacketDropPercent(int percent) {
    packet_drop_percent.store(std::clamp(percent, 0, 100));
}

int Proxy::getPacketDropPercent() const {
    return packet_drop_percent.load();
}

bool Proxy::isDroneConnected() const {
    return drone_connected.load();
}

uint64_t Proxy::getPacketsForwarded() const {
    return packets_forwarded.load();
}

uint64_t Proxy::getPacketsDropped() const {
    return packets_dropped.load();
}

void Proxy::onMessageFromClient(std::function<void(const mavlink_message_t&)> callback) {
    std::lock_guard lock(callback_mutex);
    message_callback = std::move(callback);
}

void Proxy::sendToClient(const mavlink_message_t& msg) {
    // Serialize MAVLink message to bytes
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);

    // Apply filtering and enqueue for delayed delivery to drone
    if (!tryEnqueue(buffer, len, /*to_gcs=*/false)) {
        packets_dropped++;
    }
}

bool Proxy::shouldDrop() {
    int drop_pct = packet_drop_percent.load();
    if (drop_pct <= 0) return false;
    if (drop_pct >= 100) return true;

    std::lock_guard lock(rng_mutex);
    std::uniform_int_distribution<int> dist(0, 99);
    return dist(rng) < drop_pct;
}

bool Proxy::tryEnqueue(const uint8_t* data, int len, bool to_gcs) {
    // 1. Random packet drop
    if (shouldDrop()) {
        return false;
    }

    // 2. Check radio buffer capacity
    {
        std::lock_guard lock(radio_buffer.mutex);
        if (radio_buffer.used + len > MAX_BUFFER_SIZE) {
            return false; // Buffer full — packet dropped
        }
        radio_buffer.used += len;
    }

    // 3. Calculate delivery delay: packetDelay = packetLen * timePerByte
    auto delay = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(len * TIME_PER_BYTE));
    auto deliver_at = std::chrono::steady_clock::now() + delay;

    // 4. Enqueue for delayed delivery
    DelayedPacket pkt;
    pkt.data.assign(data, data + len);
    pkt.deliver_at = deliver_at;
    pkt.to_gcs = to_gcs;
    pkt.size = len;

    {
        std::lock_guard lock(pending_mutex);
        pending_packets.push(std::move(pkt));
    }
    pending_cv.notify_one();

    return true;
}

void Proxy::handleDiscovery(const IpAddress& sender) {
    printf("[Proxy] Discovery request from %s\n", sender.to_string().c_str());

    {
        std::lock_guard lock(drone_addr_mutex);
        drone_addr = sender;
    }
    drone_connected.store(true);

    // Send discovery response back to drone
    int response[2] = {DISCOVERY_RESPONSE, 0};
    socket.sendto(response, sizeof(response), sender);
    printf("[Proxy] Discovery response sent to %s\n", sender.to_string().c_str());
}

// ---------------------------------------------------------------------------
// Receive thread: reads packets from the drone-facing socket
// ---------------------------------------------------------------------------
void Proxy::recvLoop(std::stop_token stoken) {
    char buffer[2048];
    IpAddress sender;

    while (!stoken.stop_requested()) {
        // poll_read with 50ms timeout so we can check stop_requested periodically
        if (!socket.is_valid()) break;
        if (!socket.poll_read(50)) continue;

        int received = socket.recvfrom(buffer, sizeof(buffer), sender);
        if (received <= 0) continue;

        // Check for discovery packet (int[2])
        if (received == static_cast<int>(sizeof(int) * 2)) {
            int marker = 0;
            std::memcpy(&marker, buffer, sizeof(int));
            if (marker == DISCOVERY_REQUEST) {
                handleDiscovery(sender);
                continue;
            }
        }

        // Regular packet from drone — remember its address
        {
            std::lock_guard lock(drone_addr_mutex);
            drone_addr = sender;
        }
        drone_connected.store(true);

        // Enqueue for delayed delivery to GCS
        if (!tryEnqueue(reinterpret_cast<uint8_t*>(buffer), received, /*to_gcs=*/true)) {
            packets_dropped++;
        }
    }
}

// ---------------------------------------------------------------------------
// Delivery thread: delivers delayed packets when their time comes
// ---------------------------------------------------------------------------
void Proxy::deliveryLoop(std::stop_token stoken) {
    while (!stoken.stop_requested()) {
        std::unique_lock lock(pending_mutex);

        // Wait for packets to arrive
        if (pending_packets.empty()) {
            bool has_data = pending_cv.wait(lock, stoken, [this] {
                return !pending_packets.empty();
            });
            if (!has_data) continue; // Stop requested or spurious
        }

        // Check if the earliest packet is due
        auto now = std::chrono::steady_clock::now();
        if (pending_packets.top().deliver_at > now) {
            lock.unlock();
            // Small sleep — delays are in the microsecond-to-millisecond range
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }

        // Extract packet (copy from const ref, then pop)
        DelayedPacket pkt = pending_packets.top();
        pending_packets.pop();
        lock.unlock();

        // Free radio buffer space
        {
            std::lock_guard bLock(radio_buffer.mutex);
            radio_buffer.used = std::max(0, radio_buffer.used - pkt.size);
        }

        // Deliver the packet
        if (pkt.to_gcs) {
            // Drone → GCS: parse MAVLink bytes and invoke callback
            mavlink_message_t msg;
            mavlink_status_t status;
            for (auto byte : pkt.data) {
                if (mavlink_parse_char(MAVLINK_COMM_1, byte, &msg, &status)) {
                    std::lock_guard cLock(callback_mutex);
                    if (message_callback) {
                        message_callback(msg);
                    }
                }
            }
        } else {
            // GCS → Drone: send raw bytes to remembered drone address
            std::lock_guard dLock(drone_addr_mutex);
            if (drone_connected.load()) {
                socket.sendto(pkt.data.data(), pkt.data.size(), drone_addr);
            }
        }

        packets_forwarded++;
    }
}