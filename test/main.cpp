#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../Drone/communicator.h"
#include "../GroundControl/reliable_sender.h"
#include <random>

// =============================================================================
// Mock Communicator Tests
// =============================================================================

class MockCommunicator : public ICommunicator {
public:
    MOCK_METHOD(void, sendMessage, (mavlink_message_t message), (override));
    MOCK_METHOD(void, onMessage, (uint32_t subscriber_id, std::function<void(const mavlink_message_t&)> callback), (override));
    MOCK_METHOD(void, run, (std::stop_token stoken), (override));
};

class CommunicatorTest : public ::testing::Test {
protected:
    MockCommunicator mock_comm;
};

TEST_F(CommunicatorTest, SendMessageCanBeCalled) {
    EXPECT_CALL(mock_comm, sendMessage(::testing::_)).Times(1);
    mavlink_message_t test_msg{};
    mock_comm.sendMessage(test_msg);
}

TEST_F(CommunicatorTest, OnMessageRegistersCallback) {
    EXPECT_CALL(mock_comm, onMessage(1, ::testing::_)).Times(1);
    auto callback = [](const mavlink_message_t& msg) {};
    mock_comm.onMessage(1, callback);
}

// =============================================================================
// Simulated lossy channel — drops packets at configurable rate, no UDP sockets
// =============================================================================

class SimulatedChannel {
public:
    SimulatedChannel(int drop_percent, unsigned seed = 42)
        : drop_percent_(drop_percent), rng_(seed) {}

    void setReceiver(std::function<void(const mavlink_message_t&)> cb) {
        receiver_ = std::move(cb);
    }

    void send(const mavlink_message_t& msg) {
        total_++;
        std::uniform_int_distribution<int> dist(0, 99);
        if (dist(rng_) < drop_percent_) {
            dropped_++;
            return;
        }
        forwarded_++;
        if (receiver_) receiver_(msg);
    }

    int dropped() const { return dropped_; }
    int forwarded() const { return forwarded_; }
    int total() const { return total_; }

private:
    int drop_percent_;
    std::mt19937 rng_;
    std::function<void(const mavlink_message_t&)> receiver_;
    int dropped_ = 0;
    int forwarded_ = 0;
    int total_ = 0;
};

// =============================================================================
// Simulated drone — handles incoming commands, sends ACKs back through channel
// =============================================================================

class SimulatedDrone {
public:
    explicit SimulatedDrone(SimulatedChannel& response_channel)
        : channel_(response_channel) {}

    void handleMessage(const mavlink_message_t& msg) {
        messages_received_++;
        switch (msg.msgid) {
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {
            mavlink_set_position_target_global_int_t target;
            mavlink_msg_set_position_target_global_int_decode(&msg, &target);
            last_target_lat_ = target.lat_int;
            last_target_lon_ = target.lon_int;

            // ACK with POSITION_TARGET_GLOBAL_INT
            mavlink_message_t ack;
            mavlink_msg_position_target_global_int_pack(
                1, 1, &ack,
                0, MAV_FRAME_GLOBAL_INT,
                target.type_mask,
                target.lat_int, target.lon_int, target.alt,
                0, 0, 0, 0, 0, 0, 0, 0
            );
            channel_.send(ack);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);

            // ACK with COMMAND_ACK
            mavlink_message_t ack;
            mavlink_msg_command_ack_pack(
                1, 1, &ack,
                cmd.command,
                MAV_RESULT_ACCEPTED,
                100, 0,
                msg.sysid, msg.compid
            );
            channel_.send(ack);
            break;
        }
        }
    }

    int messagesReceived() const { return messages_received_; }
    int32_t lastTargetLat() const { return last_target_lat_; }
    int32_t lastTargetLon() const { return last_target_lon_; }

private:
    SimulatedChannel& channel_;
    int messages_received_ = 0;
    int32_t last_target_lat_ = 0;
    int32_t last_target_lon_ = 0;
};

// =============================================================================
// MAVLink encode/decode tests — verify communication patterns
// =============================================================================

TEST(MavlinkPatterns, HeartbeatEncodeDecode) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, 1, &msg,
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
        MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);

    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    EXPECT_EQ(hb.type, MAV_TYPE_QUADROTOR);
    EXPECT_EQ(hb.autopilot, MAV_AUTOPILOT_GENERIC);
    EXPECT_EQ(hb.base_mode, MAV_MODE_GUIDED_ARMED);
    EXPECT_EQ(hb.system_status, MAV_STATE_ACTIVE);
}

TEST(MavlinkPatterns, GlobalPositionIntEncodeDecode) {
    mavlink_message_t msg;
    int32_t lat = static_cast<int32_t>(59.424100 * 1e7);
    int32_t lon = static_cast<int32_t>(24.813240 * 1e7);
    mavlink_msg_global_position_int_pack(1, 1, &msg,
        0, lat, lon, 100000, 100000, 500, 0, 0, 9000);

    mavlink_global_position_int_t gpos;
    mavlink_msg_global_position_int_decode(&msg, &gpos);
    EXPECT_EQ(gpos.lat, lat);
    EXPECT_EQ(gpos.lon, lon);
    EXPECT_EQ(gpos.alt, 100000);
    EXPECT_EQ(gpos.vx, 500);
    EXPECT_EQ(gpos.hdg, 9000);
}

TEST(MavlinkPatterns, SetPositionTargetEncodeDecode) {
    mavlink_message_t msg;
    int32_t lat = static_cast<int32_t>(59.424100 * 1e7);
    int32_t lon = static_cast<int32_t>(24.813240 * 1e7);
    mavlink_msg_set_position_target_global_int_pack(
        255, 190, &msg, 0, 1, 0,
        MAV_FRAME_GLOBAL_INT, 0b0000111111111000,
        lat, lon, 100.0f,
        0, 0, 0, 0, 0, 0, 0, 0);

    mavlink_set_position_target_global_int_t target;
    mavlink_msg_set_position_target_global_int_decode(&msg, &target);
    EXPECT_EQ(target.lat_int, lat);
    EXPECT_EQ(target.lon_int, lon);
    EXPECT_FLOAT_EQ(target.alt, 100.0f);
    EXPECT_EQ(target.coordinate_frame, MAV_FRAME_GLOBAL_INT);
}

TEST(MavlinkPatterns, CommandLongHoldEncodeDecode) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(255, 190, &msg, 1, 0,
        MAV_CMD_OVERRIDE_GOTO, 0, MAV_GOTO_DO_HOLD, 0, 0, 0, 0, 0, 0);

    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    EXPECT_EQ(cmd.command, MAV_CMD_OVERRIDE_GOTO);
    EXPECT_FLOAT_EQ(cmd.param1, MAV_GOTO_DO_HOLD);
}

TEST(MavlinkPatterns, CommandAckEncodeDecode) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(1, 1, &msg,
        MAV_CMD_OVERRIDE_GOTO, MAV_RESULT_ACCEPTED, 100, 0, 255, 190);

    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    EXPECT_EQ(ack.command, MAV_CMD_OVERRIDE_GOTO);
    EXPECT_EQ(ack.result, MAV_RESULT_ACCEPTED);
}

TEST(MavlinkPatterns, PositionTargetGlobalIntAckEncodeDecode) {
    mavlink_message_t msg;
    int32_t lat = static_cast<int32_t>(59.424100 * 1e7);
    int32_t lon = static_cast<int32_t>(24.813240 * 1e7);
    mavlink_msg_position_target_global_int_pack(
        1, 1, &msg, 0, MAV_FRAME_GLOBAL_INT,
        0b0000111111111000, lat, lon, 100.0f,
        0, 0, 0, 0, 0, 0, 0, 0);

    mavlink_position_target_global_int_t ptgi;
    mavlink_msg_position_target_global_int_decode(&msg, &ptgi);
    EXPECT_EQ(ptgi.lat_int, lat);
    EXPECT_EQ(ptgi.lon_int, lon);
    EXPECT_FLOAT_EQ(ptgi.alt, 100.0f);
}

// =============================================================================
// Reliable delivery tests — no packet loss baseline
// =============================================================================

TEST(ReliableDelivery, PositionTargetAckedNoLoss) {
    SimulatedChannel gcs_to_drone(0);  // 0% drop
    SimulatedChannel drone_to_gcs(0);

    SimulatedDrone drone(drone_to_gcs);
    gcs_to_drone.setReceiver([&drone](const mavlink_message_t& msg) {
        drone.handleMessage(msg);
    });

    ReliableSender sender(
        [&gcs_to_drone](const mavlink_message_t& msg) { gcs_to_drone.send(msg); },
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(500)
    );
    drone_to_gcs.setReceiver([&sender](const mavlink_message_t& msg) {
        sender.onAckReceived(msg);
    });

    int32_t lat = static_cast<int32_t>(59.424100 * 1e7);
    int32_t lon = static_cast<int32_t>(24.813240 * 1e7);
    bool acked = sender.sendPositionTargetBlocking(lat, lon, 100.0f);

    EXPECT_TRUE(acked);
    EXPECT_GE(drone.messagesReceived(), 1);
    EXPECT_EQ(drone.lastTargetLat(), lat);
    EXPECT_EQ(drone.lastTargetLon(), lon);
}

TEST(ReliableDelivery, HoldCommandAckedNoLoss) {
    SimulatedChannel gcs_to_drone(0);
    SimulatedChannel drone_to_gcs(0);

    SimulatedDrone drone(drone_to_gcs);
    gcs_to_drone.setReceiver([&drone](const mavlink_message_t& msg) {
        drone.handleMessage(msg);
    });

    ReliableSender sender(
        [&gcs_to_drone](const mavlink_message_t& msg) { gcs_to_drone.send(msg); },
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(500)
    );
    drone_to_gcs.setReceiver([&sender](const mavlink_message_t& msg) {
        sender.onAckReceived(msg);
    });

    bool acked = sender.sendHoldCommandBlocking();
    EXPECT_TRUE(acked);
    EXPECT_GE(drone.messagesReceived(), 1);
}

// =============================================================================
// Stress tests with 75% packet loss
// =============================================================================

TEST(StressTest75, PositionTargetAcked) {
    SimulatedChannel gcs_to_drone(75, 123);
    SimulatedChannel drone_to_gcs(75, 456);

    SimulatedDrone drone(drone_to_gcs);
    gcs_to_drone.setReceiver([&drone](const mavlink_message_t& msg) {
        drone.handleMessage(msg);
    });

    ReliableSender sender(
        [&gcs_to_drone](const mavlink_message_t& msg) { gcs_to_drone.send(msg); },
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(2000)
    );
    drone_to_gcs.setReceiver([&sender](const mavlink_message_t& msg) {
        sender.onAckReceived(msg);
    });

    int32_t lat = static_cast<int32_t>(59.424100 * 1e7);
    int32_t lon = static_cast<int32_t>(24.813240 * 1e7);
    bool acked = sender.sendPositionTargetBlocking(lat, lon, 100.0f);

    EXPECT_TRUE(acked) << "Position target must be ACKed even at 75% loss";
    EXPECT_GE(drone.messagesReceived(), 1);
    EXPECT_EQ(drone.lastTargetLat(), lat);
    EXPECT_EQ(drone.lastTargetLon(), lon);
}

TEST(StressTest75, HoldCommandAcked) {
    SimulatedChannel gcs_to_drone(75, 789);
    SimulatedChannel drone_to_gcs(75, 101);

    SimulatedDrone drone(drone_to_gcs);
    gcs_to_drone.setReceiver([&drone](const mavlink_message_t& msg) {
        drone.handleMessage(msg);
    });

    ReliableSender sender(
        [&gcs_to_drone](const mavlink_message_t& msg) { gcs_to_drone.send(msg); },
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(2000)
    );
    drone_to_gcs.setReceiver([&sender](const mavlink_message_t& msg) {
        sender.onAckReceived(msg);
    });

    bool acked = sender.sendHoldCommandBlocking();
    EXPECT_TRUE(acked) << "Hold command must be ACKed even at 75% loss";
    EXPECT_GE(drone.messagesReceived(), 1);
}

TEST(StressTest75, MultipleSequentialCommands) {
    SimulatedChannel gcs_to_drone(75, 202);
    SimulatedChannel drone_to_gcs(75, 303);

    SimulatedDrone drone(drone_to_gcs);
    gcs_to_drone.setReceiver([&drone](const mavlink_message_t& msg) {
        drone.handleMessage(msg);
    });

    ReliableSender sender(
        [&gcs_to_drone](const mavlink_message_t& msg) { gcs_to_drone.send(msg); },
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(2000)
    );
    drone_to_gcs.setReceiver([&sender](const mavlink_message_t& msg) {
        sender.onAckReceived(msg);
    });

    // Send 10 sequential position commands — all must succeed
    for (int i = 0; i < 10; i++) {
        int32_t lat = static_cast<int32_t>((59.420 + i * 0.001) * 1e7);
        int32_t lon = static_cast<int32_t>((24.810 + i * 0.001) * 1e7);
        bool acked = sender.sendPositionTargetBlocking(lat, lon, 100.0f);
        EXPECT_TRUE(acked) << "Command " << i << " must be ACKed at 75% loss";
    }

    // Send 5 hold commands — all must succeed
    for (int i = 0; i < 5; i++) {
        bool acked = sender.sendHoldCommandBlocking();
        EXPECT_TRUE(acked) << "Hold " << i << " must be ACKed at 75% loss";
    }
}

TEST(StressTest75, PacketDropStatistics) {
    // Verify the simulated channel actually drops ~75% of packets
    SimulatedChannel channel(75, 999);
    int received = 0;
    channel.setReceiver([&received](const mavlink_message_t&) { received++; });

    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(1, 1, &msg,
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
        MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);

    const int total = 10000;
    for (int i = 0; i < total; i++) {
        channel.send(msg);
    }

    double drop_rate = 100.0 * channel.dropped() / total;
    // Should be approximately 75% ± 3%
    EXPECT_NEAR(drop_rate, 75.0, 3.0)
        << "Simulated channel should drop ~75% of packets";
    EXPECT_EQ(channel.total(), total);
    EXPECT_EQ(channel.forwarded() + channel.dropped(), total);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
