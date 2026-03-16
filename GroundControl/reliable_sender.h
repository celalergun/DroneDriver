#pragma once
#include <mavlink/common/mavlink.h>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <iostream>

// Retry-until-ACK mechanism for MAVLink commands.
// Resends command at regular intervals until the matching ACK is received or timeout.
class ReliableSender {
public:
    using SendFunc = std::function<void(const mavlink_message_t&)>;

    explicit ReliableSender(SendFunc send_func,
                            std::chrono::milliseconds retry_interval = std::chrono::milliseconds(100),
                            std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
        : send_func_(std::move(send_func))
        , retry_interval_(retry_interval)
        , timeout_(timeout)
    {}

    ~ReliableSender() {
        if (retry_thread_.joinable()) retry_thread_.join();
    }

    // Called from receive/callback thread when any message arrives from the drone.
    // Checks if it matches a pending ACK.
    void onAckReceived(const mavlink_message_t& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (waiting_for_position_ack_ && msg.msgid == MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT) {
            waiting_for_position_ack_ = false;
            position_acked_ = true;
            cv_.notify_all();
        }
        if (waiting_for_command_ack_ && msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            if (ack.command == pending_command_id_) {
                waiting_for_command_ack_ = false;
                command_acked_ = true;
                cv_.notify_all();
            }
        }
    }

    // Non-blocking: sends SET_POSITION_TARGET_GLOBAL_INT with retries in background
    void sendPositionTarget(int32_t lat_int, int32_t lon_int, float alt) {
        mavlink_message_t msg;
        mavlink_msg_set_position_target_global_int_pack(
            255, 190, &msg,
            0, 1, 0,
            MAV_FRAME_GLOBAL_INT,
            0b0000111111111000, // https://groups.google.com/g/mavlink/c/dgGkBUsDgzQ?pli=1
            lat_int, lon_int, alt,
            0, 0, 0, 0, 0, 0, 0, 0
        );

        if (retry_thread_.joinable()) retry_thread_.join();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            waiting_for_position_ack_ = true;
            position_acked_ = false;
        }

        retry_thread_ = std::thread([this, msg]() {
            retryLoop(msg, waiting_for_position_ack_);
        });
    }

    // Non-blocking: sends MAV_CMD_OVERRIDE_GOTO (HOLD) with retries in background
    void sendHoldCommand() {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(
            255, 190, &msg,
            1, 0,
            MAV_CMD_OVERRIDE_GOTO,
            0,
            MAV_GOTO_DO_HOLD,
            0, 0, 0, 0, 0, 0
        );

        if (retry_thread_.joinable()) retry_thread_.join();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            waiting_for_command_ack_ = true;
            command_acked_ = false;
            pending_command_id_ = MAV_CMD_OVERRIDE_GOTO;
        }

        retry_thread_ = std::thread([this, msg]() {
            retryLoop(msg, waiting_for_command_ack_);
        });
    }

    void waitForCompletion() {
        if (retry_thread_.joinable()) retry_thread_.join();
    }

    // Blocking: sends position target and waits for ACK. Returns true if ACKed.
    bool sendPositionTargetBlocking(int32_t lat_int, int32_t lon_int, float alt) {
        sendPositionTarget(lat_int, lon_int, alt);
        waitForCompletion();
        std::lock_guard<std::mutex> lock(mutex_);
        return position_acked_;
    }

    // Blocking: sends hold command and waits for ACK. Returns true if ACKed.
    bool sendHoldCommandBlocking() {
        sendHoldCommand();
        waitForCompletion();
        std::lock_guard<std::mutex> lock(mutex_);
        return command_acked_;
    }

private:
    void retryLoop(const mavlink_message_t& msg, bool& waiting_flag) {
        auto start = std::chrono::steady_clock::now();
        while (true) {
            send_func_(msg);

            std::unique_lock<std::mutex> lock(mutex_);
            if (cv_.wait_for(lock, retry_interval_, [&] { return !waiting_flag; })) {
                return; // ACKed
            }

            if (std::chrono::steady_clock::now() - start > timeout_) {
                waiting_flag = false;
                return; // Timeout
            }
        }
    }

    SendFunc send_func_;
    std::chrono::milliseconds retry_interval_;
    std::chrono::milliseconds timeout_;

    std::mutex mutex_;
    std::condition_variable cv_;

    bool waiting_for_position_ack_ = false;
    bool waiting_for_command_ack_ = false;
    bool position_acked_ = false;
    bool command_acked_ = false;
    uint16_t pending_command_id_ = 0;

    std::thread retry_thread_;
};
