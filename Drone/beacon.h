#ifndef BEACON_H
#define BEACON_H
#include "communicator.h"
#include <thread>
#include <chrono>
#include <mavlink/common/mavlink.h>

class Beacon {
public:
    void start(Communicator& comm, std::stop_token stoken, std::atomic<uint8_t>& drone_mode);
private:
    std::jthread beacon_thread;
};
#endif // BEACON_H