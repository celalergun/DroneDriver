#pragma once
#include "drone_state.h"
#include "drone_status.h"
#include <udp/simple_udp.h>

class Drone {
private:
    DroneState state;
    DroneStatus status;
public:
    Drone() = delete;
    Drone(int id, double local_y, double local_x, double altitude, double speed, double heading);
    void updateState(double local_y, double local_x, double altitude, double speed, double heading);
    std::string toString() const {
        return "ID: " + std::to_string(state.getId()) + "\n" +
               "Local Y: " + std::to_string(state.getLocal_y()) + "\n" +
               "Local X: " + std::to_string(state.getLocal_x()) + "\n" +
               "Altitude: " + std::to_string(state.getAltitude()) + "\n" +
               "Speed: " + std::to_string(state.getSpeed()) + "\n" +
               "Heading: " + std::to_string(state.getHeading()) + "\n" +
               "Status: " + std::to_string(static_cast<int>(status)) + "\n";
    }
};