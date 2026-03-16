#include "drone.h"

Drone::Drone(int id, double local_y, double local_x, double altitude, double speed, double heading) : state(id, local_y, local_x, altitude, speed, heading) {
    status = DroneStatus::MAV_MODE_GUIDED_ARMED; // Default status when a drone is created
}

void Drone::updateState(double local_y, double local_x, double altitude, double speed, double heading) {
    // Update the drone's state with the new values
    state.setLocal_y(local_y);
    state.setLocal_x(local_x);
    state.setAltitude(altitude);
    state.setSpeed(speed);
    state.setHeading(heading);
}
