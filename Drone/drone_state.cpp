#include "drone_state.h"

DroneState::DroneState(int id, double local_y, double local_x, double altitude, double speed, double heading) : 
    id(id), local_y(local_y), local_x(local_x), altitude(altitude), speed(speed), heading(heading) {

    }

int DroneState::getId() const {
    return id;
}

double DroneState::getLocal_y() const {
    return local_y;
}

double DroneState::getLocal_x() const {
    return local_x;
}

double DroneState::getAltitude() const {
    return altitude;
}

double DroneState::getSpeed() const {
    return speed;
}

double DroneState::getHeading() const {
    return heading;
}

void DroneState::setLocal_y(double local_y) {
    this->local_y = local_y;
}

void DroneState::setLocal_x(double local_x) {
    this->local_x = local_x;
}

void DroneState::setAltitude(double altitude) {
    this->altitude = altitude;
}

void DroneState::setSpeed(double speed) {
    this->speed = speed;
}

void DroneState::setHeading(double heading) {
    this->heading = heading;
}

