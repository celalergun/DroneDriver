#ifndef DRONE_STATE_H
#define DRONE_STATE_H
class DroneState {
    private:
        int id;
        double local_y; // latitude
        double local_x; // longitude
        double altitude;
        double speed;
        double heading;
    public:
        DroneState() = delete;
        DroneState(int id, double local_y, double local_x, double altitude, double speed, double heading);
        int getId() const;
        double getLocal_y() const;
        double getLocal_x() const;
        double getAltitude() const;
        double getSpeed() const;
        double getHeading() const;
        void setLocal_y(double local_y);
        void setLocal_x(double local_x);
        void setAltitude(double altitude);
        void setSpeed(double speed);
        void setHeading(double heading);
};    
#endif // DRONE_STATE_H