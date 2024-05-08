#include "utilities.h"

#include <cmath>

namespace sbmpc{

std::ostream& operator<<(std::ostream& os, const coordinates& coord) {
    os << "N: " << coord.N << ", E: " << coord.E;
    return os;
}

void Stopwatch::start() {
    startTime = std::chrono::steady_clock::now();
}

double Stopwatch::stop() {
    std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
    long durationInMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    double durationInSeconds = double(durationInMicroseconds)/1000000.0;
    return durationInSeconds;
}

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

Eigen::Matrix3d ned2cartesian(const Eigen::Matrix3d & ned){
    double theta = M_PI/2;
    Eigen::Matrix3d rot_z;
    rot_z << cos(theta), -sin(theta), 0,
             sin(theta),  cos(theta), 0,
             0,           0,          1;

    double phi = -M_PI;
    Eigen::Matrix3d rot_y;
    rot_y << cos(phi),  0, sin(phi),
             0,         1, 0,
            -sin(phi),  0, cos(phi);

    Eigen::Matrix3d rot = rot_y * rot_z;

    Eigen::Matrix3d cartesian = rot * ned;

    return cartesian;
}

double waypointToHeading(Eigen::Vector2d pos, Eigen::Vector2d wp){
    Eigen::Vector2d diff = wp - pos;
    return atan2(diff(1), diff(0));
}

} // namespace sbmpc