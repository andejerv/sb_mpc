#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <chrono>

namespace sbmpc{

enum IntegrationMethod
{
    RK4,
    EULER
};

struct coordinates
{
    double N;
    double E;
};

std::ostream& operator<<(std::ostream& os, const coordinates& coord);

class Stopwatch
{
    std::chrono::time_point<std::chrono::steady_clock> startTime;

    public:
        void start();
        double stop();
};

double deg2rad(double deg);
Eigen::Matrix3d ned2cartesian(const Eigen::Matrix3d & ned);
double waypointToHeading(Eigen::Vector2d pos, Eigen::Vector2d wp);

} // namespace sbmpc