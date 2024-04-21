#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>

#include "utilities.h"

enum object_type
{
    STATIC,
    DYNAMIC
};

class Object
{
    private:
    object_type type;
    double global_N;
    double global_E;
    double size;
    double velocity;
    double heading;

    double collisionDistance = 0; // Distances lower than this counts as collision
    double safeDistance = 1; // Distance higher than this gives 0 collision risk

    public:
    // FIXME - Add a constructor that takes collision and safe distance into account
        Object(double N, double E, double s);
        Object(double N, double E, double s, double vel, double hdg);
        ~Object();

        Eigen::MatrixXd simulate_positions(int num_steps, double step_size);

        double getGlobalN();
        double getGlobalE();
        double getSize();
        double getCollisionDistance();
        double getSafeDistance();
        
};