#pragma once

#include <memory>

#include "eigen3/Eigen/Dense"
#include "utilities.h"

/*
TODO:
- Return list of simulated positions

*/

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

    public:
        Object(double N, double E, double s);
        Object(double N, double E, double s, double vel, double hdg);
        ~Object();

        Eigen::MatrixXd simulate_position(int num_steps, double step_size);
        
};