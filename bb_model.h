#pragma once

#include <eigen3/Eigen/Dense>

class BB_MODEL
{
    private:


    public:
        BB_MODEL();
        ~BB_MODEL();
        
        Eigen::MatrixXd simulate_position(int num_steps, double step_size);
        
};