#pragma once

#include <eigen3/Eigen/Dense>


// Chose integration solver
#define RungeKutta4 1
#define Linear 0

class Agent
{
    private:
        virtual Eigen::VectorXd dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input) = 0;
    public:
        Agent();
        ~Agent();
        
        Eigen::MatrixXd simulate_positions(const int num_steps, const double step_size, const Eigen::VectorXd& state, const Eigen::VectorXd& control_input);
};

class BB_MODEL: public Agent
{   
    private:
        Eigen::VectorXd dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input) override;
    public:
        BB_MODEL();
        ~BB_MODEL();        
};