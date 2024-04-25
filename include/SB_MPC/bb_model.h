#pragma once

#include <eigen3/Eigen/Dense>


// Chose integration solver, uncomment one of the following
#define RungeKutta4
//#define EulerForward

namespace sbmpc{

class Agent
{
    private:
        virtual Eigen::VectorXd dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input) = 0;

    protected:
        Eigen::Vector2d state;

    public:
        Agent();
        ~Agent();

        void setState(const Eigen::VectorXd& _state);
        Eigen::VectorXd getState();

        Eigen::MatrixXd simulate_positions(const int num_steps, const double step_size, const Eigen::VectorXd& control_input);

};

class BB_MODEL: public Agent
{   
    private:
        Eigen::VectorXd dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input) override;

    public:
        BB_MODEL(double _x, double _y);
        ~BB_MODEL();

};

} // namespace sbmpc