#pragma once

#include <eigen3/Eigen/Dense>


// Chose integration solver, uncomment one of the following
#define RungeKutta4
//#define EulerForward


class Agent
{
    private:
        virtual Eigen::VectorXd dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input) = 0;

    protected:
        Eigen::VectorXd _state;

    public:
        Agent();
        ~Agent();

        void setState(const Eigen::VectorXd& state);
        Eigen::VectorXd getState();

        Eigen::MatrixXd simulate_positions(const int num_steps, const double step_size, const Eigen::VectorXd& control_input);

};

class BB_MODEL: public Agent
{   
    private:
        Eigen::VectorXd dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input) override;

    public:
        BB_MODEL(double _N, double _E);
        ~BB_MODEL();

};