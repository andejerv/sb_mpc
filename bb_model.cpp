#include "bb_model.h"

Agent::Agent(){
    // Constructor
}

Agent::~Agent(){
    // Destructor
}

BB_MODEL::BB_MODEL(){
    // Constructor
}

BB_MODEL::~BB_MODEL(){
    // Destructor
}


Eigen::MatrixXd Agent::simulate_positions(const int num_steps, const double step_size, const Eigen::VectorXd & state, const Eigen::VectorXd & control_input){
    Eigen::MatrixXd positions = Eigen::MatrixXd::Zero(4, num_steps+1); //state matrix, rows=num_states, cols=initial+num_steps

    // Integrate the system
    positions.col(0) = state;
    for(int i = 1; i < num_steps+1; i++){

        #if RungeKutta4

            Eigen::VectorXd k1 = step_size * this->dxdt(positions.col(i-1), control_input);
            Eigen::VectorXd k2 = step_size * this->dxdt(positions.col(i-1) + 0.5*k1, control_input);
            Eigen::VectorXd k3 = step_size * this->dxdt(positions.col(i-1) + 0.5*k2, control_input);
            Eigen::VectorXd k4 = step_size * this->dxdt(positions.col(i-1) + k3, control_input);

            positions.col(i) = positions.col(i-1) + (k1 + 2*k2 + 2*k3 + k4)/6.0;

        #elif Linear

            state = positions.col(i-1);
            state = state + step_size * this->dxdt(state, control_input);
            positions.col(i) = state;

        #endif

    }

    return positions;
}


// Dynamic equations of the system
Eigen::VectorXd BB_MODEL::dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input){
    Eigen::VectorXd dxdt(4);
    dxdt(0) = state(2) * cos(state(3)); // N
    dxdt(1) = state(2) * sin(state(3)); // E
    dxdt(2) = control_input(0); // Velocity
    dxdt(3) = control_input(1); // Heading

    return dxdt;
}