#include "bb_model.h"

Agent::Agent(){
    // Constructor
}

Agent::~Agent(){
    // Destructor
}


void Agent::setState(const Eigen::VectorXd & state){
    _state = state;
}

Eigen::VectorXd Agent::getState(){
    return _state;
}


Eigen::MatrixXd Agent::simulate_positions(const int num_steps, const double step_size, const Eigen::VectorXd & control_input){
    Eigen::MatrixXd positions = Eigen::MatrixXd::Zero(this->_state.rows(), num_steps+1); //state matrix, rows=num_states, cols=initial+num_steps

    // Integrate the system
    positions.col(0) = this->_state;
    for(int i = 1; i < num_steps+1; i++){

        #ifdef RungeKutta4

            Eigen::VectorXd k1 = step_size * this->dxdt(positions.col(i-1), control_input);
            Eigen::VectorXd k2 = step_size * this->dxdt(positions.col(i-1) + 0.5*k1, control_input);
            Eigen::VectorXd k3 = step_size * this->dxdt(positions.col(i-1) + 0.5*k2, control_input);
            Eigen::VectorXd k4 = step_size * this->dxdt(positions.col(i-1) + k3, control_input);

            positions.col(i) = positions.col(i-1) + (k1 + 2*k2 + 2*k3 + k4)/6.0;
        #endif
        
        #ifdef EulerForward

            positions.col(i) = positions.col(i-1) + step_size * this->dxdt(state, control_input);
            
        #endif

    }

    return positions;
}


BB_MODEL::BB_MODEL(double _N, double _E){
    _state = Eigen::VectorXd(2);
    _state << _N, _E;
}

BB_MODEL::~BB_MODEL(){
    // Destructor
}


// Dynamic equations of the system
Eigen::VectorXd BB_MODEL::dxdt(const Eigen::VectorXd& state, const Eigen::VectorXd& control_input){
    Eigen::VectorXd dxdt(state.rows());
    dxdt(0) = control_input(0) * cos(control_input(1)); // N
    dxdt(1) = control_input(0) * sin(control_input(1)); // E

    return dxdt;
}