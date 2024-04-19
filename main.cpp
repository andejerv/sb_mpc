#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "bb_model.h"
#include "object.h"

int main(){

    Object dynamic_obj = Object(0,0,1,1,0); // N, E, Size, Velocity, Heading

    Eigen::MatrixXd dynamic_pos = dynamic_obj.simulate_position(10, 1);

    std::cout << dynamic_pos << std::endl;

    BB_MODEL bb_model = BB_MODEL();

    Eigen::VectorXd state(4);
    state << 0, 0, 1, 0; // N, E, Velocity, Heading

    Eigen::VectorXd control_input(2);
    control_input << 1, 0; // Acceleration, Heading

    Eigen::MatrixXd positions = bb_model.simulate_positions(10, 1, state, control_input);

    std::cout << positions << std::endl;
    


    return 0;
}