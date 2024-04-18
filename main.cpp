#include <iostream>
#include <cmath>

#include "object.h"

int main(){

    Object dynamic_obj = Object(0,0,1,1,0); // N, E, Size, Velocity, Heading

    Eigen::MatrixXd dynamic_pos = dynamic_obj.simulate_position(10, 1);

    std::cout << dynamic_pos << std::endl;

    return 0;
}