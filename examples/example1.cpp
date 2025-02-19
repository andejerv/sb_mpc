#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "SB_MPC.h"

Stopwatch sw;

int main(){
    // Init agent model
    BB_MODEL bb_model = BB_MODEL(0,0);

    // Create control inputs
    Eigen::VectorXd control_input(2);
    control_input << 1, (static_cast<double>(3)/2)*M_PI; // Acceleration, Heading

    // Check execution time of position simulation
    int num_iterations = 100;
    Eigen::MatrixXd positions;

    sw.start();

    for (int i = 0; i < num_iterations; i++){
        positions = bb_model.simulate_positions(1000, 0.1, control_input); // Simulate 1000 steps with 0.1s step size
    }

    double duration = sw.stop()/num_iterations;

    // Print results
    std::cout << "Duration: " << duration << " seconds" << std::endl;
    //std::cout << positions << std::endl; A bit much to print 1000x2 matrix ;)
    
    

    return 0;
}