#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "bb_model.h"
#include "object.h"
#include "utilities.h"

int main(){
    Stopwatch sw;

    BB_MODEL bb_model = BB_MODEL(0,0);

    Eigen::VectorXd control_input(2);
    control_input << 1, (static_cast<double>(3)/2)*M_PI; // Acceleration, Heading

    int num_iterations = 100;
    Eigen::MatrixXd positions;
    sw.start();
    for (int i = 0; i < num_iterations; i++){
        positions = bb_model.simulate_positions(10, 1, control_input);
    }
    double duration = sw.stop()/num_iterations;

    std::cout << "Duration: " << duration << " seconds" << std::endl;

    std::cout << positions << std::endl;
    
    


    return 0;
}