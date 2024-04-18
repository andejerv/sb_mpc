#include "bb_model.h"

BB_MODEL::BB_MODEL(){
    // Constructor
}

BB_MODEL::~BB_MODEL(){
    // Destructor
}

Eigen::MatrixXd BB_MODEL::simulate_position(const int num_steps, const double step_size){
    Eigen::MatrixXd position(2, num_steps+1);

    //TODO

    return position;
}