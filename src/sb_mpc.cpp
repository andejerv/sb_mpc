#include "sb_mpc.h"

SB_MPC::SB_MPC(int n_steps, double step_sz): num_steps(n_steps), step_size(step_sz){
    // Constructor
}

SB_MPC::~SB_MPC(){
    // Destructor
}

void SB_MPC::getBestControlOffset(double & hdg_os_best, double & vel_os_best, double hdg, double vel, BB_MODEL * usv, std::vector<Object*>* obstacles){
    double cost = INFINITY;
    hdg_os_best = 0;
    vel_os_best = 1;
    int n_obst = obstacles->size();

    // If there are no obstacles, return with no offset
    if(n_obst == 0){
        return;
    }

    Eigen::MatrixXd sim_states;
    // For all possible heading offsets
    for (auto offset : hdg_offsets){
        // Simulate the vehicle with the offset
        sim_states = usv->simulate_positions(this->num_steps, this->step_size, Eigen::Vector2d(vel, hdg + offset));

    }


    return;
}

double SB_MPC::costFunction(){
    return 0;
}