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

    // If there are no obstacles, return
    if(n_obst == 0){
        return;
    }

    return;
}