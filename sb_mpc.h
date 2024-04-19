#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

#include "bb_model.h"
#include "object.h"

class SB_MPC
{

    private:
        int num_steps = 0;
        double step_size = 0;

    public:
        SB_MPC(int n_steps, double step_sz);
        ~SB_MPC();

        void getBestControlOffset(double & hdg_os_best, double & vel_os_best, double hdg, double vel, BB_MODEL * usv, std::vector<Object*>* obstacles);
        
};