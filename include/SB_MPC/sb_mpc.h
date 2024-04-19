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

        double hdg_offsets[5] = {-10, -5, 0, 5, 10};
        //double vel_offsets[7] = {0.5, 1, 1.5, 2};

    public:
        SB_MPC(int n_steps, double step_sz);
        ~SB_MPC();

        void getBestControlOffset(double & hdg_os_best, double & vel_os_best, double hdg, double vel, BB_MODEL * usv, std::vector<Object*>* obstacles);
        double costFunction();
};