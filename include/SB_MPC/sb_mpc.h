#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

#include "agent.h"
#include "object.h"

namespace sbmpc{

class SB_MPC
{

    private:
        int num_steps = 0;
        double step_size = 0;

        double hdg_offsets[9] = {deg2rad(-90), deg2rad(-45), deg2rad(-30), deg2rad(-15), deg2rad(0), deg2rad(15), deg2rad(30), deg2rad(55), deg2rad(90)}; // Offsets in rads
        double vel_offsets[3] = {0, 0.5, 1};

        // Saftey parameters
        double time_to_collision_weigth = 2.0;
        double distance_within_dsafe_weigth = 3;
        double turning_penalty_constant = 1.5;
        double port_penalty_constant = 1.5;
        double starboard_penalty_constant = 1.0;
        double slow_speed_penalty_constant = 3.0;


    public:
        SB_MPC(int n_steps, double step_sz);
        ~SB_MPC();

        void getBestControlOffset(double & hdg_os_best, double & vel_os_best, double hdg, double vel, BB_MODEL * usv, std::vector<Object*>& obstacles);
        double costFunction(const Eigen::MatrixXd & sim_states, const std::vector<Eigen::MatrixXd> & obst_positions, std::vector<Object*>& obstacles);
        double collisionRisk(int k, double dist, Object * obj);
};

} // namespace sbmpc