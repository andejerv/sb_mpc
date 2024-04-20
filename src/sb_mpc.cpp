#include "sb_mpc.h"

SB_MPC::SB_MPC(int n_steps, double step_sz): num_steps(n_steps), step_size(step_sz){
    // Constructor
}

SB_MPC::~SB_MPC(){
    // Destructor
}

void SB_MPC::getBestControlOffset(double & hdg_os_best, double & vel_os_best, double hdg, double vel, BB_MODEL * usv, std::vector<Object*>& obstacles){
    double cost = INFINITY;
    hdg_os_best = 0;
    vel_os_best = 1;
    double cost_i = 0;
    int n_obst = obstacles.size();

    std::vector<double> costs;

    // If there are no obstacles, return with no offset
    if(n_obst == 0){
        return;
    }

    std::cout << "Detected " << n_obst << " obstacle(s)." << std::endl;

    // Create a vector of Eigen::MatrixXd to store the positions of the obstacles
    std::vector<Eigen::MatrixXd> obst_positions;
    for (int i = 0; i < n_obst; i++){
        obst_positions.push_back(obstacles.at(i)->simulate_positions(this->num_steps, this->step_size));
    }

    Eigen::MatrixXd sim_states;
    // For all possible heading offsets
    for (auto offset : hdg_offsets){
        // Simulate the vehicle with the offset
        std::cout << "Checking offset: " << offset << std::endl << std::endl;
        sim_states = usv->simulate_positions(this->num_steps, this->step_size, Eigen::Vector2d(vel, hdg + offset));
        std::cout << "Simulated states: " << std::endl << sim_states << std::endl << std::endl;
        cost_i = costFunction(sim_states, obst_positions, obstacles);
        costs.push_back(cost_i);
    }

    // Find lowest cost in cost vector
    for(int i = 0; i < costs.size(); i++){
        std::cout << "Cost: " << costs.at(i) << std::endl;
        if(costs.at(i) < cost){
            cost = costs.at(i);
            hdg_os_best = this->hdg_offsets[i];
        }
    }

    return;
}

double SB_MPC::costFunction(const Eigen::MatrixXd & sim_states, const std::vector<Eigen::MatrixXd> & obst_positions, std::vector<Object*>& obstacles){
    double cost = 0;

    // First two states should be (x,y) position in grid
    Eigen::MatrixXd sim_states_pos = sim_states.block(0, 0, 2, sim_states.cols());

    // For every obstacle
    for(int i = 0; i < obst_positions.size(); i++){
        Eigen::MatrixXd dist = obst_positions.at(i) - sim_states;
        std::cout << "Distance: " << std::endl << dist << std::endl << std::endl;
        

        for(int k = 0; k < dist.cols(); k++){
            cost += collisionRisk(k, dist.col(k).norm(), obstacles.at(i));
        }
    }

    return cost;
}

double SB_MPC::collisionRisk(int k, double dist, Object * obj){
    if(dist > obj->getSafeDistance()){
        return 0;
    }

    if(dist < obj->getCollisionDistance()){
        // FIXME This should be size * abs(speed difference vector)²
        return obj->getSize();
    }

    double time_risk = 1/(std::pow(k*this->step_size, this->time_to_collision_weigth));
    double distance_risk = std::pow(obj->getSafeDistance()/dist, this->distance_within_dsafe_weigth);

    return time_risk * distance_risk;
}