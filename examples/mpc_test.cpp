#include "SB_MPC.h"


int main(){

    BB_MODEL bb_model = BB_MODEL(0,0);

    SB_MPC mpc = SB_MPC(10, 1);

    std::vector<Object*> obstacles;

    obstacles.push_back(new Object(0, 2, 1));

    double hdg_os_best;
    double vel_os_best;

    mpc.getBestControlOffset(hdg_os_best, vel_os_best, 0, 1, &bb_model, obstacles);

    std::cout << "Best heading offset: " << hdg_os_best << std::endl;


    for(int i = 0; i < obstacles.size(); i++){
        delete obstacles.at(i);
    }

    return 0;
}