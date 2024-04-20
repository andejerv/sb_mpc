#include "SB_MPC.h"


int main(){

    // Init agent model
    BB_MODEL bb_model = BB_MODEL(0,0);

    SB_MPC mpc = SB_MPC(10, 1);

    std::vector<Object*> obstacles;

    obstacles.push_back(new Object(1, 1, 1));

    double hdg_os_best;
    double vel_os_best;

    mpc.getBestControlOffset(hdg_os_best, vel_os_best, 0, 1, &bb_model, &obstacles);


    for(int i = 0; i < obstacles.size(); i++){
        delete obstacles.at(i);
    }

    return 0;
}