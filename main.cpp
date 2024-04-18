#include <iostream>
#include <cmath>

#include "object.h"
#include "utilities.h"

int main(){

    Object dynamic_obj = Object(0,0,1,1,M_PI/2);
    std::unique_ptr<coordinates[]> dynamic_pos = dynamic_obj.simulate_position(10, 1);
    for (int i = 0; i < 11; i++){
        std::cout << dynamic_pos[i] << std::endl;
    }
    return 0;
}