#include "object.h"
#include <iostream>

Object::Object(double N, double E, double s): type(object_type::STATIC), global_N(N), global_E(E), size(s), velocity(0), heading(0){
    // Constructor
}

Object::Object(double N, double E, double s, double vel, double hdg): type(object_type::DYNAMIC), global_N(N), global_E(E), size(s), velocity(vel), heading(hdg){

}

Object::~Object(){
    // Destructor
}

// Position prediction function
std::unique_ptr<coordinates[]> Object::simulate_position(const int num_steps, const double step_size){
    std::unique_ptr<coordinates[]> position = std::make_unique<coordinates[]>(num_steps+1);

    switch (this->type)
    {
    case object_type::STATIC:
        for (int i = 0; i < num_steps+1; i++){
            position[i] = {global_N, global_E};
        }
        break;

    case object_type::DYNAMIC:
        position[0] = {global_N, global_E};
        for (int i = 1; i < num_steps+1; i++){
            position[i] = {position[i-1].N + velocity*cos(heading)*step_size, position[i-1].E + velocity*sin(heading)*step_size};
        }
        break;

    default:
        std::cout << "Invalid object type" << std::endl;
        break;
    }

    return position;
}