#include "object.h"
#include <iostream>
#include <cmath>

Object::Object(double N, double E, double s): type(object_type::STATIC), global_N(N), global_E(E), size(s), velocity(0), heading(0){
    // Constructor
}

Object::Object(double N, double E, double s, double vel, double hdg): type(object_type::DYNAMIC), global_N(N), global_E(E), size(s), velocity(vel), heading(hdg){

}

Object::~Object(){
    // Destructor
}

// Position prediction function
Eigen::MatrixXd Object::simulate_positions(const int num_steps, const double step_size){
    Eigen::MatrixXd position(2, num_steps+1);

    switch (this->type)
    {
    case object_type::STATIC:
        for (int i = 0; i < num_steps+1; i++){
            position(0,i) = global_N;
            position(1,i) = global_E;
        }
        break;

    case object_type::DYNAMIC:
        position(0,0) = global_N;
        position(1,0) = global_E;
        for (int i = 1; i < num_steps+1; i++){
            position(0,i) = position(0,i-1) + velocity*std::cos(heading)*step_size;
            position(1,i) = position(1,i-1) + velocity*std::sin(heading)*step_size;
        }
        break;

    default:
        std::cout << "Invalid object type" << std::endl;
        break;
    }

    return position;
}

double Object::getGlobalN(){
    return this->global_N;
}

double Object::getGlobalE(){
    return this->global_E;
}

double Object::getSize(){
    return this->size;
}

double Object::getCollisionDistance(){
    return this->collisionDistance;
}

double Object::getSafeDistance(){
    return this->safeDistance;
}