#include "object.h"

Object::Object(double N, double E, double s): type(object_type::STATIC), global_N(N), global_E(E), size(s), velocity(0), heading(0){
    // Constructor
}

Object::Object(double N, double E, double s, double vel, double hdg): type(object_type::DYNAMIC), global_N(N), global_E(E), size(s), velocity(vel), heading(hdg){

}

Object::~Object(){
    // Destructor
}