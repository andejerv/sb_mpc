#include "object.h"
#include <iostream>
#include <cmath>

namespace sbmpc{

Object::Object(double _x, double _y, double s, double collisionDist, double safeDist)
:type(object_type::STATIC), x(_x), y(_y), size(s), collisionDistance(collisionDist), safeDistance(safeDist), velocity(0), heading(0)
{
    // Constructor
}

Object::Object(double _x, double _y, double s , double collisionDist, double safeDist, double vel, double hdg) 
:type(DYNAMIC), method(EULER), x(_x), y(_y), size(s), collisionDistance(collisionDist), safeDistance(safeDist), velocity(vel), heading(hdg)
{
    // Contructor
}

Object::Object(double _x, double _y, double s , double collisionDist, double safeDist, double vel, double hdg, IntegrationMethod m) 
:type(object_type::DYNAMIC), method(m), x(_x), y(_y), size(s), collisionDistance(collisionDist), safeDistance(safeDist), velocity(vel), heading(hdg)
{
    // Contructor
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
            position(0,i) = x;
            position(1,i) = y;
        }
        break;

    case object_type::DYNAMIC:
        position(0,0) = x;
        position(1,0) = y;
        for (int i = 1; i < num_steps+1; i++){
            position(0,i) = position(0,i-1) + this->velocity*std::cos(this->heading)*step_size;
            position(1,i) = position(1,i-1) + this->velocity*std::sin(this->heading)*step_size;
        }
        break;

    default:
        std::cout << "Invalid object type" << std::endl;
        break;
    }

    return position;
}

double Object::getX(){
    return this->x;
}

double Object::getY(){
    return this->y;
}

double Object::getSize(){
    return this->size;
}

object_type Object::getType(){
    return this->type;
}

double Object::getVelocity(){
    return this->velocity;
}

double Object::getHeading(){
    return this->heading;
}

double Object::getCollisionDistance(){
    return this->collisionDistance;
}

double Object::getSafeDistance(){
    return this->safeDistance;
}

void Object::setX(double _x){
    this->x = _x;
}

void Object::setY(double _y){
    this->y = _y;
}

void Object::setSize(double s){
    this->size = s;
}

void Object::setType(object_type t){
    this->type = t;
}

void Object::setVelocity(double vel){
    this->velocity = vel;
}

void Object::setHeading(double hdg){
    this->heading = hdg;
}

void Object::setCollisionDistance(double collisionDist){
    this->collisionDistance = collisionDist;
}

void Object::setSafeDistance(double safeDist){
    this->safeDistance = safeDist;
}

} // namespace sbmpc