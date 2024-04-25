#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>

#include "utilities.h"

namespace sbmpc{

enum object_type
{
    STATIC,
    DYNAMIC
};

class Object
{
    private:
    object_type type;
    double x;
    double y;
    double size;
    double velocity;
    double heading;

    double collisionDistance; // Distances lower than this counts as collision
    double safeDistance; // Distance higher than this gives 0 collision risk

    public:
    // FIXME - Add a constructor that takes collision and safe distance into account
        Object(double _x, double _y, double s, double collisionDist, double safeDist);
        Object(double _x, double _y, double s, double collisionDist, double safeDist, double vel, double hdg);
        ~Object();

        Eigen::MatrixXd simulate_positions(int num_steps, double step_size);

        double getX();
        double getY();
        double getSize();
        object_type getType();
        double getVelocity();
        double getHeading();
        double getCollisionDistance();
        double getSafeDistance();

        void setX(double _x);
        void setY(double _y);
        void setSize(double s);
        void setType(object_type t);
        void setVelocity(double vel);
        void setHeading(double hdg);
        void setCollisionDistance(double collisionDist);
        void setSafeDistance(double safeDist);
        
};

} // namespace sbmpc