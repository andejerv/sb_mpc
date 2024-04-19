#pragma once

#include <iostream>
#include <chrono>

struct coordinates
{
    double N;
    double E;
};

std::ostream& operator<<(std::ostream& os, const coordinates& coord);

class Stopwatch
{
    std::chrono::time_point<std::chrono::steady_clock> startTime;

    public:
        void start();
        double stop();
};
