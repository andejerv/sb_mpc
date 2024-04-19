#include "utilities.h"

std::ostream& operator<<(std::ostream& os, const coordinates& coord) {
    os << "N: " << coord.N << ", E: " << coord.E;
    return os;
}

void Stopwatch::start() {
    startTime = std::chrono::steady_clock::now();
}

double Stopwatch::stop() {
    std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
    long durationInMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    double durationInSeconds = double(durationInMicroseconds)/1000000.0;
    return durationInSeconds;
}