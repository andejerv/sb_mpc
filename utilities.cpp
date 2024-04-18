#include "utilities.h"

std::ostream& operator<<(std::ostream& os, const coordinates& coord) {
    os << "N: " << coord.N << ", E: " << coord.E;
    return os;
}