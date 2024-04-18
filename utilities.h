#pragma once

#include <iostream>

struct coordinates
{
    double N;
    double E;
};

std::ostream& operator<<(std::ostream& os, const coordinates& coord);
