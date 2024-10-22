#pragma once

#include <vector>
#include "../Color/Color.h"

struct NodeColorData {
    std::vector<Color> availableColors;
    uint remainingColors;
};