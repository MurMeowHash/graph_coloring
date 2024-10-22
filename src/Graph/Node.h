#pragma once

#include <vector>
#include "../utils/utils.h"

class Node {
private:
    static constexpr uint DEFAULT_COLOR{1};
    uint color;
    const char *name;
public:
    explicit Node(const char *name, uint color = DEFAULT_COLOR);
    void setColor(uint clr);
    void setName(const char *targetName);
    NODISCARD const char* getName() const;
    NODISCARD uint getColor() const;
};