#pragma once

#include "Graph.h"

struct GraphComparator {
    bool operator()(Graph *lhs, Graph *rhs);
};