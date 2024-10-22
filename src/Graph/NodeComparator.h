#pragma once

#include "NodeData.h"

struct NodeComparator {
    bool operator()(const NodeData &lhs, const NodeData &rhs) const;
};