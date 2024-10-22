#include "NodeComparator.h"

bool NodeComparator::operator()(const NodeData &lhs, const NodeData &rhs) const {
    return lhs.countConflicts < rhs.countConflicts;
}
