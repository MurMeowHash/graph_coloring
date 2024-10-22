#include "GraphComparator.h"

bool GraphComparator::operator()(Graph *lhs, Graph *rhs) {
    return lhs->conflicts > rhs->conflicts;
}