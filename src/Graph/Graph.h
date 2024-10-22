#pragma once

#include "Node.h"
#include "NodeColorData.h"
#include <random>
#include <map>

class GraphComparator;

class Graph {
    friend class GraphComparator;
private:
    static constexpr int INVALID_INDEX{-1};
    static constexpr uint EDGE{1};
    static constexpr uint MAX_STEPS_RIGHT_COUNT{100};
    static constexpr uint NO_COLOR{0};
    static const uint INFINITE_CONFLICTS;
    std::vector<std::vector<uint>> adjacencyMatrix;
    std::vector<Node *> nodes;
    uint conflicts;
    std::vector<uint> beamColors;
    std::vector<Color> backtrackingColors;
    static std::random_device rd;
    static std::mt19937 mt;
    void initColors(uint countColors);
    void copyNodes(const Graph &graph);
    void increaseRowsSize();
    NODISCARD uint getConflictsCount(uint node) const;
    NODISCARD auto heapifyNodes() const;
    void countStateConflicts();
    NODISCARD auto getNeighborStates(size_t k) const;
    void setInitialBeamState();
    void setInitialBacktrackingState();
    NODISCARD bool checkGoalState() const;
    NODISCARD std::vector<Graph *> getNeighborStates(const std::pair<int, NodeColorData*> &currentNode) const;
    void updateNeighbors(const std::map<int, NodeColorData *> &activeNodes, uint nodeIndex);
    bool backtrackingUtil(const std::map<int, NodeColorData *> &activeNodes, Graph *&goalState);
    static void mergeQueues(auto &dest, auto &src);
    static void clearStateQueue(auto &target);
    NODISCARD static std::map<int, NodeColorData *> copyActiveNodes(const std::map<int, NodeColorData *> &src);
    static void deleteStatesData(const std::vector<Graph *> &target);
    static void deleteActiveNodes(const std::map<int, NodeColorData *> &activeNodes);
public:
    explicit Graph(uint countColors);
    Graph(const Graph &rhs);
    void addNode(const char *name);
    void makeEdge(int lhsIndex, int rhsIndex);
    void makeEdge(const char *lhs, const char *rhs);
    bool localBeamSearch(size_t k);
    bool backtracking();
    NODISCARD std::vector<Node *> getNodes() const;
    NODISCARD uint getStateConflicts();
    ~Graph();
    Graph& operator=(const Graph &rhs);
};