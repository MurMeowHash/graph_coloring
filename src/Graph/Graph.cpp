#include "Graph.h"
#include "GraphComparator.h"
#include "NodeComparator.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include <queue>

std::random_device Graph::rd;
std::mt19937 Graph::mt{rd()};
const uint Graph::INFINITE_CONFLICTS{std::numeric_limits<uint>::max()};

Graph::Graph(uint countColors)
: conflicts{0}, beamColors(countColors), backtrackingColors(countColors) {
    initColors(countColors);
}

void Graph::initColors(uint countColors) {
    for(uint i = 0; i < countColors; i++) {
        uint currentColor = i + 1;
        beamColors[i] = currentColor;
        backtrackingColors[i].color = currentColor;
        backtrackingColors[i].available = true;
    }
}

Graph::Graph(const Graph &rhs)
: adjacencyMatrix(rhs.adjacencyMatrix), conflicts{rhs.conflicts}, beamColors(rhs.beamColors),
backtrackingColors(rhs.backtrackingColors) {
    copyNodes(rhs);
}

void Graph::copyNodes(const Graph &graph) {
    nodes.resize(graph.nodes.size());
    for(uint i = 0; i < nodes.size(); i++) {
        nodes[i] = new Node{*graph.nodes[i]};
    }
}

void Graph::increaseRowsSize() {
    for(auto &row : adjacencyMatrix) {
        row.emplace_back();
    }
}

void Graph::addNode(const char *name) {
    nodes.push_back(new Node{name});
    adjacencyMatrix.emplace_back(adjacencyMatrix.size());
    increaseRowsSize();
}

void Graph::makeEdge(int lhsIndex, int rhsIndex) {
    adjacencyMatrix[lhsIndex][rhsIndex] = EDGE;
    adjacencyMatrix[rhsIndex][lhsIndex] = EDGE;
}

void Graph::makeEdge(const char *lhs, const char *rhs) {
    int lhsIndex{INVALID_INDEX}, rhsIndex{INVALID_INDEX};
    for(int i = 0; i < nodes.size(); i++) {
        const char *nodeName = nodes[i]->getName();
        if(nodeName == lhs) {
            lhsIndex = i;
        } else if(nodeName == rhs) {
            rhsIndex = i;
        }
    }
    if(lhsIndex == INVALID_INDEX
    || rhsIndex == INVALID_INDEX) {
        return;
    }
    makeEdge(lhsIndex, rhsIndex);
}

uint Graph::getConflictsCount(uint node) const {
    uint nodeConflicts{0};
    const std::vector<uint> &neighbors = adjacencyMatrix[node];
    for(uint i = 0; i < neighbors.size(); i++) {
        if(neighbors[i] != 0) {
            if(nodes[i]->getColor() == nodes[node]->getColor()) {
                nodeConflicts++;
            }
        }
    }
    return nodeConflicts;
}

auto Graph::heapifyNodes() const {
    std::priority_queue<NodeData, std::vector<NodeData>, NodeComparator> nodesQueue;
    for(uint i = 0; i < nodes.size(); i++) {
        uint nodeConflicts = getConflictsCount(i);
        nodesQueue.emplace(i, nodeConflicts);
    }
    return nodesQueue;
}

void Graph::countStateConflicts() {
    conflicts = 0;
    for(uint i = 0; i < adjacencyMatrix.size(); i++) {
        for(uint j = i + 1; j < adjacencyMatrix[i].size(); j++) {
            if(adjacencyMatrix[i][j]) {
                if(nodes[i]->getColor() == nodes[j]->getColor()) {
                    conflicts++;
                }
            }
        }
    }
}

auto Graph::getNeighborStates(size_t k) const {
    std::priority_queue<Graph *, std::vector<Graph *>, GraphComparator> neighborStates;
    auto nodesQueue = heapifyNodes();
    size_t neighborsSize = std::min(k, nodesQueue.size());
    for(uint i = 0; i < neighborsSize; i++) {
        for(uint color : beamColors) {
            uint worstNodeIndex = nodesQueue.top().node;
            if(color != nodes[worstNodeIndex]->getColor()) {
                auto neighborState = new Graph{*this};
                Node *worstNode = neighborState->nodes[worstNodeIndex];
                worstNode->setColor(color);
                uint nodeConflictsAfter = neighborState->getConflictsCount(worstNodeIndex);
                neighborState->conflicts += nodeConflictsAfter - nodesQueue.top().countConflicts;
                neighborStates.emplace(neighborState);
            }
        }
        nodesQueue.pop();
    }
    return neighborStates;
}

void Graph::setInitialBeamState() {
    std::uniform_int_distribution<uint> colorsDistribution(0, beamColors.size() - 1);
    for(Node *node : nodes) {
        node->setColor(beamColors[colorsDistribution(mt)]);
    }
    countStateConflicts();
}

void Graph::setInitialBacktrackingState() {
    for(Node *node : nodes) {
        node->setColor(0);
    }
}

bool Graph::checkGoalState() const {
    return conflicts == 0;
}

bool Graph::localBeamSearch(size_t k) {
    setInitialBeamState();
    std::priority_queue<Graph *, std::vector<Graph *>, GraphComparator> openStates;
    openStates.emplace(new Graph{*this});
    bool foundGoal{false}, detectedLocalExtrema{false};
    uint availableStepsRight{MAX_STEPS_RIGHT_COUNT};
    uint prevBestStateConflicts{INFINITE_CONFLICTS};
    while(!foundGoal
    && !detectedLocalExtrema
    && availableStepsRight != 0) {
        Graph *bestState = openStates.top();
        if(bestState->conflicts == prevBestStateConflicts) {
            availableStepsRight--;
        } else if(bestState->conflicts > prevBestStateConflicts) {
            detectedLocalExtrema = true;
        }
        prevBestStateConflicts = bestState->conflicts;
        if(detectedLocalExtrema || availableStepsRight == 0) {
            *this = *bestState;
        } else {
            if(bestState->checkGoalState()) {
                foundGoal = true;
                *this = *bestState;
            } else {
                typeof(openStates) neighborStates;
                size_t neighborSizes = std::min(k, openStates.size());
                for(size_t i = 0; i < neighborSizes; i++) {
                    Graph *currentState = openStates.top();
                    auto tmpStates = currentState->getNeighborStates(k);
                    mergeQueues(neighborStates, tmpStates);
                    delete currentState;
                    openStates.pop();
                }
                clearStateQueue(openStates);
                openStates = std::move(neighborStates);
            }
        }
    }
    clearStateQueue(openStates);
    return foundGoal;
}

std::vector<Graph *> Graph::getNeighborStates(const std::pair<int, NodeColorData*> &currentNode) const {
    std::vector<Graph *> neighborStates;
    for(uint i = 0; i < currentNode.second->availableColors.size(); i++) {
        if(currentNode.second->availableColors[i].available) {
            auto neighborState = new Graph{*this};
            neighborState->nodes[currentNode.first]->setColor(currentNode.second->availableColors[i].color);
            neighborStates.emplace_back(neighborState);
        }
    }
    return neighborStates;
}

void Graph::updateNeighbors(const std::map<int, NodeColorData *> &activeNodes, uint nodeIndex) {
    const std::vector<uint> &neighbors = adjacencyMatrix[nodeIndex];
    Node *node = nodes[nodeIndex];
    for(int i = 0; i < neighbors.size(); i++) {
        if(neighbors[i] != 0
        && nodes[i]->getColor() == NO_COLOR) {
            auto neighborNode = activeNodes.find(i);
            neighborNode->second->remainingColors--;
            neighborNode->second->availableColors[node->getColor() - 1].available = false;
        }
    }
}

bool Graph::backtrackingUtil(const std::map<int, NodeColorData *> &activeNodes, Graph *&goalState) {
    if(activeNodes.empty()) {
        goalState = new Graph{*this};
        return true;
    }
    auto currentNodeIterator = std::min_element(activeNodes.begin(), activeNodes.end(),
                                                [](const std::pair<int, NodeColorData*> &lhs,
                                                        const std::pair<int, NodeColorData*> &rhs) {
        return lhs.second->remainingColors < rhs.second->remainingColors;
    });
    if(currentNodeIterator->second->remainingColors == backtrackingColors.size()) {
        std::uniform_int_distribution<int> distribution(0, static_cast<int>(nodes.size() - 1));
        currentNodeIterator = activeNodes.find(distribution(mt));
    }
    std::vector<Graph *> neighborStates = getNeighborStates(*currentNodeIterator);
    if(neighborStates.empty()) {
        return false;
    }
    bool foundGoal{false};
    for(Graph *neighbor : neighborStates) {
        if(!foundGoal) {
            auto copiedActiveNodes = copyActiveNodes(activeNodes);
            neighbor->updateNeighbors(copiedActiveNodes, currentNodeIterator->first);
            copiedActiveNodes.erase(currentNodeIterator->first);
            foundGoal = neighbor->backtrackingUtil(copiedActiveNodes, goalState);
            deleteActiveNodes(copiedActiveNodes);
        }
    }
    deleteStatesData(neighborStates);
    return foundGoal;
}

bool Graph::backtracking() {
    setInitialBacktrackingState();
    std::map<int, NodeColorData *> activeNodes;
    for(int i = 0; i < nodes.size(); i++) {
        activeNodes.emplace(i, new NodeColorData{backtrackingColors,
                                                 static_cast<uint>(backtrackingColors.size())});
    }

    Graph *goalState{nullptr};
    bool succeed = backtrackingUtil(activeNodes, goalState);
    if(goalState) {
        *this = *goalState;
    }
    deleteActiveNodes(activeNodes);
    delete goalState;
    return succeed;
}

void Graph::mergeQueues(auto &dest, auto &src) {
    while(!src.empty()) {
        dest.emplace(src.top());
        src.pop();
    }
}

void Graph::clearStateQueue(auto &target) {
    while(!target.empty()) {
        auto state = target.top();
        delete state;
        target.pop();
    }
}
std::map<int, NodeColorData *> Graph::copyActiveNodes(const std::map<int, NodeColorData *> &src) {
    std::map<int, NodeColorData *> dest;
    for(const auto &srcData : src) {
        dest.emplace(srcData.first, new NodeColorData{*srcData.second});
    }
    return dest;
}

void Graph::deleteStatesData(const std::vector<Graph *> &target) {
    for(auto allocData : target) {
        delete allocData;
    }
}

void Graph::deleteActiveNodes(const std::map<int, NodeColorData *> &activeNodes) {
    for(const auto &nodesData : activeNodes) {
        delete nodesData.second;
    }
}

Graph::~Graph() {
    for(Node *node : nodes) {
        delete node;
    }
}

Graph &Graph::operator=(const Graph &rhs) {
    adjacencyMatrix = rhs.adjacencyMatrix;
    conflicts = rhs.conflicts;
    beamColors = rhs.beamColors;
    backtrackingColors = rhs.backtrackingColors;
    copyNodes(rhs);
    return *this;
}

std::vector<Node *> Graph::getNodes() const {
    return nodes;
}

uint Graph::getStateConflicts() {
    countStateConflicts();
    return conflicts;
}
