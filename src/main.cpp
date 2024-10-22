#include <iostream>

#include "Graph/Graph.h"
#include <set>

Graph *makeUSAGraph(uint colorsCount);
Graph *makeAustraliaGraph();
void testLocalBeamSearch(size_t k);
void testBacktracking();
void printGraph(Graph *graph);

int main() {
    testLocalBeamSearch(20);
//    testBacktracking();
    return 0;
}

Graph *makeUSAGraph(uint colorsCount) {
    auto graph = new Graph{colorsCount};
    graph->addNode("Washington");
    graph->addNode("Oregon");
    graph->addNode("Idaho");
    graph->addNode("California");
    graph->addNode("Nevada");
    graph->addNode("Utah");
    graph->addNode("Arizona");
    graph->addNode("New Mexico");
    graph->addNode("Colorado");
    graph->addNode("Wyoming");
    graph->addNode("Montana");
    graph->addNode("North Dakota");
    graph->addNode("South Dakota");
    graph->addNode("Nebraska");
    graph->addNode("Kansas");
    graph->addNode("Oklahoma");
    graph->addNode("Texas");
    graph->addNode("Minnesota");
    graph->addNode("Iowa");
    graph->addNode("Missouri");
    graph->makeEdge("Washington", "Oregon");
    graph->makeEdge("Washington", "Idaho");
    graph->makeEdge("Oregon", "Idaho");
    graph->makeEdge("Oregon", "California");
    graph->makeEdge("Oregon", "Nevada");
    graph->makeEdge("California", "Nevada");
    graph->makeEdge("California", "Arizona");
    graph->makeEdge("Idaho", "Montana");
    graph->makeEdge("Idaho", "Wyoming");
    graph->makeEdge("Idaho", "Utah");
    graph->makeEdge("Idaho", "Nevada");
    graph->makeEdge("Nevada", "Arizona");
    graph->makeEdge("Nevada", "Utah");
    graph->makeEdge("Arizona", "Utah");
    graph->makeEdge("Arizona", "New Mexico");
    graph->makeEdge("Utah", "Colorado");
    graph->makeEdge("Utah", "Wyoming");
    graph->makeEdge("Montana", "Wyoming");
    graph->makeEdge("Montana", "North Dakota");
    graph->makeEdge("Montana", "South Dakota");
    graph->makeEdge("Wyoming", "Colorado");
    graph->makeEdge("Wyoming", "South Dakota");
    graph->makeEdge("Wyoming", "Nebraska");
    graph->makeEdge("Colorado", "New Mexico");
    graph->makeEdge("Colorado", "Nebraska");
    graph->makeEdge("Colorado", "Kansas");
    graph->makeEdge("Colorado", "Oklahoma");
    graph->makeEdge("New Mexico", "Oklahoma");
    graph->makeEdge("New Mexico", "Texas");
    graph->makeEdge("Texas", "Oklahoma");
    graph->makeEdge("Oklahoma", "Kansas");
    graph->makeEdge("Oklahoma", "Missouri");
    graph->makeEdge("Kansas", "Nebraska");
    graph->makeEdge("Kansas", "Missouri");
    graph->makeEdge("Nebraska", "South Dakota");
    graph->makeEdge("Nebraska", "Missouri");
    graph->makeEdge("Nebraska", "Iowa");
    graph->makeEdge("South Dakota", "Iowa");
    graph->makeEdge("South Dakota", "Minnesota");
    graph->makeEdge("South Dakota", "North Dakota");
    graph->makeEdge("North Dakota", "Minnesota");
    graph->makeEdge("Minnesota", "Iowa");
    graph->makeEdge("Iowa", "Missouri");
    return graph;
}

Graph *makeAustraliaGraph() {
    auto graph = new Graph{3};
    graph->addNode("Western Australia");
    graph->addNode("Northern Territory");
    graph->addNode("South Australia");
    graph->addNode("Queensland");
    graph->addNode("New South Wales");
    graph->addNode("Victoria");
    graph->makeEdge("Western Australia", "Northern Territory");
    graph->makeEdge("Western Australia", "South Australia");
    graph->makeEdge("Northern Territory", "South Australia");
    graph->makeEdge("Northern Territory", "Queensland");
    graph->makeEdge("South Australia", "Queensland");
    graph->makeEdge("South Australia", "New South Wales");
    graph->makeEdge("South Australia", "Victoria");
    graph->makeEdge("Queensland", "New South Wales");
    graph->makeEdge("New South Wales", "Victoria");
    return graph;
}

void testLocalBeamSearch(size_t k) {
    Graph *graph = makeUSAGraph(4);
    graph->localBeamSearch(k);
    printGraph(graph);
}

void testBacktracking() {
    Graph *graph = makeUSAGraph(4);
    graph->backtracking();
    printGraph(graph);
}

void printGraph(Graph *graph) {
    auto nodes = graph->getNodes();
    for(Node *node : nodes) {
        std::cout << node->getName() << ": " << node->getColor() << '\n';
    }
    std::cout << "Conflicts count: " << graph->getStateConflicts();
}