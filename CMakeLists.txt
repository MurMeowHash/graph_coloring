cmake_minimum_required(VERSION 3.29)
project(graph_coloring)

set(CMAKE_CXX_STANDARD 20)

add_executable(Laboratory2 src/main.cpp
        src/Graph/Node.cpp
        src/Graph/Node.h
        src/utils/utils.cpp
        src/utils/utils.h
        src/Color/Color.h
        src/Graph/Graph.cpp
        src/Graph/Graph.h
        src/Error/Error.cpp
        src/Error/Error.h
        src/Graph/GraphComparator.cpp
        src/Graph/GraphComparator.h
        src/Graph/NodeColorData.h
        src/Graph/NodeComparator.cpp
        src/Graph/NodeComparator.h
        src/Graph/NodeData.h
)
