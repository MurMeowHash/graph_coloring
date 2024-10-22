#include "Node.h"

Node::Node(const char *name, uint color)
: name{name}, color{color} {

}

void Node::setColor(uint clr) {
    color = clr;
}

void Node::setName(const char *targetName) {
    name = targetName;
}

const char *Node::getName() const {
    return name;
}

uint Node::getColor() const {
    return color;
}
