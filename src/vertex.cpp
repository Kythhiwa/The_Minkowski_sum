#include "vertex.h"
#include "halfEdge.h"
#include "config.h"

bool Vertex::operator<(const Vertex& other) const {
    if (y_ > other.y_) return true;
    if (y_ < other.y_) return false;
    return x_ < other.x_;
}

bool Vertex::operator>(const Vertex& other) const {
    return other < *this;
}

 
bool Vertex::operator==(const Vertex& other) const {
    return std::abs(y_ - other.y_) < Geometry::eps && std::abs(x_ - other.x_) < Geometry::eps;
}
