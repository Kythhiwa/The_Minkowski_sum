#include "eventI.h"
#include "vertex.h"
#include "halfEdge.h"
#include "config.h"


void Event::merge(const Event &other) {
    edges.insert(edges.end(), other.edges.begin(), other.edges.end());
}


bool Event::operator==(const Event &other) const {
    return std::abs(v->getX()-other.v->getX()) < Geometry::eps && std::abs(v->getY()-other.v->getY()) < Geometry::eps;
}
bool Event::operator<(const Event &other) const {
    if (std::abs(v->getY() - other.v->getY()) >= Geometry::eps) {
        return v->getY() > other.v->getY();
    }
    if (std::abs(v->getX() - other.v->getX()) >= Geometry::eps) {
        return v->getX() < other.v->getX();
    }
    return type > other.type;    
}

void Event::print() const {
    std::cout << "Event: ";
    switch (type){
        case START:
            std::cout << "START\n";
            break;
        case END:
            std::cout << "END\n";
            break;
        case INTERSECION:
            std::cout << "INTERSECTION\n";
            break;
    }
    std::cout <<  v->getX() << ":" << v->getY() << "\n";
    for (const auto& h : edges) {
        std::cout << "(" <<h->getOrigin()->getX() << ":" << h->getOrigin()->getY() << ")->(";
        std::cout << h->getEndPoint()->getX() << ":" << h->getEndPoint()->getY() << ")\n";
    }
    std::cout << "~~~~~~~~~~~~~\n";
}

