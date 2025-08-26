#include "eventT.h" 
#include "vertex.h" 
#include "config.h"
#include "halfEdge.h"


bool Event_T::operator==(const Event_T &other) const {
    return std::abs(v->getX()-other.v->getX()) < Geometry::eps && std::abs(v->getY()-other.v->getY()) < Geometry::eps;
}
bool Event_T::operator<(const Event_T & other) const {
    if (v->getY() - other.v->getY()) {
        return v->getY() > other.v->getY();
    }
    
    if (v->getX() != other.v->getX()) {
        return v->getX() < other.v->getX();
    }
    
    return type > other.type;
}

void Event_T::print() const {
    std::cout << "Event: ";
    switch (type){
        case START:
            std::cout << "START\n";
            break;
        case END:
            std::cout << "END\n";
            break;
        case REGULAR:
            std::cout << "REGULAR\n";
            break;
        case SPLIT:
            std::cout << "SPLIT\n";
            break;
        case MERGE:
            std::cout << "MERGE\n";
    }
    std::cout << v->getX() << ":" << v->getY() << "\n";
    prev->print();
    next->print();
    std::cout << "~~~~~~~~~~~~~\n";
}



HalfEdge* Event_T::getL() {
    if (next->getEndPoint()->getY() < v->getY()) {
        return next;
    } else {
        return prev;
    }
}
