#include "sweepline.h"
#include "halfEdge.h"
#include "vertex.h"
#include "config.h"

double Sweepline::SweeplineComparator::getIntX(const HalfEdge *e) const {
    const Vertex* v1 = e->getOrigin();
    const Vertex* v2 = e->getEndPoint();
    if (e->isHorizontal()) {
        return std::min(v1->getX(), v2->getX()) ;
    }
    double t = (*sweepline_y - v1->getY()) / (v2->getY() - v1->getY());
    return v1->getX() + t * (v2->getX() - v1->getX());
}

bool Sweepline::SweeplineComparator::operator()(const HalfEdge *a, const HalfEdge *b)  const {
    double x_a = getIntX(a);
    double x_b = getIntX(b);
    if (std::abs(x_a - x_b) >= Geometry::eps) {
        return x_a < x_b;
    }
    
    const Vertex* a1 = a->getOrigin();
    const Vertex* a2 = a->getEndPoint();
    const Vertex* b1 = b->getOrigin();
    const Vertex* b2 = b->getEndPoint();


    double slope_a = (a2->getX() - a1->getX()) / (a2->getY() - a1->getY());
    double slope_b = (b2->getX() - b1->getX()) / (b2->getY() - b1->getY());
    return slope_a > slope_b;
}

bool Sweepline::SweeplineComparator::operator()(double x, HalfEdge *a) const {
    //std::cout << "INTe >" << getIntX(a) << "\n";
    return getIntX(a) > x;
}

bool Sweepline::SweeplineComparator::operator()(HalfEdge *a, double x) const {
    //std::cout << "INTe <" << getIntX(a) << "\n";
    return getIntX(a) < x;
}


void Sweepline::insert(HalfEdge *e) {
    if (!e) return;
    T.insert(e);
}

void Sweepline::erase(HalfEdge *e) {
    if (!e) return;
    T.erase(e);
}


HalfEdge* Sweepline::bigger(double x) const {
    auto it = T.upper_bound(x); // > x 
    return (it != T.end()) ? *it : nullptr;
}

HalfEdge* Sweepline::bigger(HalfEdge* x) const{
    auto it = T.upper_bound(x); // > x 
    return (it != T.end()) ? *it : nullptr;

}

HalfEdge* Sweepline::less(double x) const {
    auto it = T.lower_bound(x); // >= x
    return (it != T.begin()) ? *std::prev(it) : nullptr;
}

HalfEdge* Sweepline::less(HalfEdge* x) const {
    auto it = T.lower_bound(x);
    return (it != T.begin()) ? *std::prev(it) : nullptr;
}

void Sweepline::print() const {
    for (const auto& now : T) {
        std::cout << "(" << now->getOrigin()->getX() << " " << now->getOrigin()->getY() << ") ->";
        std::cout << "(" << now->getEndPoint()->getX() << " " << now->getEndPoint()->getY() << ")\n";
    }
}
