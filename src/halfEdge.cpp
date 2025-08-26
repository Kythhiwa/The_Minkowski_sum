#include "halfEdge.h"
#include "vertex.h"
#include "face.h" 
#include "config.h"
#include <cstdlib>

bool HalfEdge::segmentsIntersect(HalfEdge *a, HalfEdge *b, Vertex &intersection) {
    if (a == nullptr || b == nullptr) return false;
    const Vertex* p1 = a->getOrigin();
    const Vertex* p2 = a->getEndPoint();
    const Vertex* q1 = b->getOrigin();
    const Vertex* q2 = b->getEndPoint();

    double r_x = p2->getX() - p1->getX();
    double r_y = p2->getY() - p1->getY();
    double s_x = q2->getX() - q1->getX();
    double s_y = q2->getY() - q1->getY();

    double r_cross_s = r_x * s_y - r_y * s_x;
    double qp_x = q1->getX() - p1->getX();
    double qp_y = q1->getY() - p1->getY();
    double qp_cross_r = qp_x * r_y - qp_y * r_x;

    if (std::abs(r_cross_s) < Geometry::eps && std::abs(qp_cross_r) < Geometry::eps) {
        double r_sq = r_x*r_x + r_y*r_y;
        if (r_sq < Geometry::eps * Geometry::eps) return false;

        double t0 = (qp_x*r_x + qp_y*r_y) / r_sq;
        double t1 = t0 + (s_x*r_x + s_y*r_y) / r_sq;

        if (std::max(t0, t1) < -Geometry::eps || std::min(t0, t1) > 1 + Geometry::eps)
            return false;

        double t = std::max(0.0, std::min(t0, t1));
        intersection.setX(p1->getX() + t*r_x);
        intersection.setY(p1->getY() + t*r_y);
        return true;
    }

    if (std::abs(r_cross_s) < Geometry::eps)
        return false;

    double t = (qp_x*s_y - qp_y*s_x) / r_cross_s;
    double u = qp_cross_r / r_cross_s;

    if (t >= -Geometry::eps && t <= 1 + Geometry::eps && u >= -Geometry::eps && u <= 1 + Geometry::eps) {
        intersection.setX(p1->getX() + t*r_x);
        intersection.setY(p1->getY() + t*r_y);
        return true;
    }

    return false;
}

Vertex* HalfEdge::getEndPoint() {
    if (next_->getOrigin() == nullptr) {
        throw "The segment has an incorrect endpoint\n";
    }
    return next_->getOrigin();
}


const Vertex* HalfEdge::getEndPoint() const{
    if (next_->getOrigin() == nullptr) {
        throw "The segment has an incorrect endpoint\n";
    }
    return next_->getOrigin();
}

bool HalfEdge::isHorizontal() const {
    return std::abs(origin_->getY() - getEndPoint()->getY()) < Geometry::eps;
}

void HalfEdge::print() const {
    std::cout << "(" <<origin_->getX() << ":" << origin_->getY() << ")->(";
    std::cout << getEndPoint()->getX() << ":" << getEndPoint()->getY() << ")\n";

}

bool HalfEdge::operator<(const HalfEdge& other) const {
    if (std::abs(origin_->getX() - other.origin_->getX()) >= Geometry::eps) {
        return origin_->getX() < other.origin_->getX();
    }
    if (std::abs(origin_->getY() - other.origin_->getY()) >= Geometry::eps) {
        return origin_->getY() < other.origin_->getY();
    }
    
    if (std::abs(getEndPoint()->getX() - other.getEndPoint()->getX()) >= Geometry::eps) {
        return getEndPoint()->getX() < other.getEndPoint()->getX();
    }
    if (std::abs(getEndPoint()->getY() - other.getEndPoint()->getY()) >= Geometry::eps) {
        return getEndPoint()->getY() < other.getEndPoint()->getY();
    }
    
    return false; // рёбра идентич
}
