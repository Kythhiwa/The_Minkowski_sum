#ifndef SWEEPLINE_H
#define SWEEPLINE_H

#include <set>
#include <cmath>
#include <iostream>
#include <algorithm>

class HalfEdge;

class Sweepline{
    struct SweeplineComparator {
        using is_transparent = void;
        const double *sweepline_y;
        SweeplineComparator(const double *y) : sweepline_y(y) {}

        double getIntX(const HalfEdge *edge) const; // X координата пересечения с sweepline 

        bool operator()(const HalfEdge *a, const HalfEdge *b) const;
        bool operator()(double x, HalfEdge *a) const;
        bool operator()(HalfEdge *a, double x) const;
    };
    
    double current_y;
    std::set<HalfEdge*, SweeplineComparator> T;
public:
    Sweepline(double y) : current_y(y), T{SweeplineComparator(&current_y)} {}
    
    void insert(HalfEdge *e);
    void erase(HalfEdge *e);
    void setY(double y) {current_y = y; };
    
    HalfEdge* bigger(double x) const;
    HalfEdge* bigger(HalfEdge* x) const;
    HalfEdge* less(double x) const;
    HalfEdge* less(HalfEdge* x) const;

    void print() const;

};





#endif
