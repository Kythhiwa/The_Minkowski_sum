#ifndef DCEL_H
#define DCEL_H

#include <memory>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <stack>
#include <iomanip>
#include <limits>
#include <cfloat>
#include <set>
class Vertex;
class HalfEdge;
class Face;
class Event_T;

class Dcel {
    static int id;
    std::vector<std::unique_ptr<Vertex>> vertex;
    std::vector<std::unique_ptr<HalfEdge>> halfEdge;
    std::vector<std::unique_ptr<Face>> face;
    int id_;
    void setDiag(Event_T a, Event_T b);
    void addDiag(HalfEdge* f, HalfEdge *s);
    void triangulateMonotonePolygon(HalfEdge* h, std::set<HalfEdge*> &us);
public:
    Dcel();
    Dcel(std::vector<std::pair<double, double>> points);
    ~Dcel();

    void normalize(Dcel& a);
        
    static void solve(Dcel& dest, Dcel& a, Dcel& b);
    void add(const Dcel& source);
    void copy(const Dcel& source);
    const std::vector<Vertex*> getVertex() const;
    const std::vector<HalfEdge*> getHalfEdge() const;
    const std::vector<Face*> getFace() const;
    
    void setHoles(std::vector<std::pair<double, double>> points);
    
    std::vector<std::vector<std::pair<double, double>>> triang();

    bool isCounterClockwise(Face* face) const;
    Face* findFacePoint(double x, double y) const;
    std::pair<double, double> getInnerPoint(HalfEdge* start) const;
    static void merge(Dcel& dest, Dcel& a);
    void print() const;
    
};


#endif
