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
class Vertex;
class HalfEdge;
class Face;

class Dcel {
    static int id;
    std::vector<std::unique_ptr<Vertex>> vertex;
    std::vector<std::unique_ptr<HalfEdge>> halfEdge;
    std::vector<std::unique_ptr<Face>> face;
    int id_;
public:
    Dcel();
    Dcel(std::vector<std::pair<double, double>> points);
    ~Dcel();

        
    static void solve(Dcel& dest, Dcel& a, Dcel& b);
    void add(const Dcel& source);
    const std::vector<Vertex*> getVertex() const;
    const std::vector<HalfEdge*> getHalfEdge() const;
    const std::vector<Face*> getFace() const;

    void setHoles(std::vector<std::pair<double, double>> points);
    
    bool isCounterClockwise(Face* face) const;
    Face* findFacePoint(double x, double y) const;
    std::pair<double, double> getInnerPoint(HalfEdge* start) const;
    static void merge(Dcel& dest, Dcel& a, Dcel& b);
    void print() const;
    void fix();
    void dfs() const;
    void test(Dcel& a, Dcel&b);
};


#endif
