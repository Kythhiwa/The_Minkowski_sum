#ifndef DCEL_H
#define DCEL_H

#include <memory>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <map>

class Vertex;
class HalfEdge;
class Face;

class Dcel {
    std::vector<std::unique_ptr<Vertex>> vertex;
    std::vector<std::unique_ptr<HalfEdge>> halfEdge;
    std::vector<std::unique_ptr<Face>> face;
    int id_;
public:
    Dcel();
    Dcel(std::vector<std::pair<double, double>> points, int id = 0);
    ~Dcel();
    
    void add(const Dcel& source);// нет логики holes, объединяет только по вершинам и простым ребрам

    static void merge(Dcel& dest, Dcel& a, Dcel& b);
    void print() const;
};


#endif
