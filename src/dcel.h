#ifndef DCEL_H
#define DCEL_H

#include <vector>
#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <GL/glut.h>

struct Vertex;
struct HalfEdge;
struct Face;

struct Vertex {
    double x, y;
    HalfEdge *incidentEdge;
    
    Vertex(double x, double y);

    bool operator<(const Vertex& other) const;
    bool operator==(const Vertex& other) const;
};


struct HalfEdge {
    HalfEdge *prev, *next, *twin;
    Vertex *origin;
    Face* incidentFace;
    
    Vertex* endPoint() const;
    bool isHotizontal() const;
};

struct Face {
    HalfEdge *outerComponent;
    std::vector<HalfEdge*> holes;
};


class DCEL {
private:

    struct Event {
        Vertex* point;
        enum Type { UPPER, LOWER, INTERSECTION } type;
        std::vector<HalfEdge*> segments;

        bool operator<(const Event& other) const;
    };

    struct SegmentComparator { // компаратор
        double sweepY;

        SegmentComparator(double y) : sweepY(y) {}
        bool operator()(HalfEdge *a, HalfEdge *b) const; // функтор для set 
        double getXAtSweepY(HalfEdge *seg) const; // возвращает x координату пересечения sweepline с seg
    };

    bool segmentIntersect(const Vertex *p1, const Vertex *p2, const Vertex *q1, const Vertex *q2, Vertex &intersecion);

    std::vector<std::unique_ptr<Vertex>> vertex;
    std::vector<std::unique_ptr<HalfEdge>> halfEdge;
    std::vector<std::unique_ptr<Face>> face;
    

public:
    DCEL();
    DCEL(std::vector<std::pair<double,double>> p);
    
    void merge(DCEL& other); // not optimal
    void print() const;

};



#endif
