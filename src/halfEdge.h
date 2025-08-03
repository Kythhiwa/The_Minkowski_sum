#ifndef HALFEDGE_H
#define HALFEDGE_H
#include <cmath>
#include <iostream>
class Vertex;
class Face;

class HalfEdge {    
    HalfEdge *prev_, *next_, *twin_;
    Vertex *origin_;
    Face *incidentFace_;
    int id_;
public:
    HalfEdge() : id_(0), prev_(nullptr), next_(nullptr), twin_(nullptr), origin_(nullptr), incidentFace_(nullptr) {}
    HalfEdge(const HalfEdge& other) : id_(other.id_), prev_(nullptr), next_(nullptr), twin_(nullptr), origin_(nullptr), incidentFace_(nullptr) {}
    static bool segmentsIntersect(HalfEdge *a, HalfEdge *b, Vertex &intersection);

    bool isHorizontal() const;

    HalfEdge* getPrev() { return prev_; }
    HalfEdge* getNext() { return next_; }
    HalfEdge* getTwin() { return twin_; }
    Vertex* getOrigin() { return origin_; }
    Vertex* getEndPoint();
    Face* getIncidentFace() { return incidentFace_; }

    int getId() const {return id_;};
    const HalfEdge* getPrev() const { return prev_; }
    const HalfEdge* getNext() const { return next_; }
    const HalfEdge* getTwin() const { return twin_; }
    const Vertex* getOrigin() const { return origin_; }
    const Vertex* getEndPoint() const;
    const Face* getIncidentFace() const { return incidentFace_; }

    void setPrev(HalfEdge *prev) { prev_ = prev; }
    void setNext(HalfEdge *next) { next_ = next; }
    void setTwin(HalfEdge *twin) { twin_ = twin; }
    void setOrigin(Vertex *origin) { origin_ = origin; }
    void setIncidentFace(Face *incidentFace) { incidentFace_ = incidentFace; }
    void setId(int x) {id_ = x; }

    void print() const;
};

#endif
