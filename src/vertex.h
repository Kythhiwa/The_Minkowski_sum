#ifndef VERTEX_H
#define VERTEX_H

#include <cmath>
#include <iostream>

class HalfEdge;

class Vertex{
    int id_;
    double x_, y_;
    HalfEdge *incidentEdge_;
public:
    Vertex(double x, double y, int id = 0) : x_(x), y_(y), incidentEdge_(nullptr), id_(id) {};
    Vertex(const Vertex& other) : x_(other.x_), y_(other.y_), id_(other.id_), incidentEdge_(nullptr) {}

    double getX() const { return x_; }
    double getY() const { return y_; }
    HalfEdge* getIncidentEdge() { return incidentEdge_; }
    double getId() const { return id_; } 

    const HalfEdge* getIncidentEdge() const { return incidentEdge_; }

    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }
    void setIncidentEdge(HalfEdge *incidentEdge) { incidentEdge_ = incidentEdge; }
    void setId(int x) {id_ = x; }

    bool operator<(const Vertex& other) const;
    bool operator>(const Vertex& other) const;
    bool operator==(const Vertex& other) const;

    void print() const {std::cout << "("<< x_ << ":" << y_ << ")\n";}
};





#endif
