#ifndef EVENTI_H
#define EVENTI_H

#include <vector>
#include <cmath>
#include <iostream>

class Vertex;
class HalfEdge;

class Event {
public:
    enum Type {START, END, INTERSECION};
private:
    Type type;
    Vertex *v;
    std::vector<HalfEdge*> edges;
public:
    Event(Vertex *v, Type t) : v(v), type(t) {}

    Vertex* getVertex() {return v; }
    
    Type getType() const {return type; }
    const Vertex* getVertex() const {return v;}
    std::vector<HalfEdge*> getEdges() const {return edges; }

    static Event eventStart(Vertex *v) {
        Event e(v, START);
        return e;
    }

    static Event eventEnd(Vertex *v) {
        Event e(v, END);
        return e;
    }

    static Event eventInt(Vertex *v) {
        Event e(v, INTERSECION);
        return e;
    }

    void setEdges(std::vector<HalfEdge*> e) {
        edges = e;
    }

    void merge(const Event &other);
    
    bool operator<(const Event &other) const;
    bool operator==(const Event &other) const;
    void print() const;
};

#endif
