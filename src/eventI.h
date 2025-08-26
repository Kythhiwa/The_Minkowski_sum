#ifndef EVENTI_H
#define EVENTI_H

#include <vector>
#include <cmath>
#include <iostream>
#include <set>

class Vertex;
class HalfEdge;

class Event {
public:
    enum Type {START, END, INTERSECION};
private:
    struct HalfEdgeComparator {
        bool operator()(const HalfEdge* a, const HalfEdge* b) const;
    };
    Type type;
    Vertex *v;
    std::set<HalfEdge*, HalfEdgeComparator> edges;
    bool f;
public:
    Event(Vertex *v, Type t, bool f = false) : v(v), type(t), f(f) {}

    Vertex* getVertex() {return v; }
    
    Type getType() const {return type; }
    const Vertex* getVertex() const {return v;}
    std::vector<HalfEdge*> getEdges() const {
        std::vector<HalfEdge*> v;
        for (auto &now : edges) {
            v.push_back(now);
        }
        return v; 
    }

    static Event eventStart(Vertex *v) {
        Event e(v, START);
        return e;
    }

    static Event eventEnd(Vertex *v) {
        Event e(v, END);
        return e;
    }

    static Event eventInt(Vertex *v, bool f = false) {
        Event e(v, INTERSECION, f);
        return e;
    }
    bool getF(){ return f;}

    void setF(bool f_) {f = f_;}

    void setEdges(std::set<HalfEdge*> e) {
        for (auto& now : e) {
            edges.insert(now);
        }
    }

    void merge(const Event &other);
    
    bool operator<(const Event &other) const;
    bool operator==(const Event &other) const;
    void print() const;
};

#endif
