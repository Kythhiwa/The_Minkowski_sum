#ifndef EVENTT_H
#define EVENTT_H

#include <iostream>


class Vertex;
class HalfEdge;

class Event_T {
public:
    enum Type {START, END, REGULAR, SPLIT, MERGE};
private:
    Type type;
    Vertex *v;
    HalfEdge *prev;
    HalfEdge *next;
public:
    Event_T(Vertex *v, Type t) : v(v), type(t) {}
    Event_T(Vertex *v, Type t, HalfEdge *p, HalfEdge *n) : v(v), type(t), prev(p), next(n) {}

    Vertex* getVertex() {return v; }
    HalfEdge* getPrev() {return prev; }
    HalfEdge* getNext() {return next; }

    Type getType() const {return type; }
    const Vertex* getVertex() const {return v;}
    const HalfEdge* getPrev() const {return prev; }
    const HalfEdge* getNext() const {return next; }

    void setPrev(HalfEdge* h) {prev = h;}
    void setNext(HalfEdge* h) {next = h;}

    HalfEdge* getL();

    

    void merge(const Event_T &other){std::cout << "MERGE\n";};
    
    bool operator<(const Event_T &other) const;
    bool operator==(const Event_T &other) const;
    void print() const;

};













#endif
