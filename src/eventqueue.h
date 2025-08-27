#ifndef EVENTQUEUE_H
#define EVENTQUEUE_H

#include <set>
#include <iostream>
#include"halfEdge.h"
template <typename Event>
class EventQueue {
    std::set<Event> Q;
public:
    void push(const Event &event) {
        auto it  = Q.find(event);
        if (it != Q.end()) {
            const_cast<Event&>(*it).merge(event);
        } else {
            Q.insert(event);
        }
    }

    Event pop() {
        Event e = *Q.begin();
        Q.erase(Q.begin());
        return e;
    }
    bool isEmpty() { return Q.empty(); }

    void print() const {
        for (const auto& now : Q) {
            now.print();
        }
    }
};



#endif
