#ifndef FACE_H
#define FACE_H

#include <vector>

class HalfEdge;

class Face {
public:
    enum class Type {
        OUTER,
        INNER,
        HOLES
    };
private:
    HalfEdge *outerComponent_;
    std::vector<HalfEdge*> holes_; // каждое ребро обозначает свою дыру
    Type type_;
    int id_;
public:
    Face(Type type) : type_(type) {}
    Face(const Face& other) : id_(other.id_), type_(other.type_), outerComponent_(nullptr) {}

    HalfEdge* getOuterComponent() { return outerComponent_; }
    Type getType() { return type_; }

    const HalfEdge* getOuterComponent() const { return outerComponent_; }
    const Type getType() const { return type_; }
    int getId() const {return id_; }

    void setType(Type type) {type_ = type; }
    void setOuterComponent(HalfEdge *outerComponent) { outerComponent_ = outerComponent; }
    void setId(int x) {id_ = x; }
};


#endif
