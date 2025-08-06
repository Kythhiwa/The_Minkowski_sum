#include "dcel.h"
#include "vertex.h"
#include "halfEdge.h"
#include "face.h"
#include "sweepline.h"
#include "eventqueue.h"
#include "eventI.h"
#include "config.h"

Dcel::Dcel() = default;
Dcel::Dcel(std::vector<std::pair<double, double>> points, int id) {
    id_ = id;
    if (points.size() < 3) {
        throw std::invalid_argument("[ERROR]Point count < 3\n");
    }
    for (const auto& [x, y] : points) {
        vertex.push_back(std::make_unique<Vertex>(x, y));
    }
    face.push_back(std::make_unique<Face>(Face::Type::OUTER));
    face.push_back(std::make_unique<Face>(Face::Type::INNER));
    
    for (size_t i = 0; i < vertex.size(); ++i) {
        vertex[i]->setId(id_);
        size_t next_i = (i + 1) % vertex.size();

        auto out = std::make_unique<HalfEdge>();
        auto in = std::make_unique<HalfEdge>();
        out->setId(id);
        in->setId(id);

        out->setOrigin(vertex[i].get());
        in->setOrigin(vertex[next_i].get());
    
        out->setTwin(in.get());
        in->setTwin(out.get());

        out->setIncidentFace(face[1].get());
        in->setIncidentFace(face[0].get());

        vertex[i]->setIncidentEdge(out.get());
        halfEdge.push_back(std::move(out));
        halfEdge.push_back(std::move(in));
    }
       
    for (size_t i = 0; i < halfEdge.size(); i += 2) {
        size_t next_i_in = (i + 2) % halfEdge.size();
        halfEdge[i]->setNext(halfEdge[next_i_in].get());
        halfEdge[next_i_in]->setPrev(halfEdge[i].get());

        size_t next_i_ou = (i + 3) % halfEdge.size();
        halfEdge[i + 1]->setPrev(halfEdge[next_i_ou].get());
        halfEdge[next_i_ou]->setNext(halfEdge[i + 1].get());
    }
    face[0]->setOuterComponent(halfEdge[1].get());
    face[1]->setOuterComponent(halfEdge[0].get());
    
    for (auto& h: halfEdge) {
        if (h->isHorizontal()){
            if (h->getOrigin()->getX() > h->getEndPoint()->getX()) {
                h->getOrigin()->setY(h->getOrigin()->getY() + Geometry::eps);
            } else {
                h->getEndPoint()->setY(h->getEndPoint()->getY() + Geometry::eps);
            }
        }
    }
    for (size_t i = 0; i < face.size(); ++i) {
        face[i]->setId(id_);
    }
}

Dcel::~Dcel() = default;

void Dcel::print() const {
    
    std::cout << "=== DCEL Structure ===" << std::endl;
    
    std::cout << "\nVertices (" << vertex.size() << "):\n";
    std::unordered_map<Vertex*, size_t> vertex_ids;
    for (size_t i = 0; i < vertex.size(); ++i) {
        vertex_ids[vertex[i].get()] = i;
        std::cout << "  V" << i << ": (" << vertex[i]->getX() << ", " << vertex[i]->getY() << ")\n";
    }

    std::cout << "\nHalf-Edges (" << halfEdge.size() << "):\n";
    std::unordered_map<HalfEdge*, size_t> edge_ids;
    for (size_t i = 0; i < halfEdge.size(); ++i) {
        edge_ids[halfEdge[i].get()] = i;
        auto* e = halfEdge[i].get();
        std::cout << "  E" << i << ": V" << vertex_ids[e->getOrigin()] 
                  << " -> V" << vertex_ids[e->getEndPoint()] << "\n";
    }

    std::cout << "\nFace Traversal:\n";
    std::unordered_set<HalfEdge*> visited_edges;

    for (size_t fi = 0; fi < face.size(); ++fi) {
        if (!face[fi]->getOuterComponent()) continue;

        std::cout << "Face " << fi << " (" 
                  << (fi == 0 ? "outer" : "inner") << "): ";

        HalfEdge* start = face[fi]->getOuterComponent();
        HalfEdge* current = start;
        
        do {
            std::cout << "V" << vertex_ids[current->getOrigin()] << " -> ";
            visited_edges.insert(current);
            current = current->getNext();
        } while (current != start && !visited_edges.count(current));

        std::cout << "[loop]\n";
    }
}


struct VertexComparator {
    double eps;
    
    VertexComparator(double epsilon = Geometry::eps) : eps(epsilon) {}
    
    bool operator()(const std::pair<double, double>& a, 
                   const std::pair<double, double>& b) const {
        if (std::abs(a.first - b.first) > eps) {
            return a.first < b.first;
        }
        if (std::abs(a.second - b.second) > eps) {
            return a.second < b.second;
        }
        return false; // Вершины считаются равными
    }
};


void Dcel::add(const Dcel& other) {
    std::map<const Vertex*, Vertex*> vertexMap;
    std::map<const HalfEdge*, HalfEdge*> halfEdgeMap;
    std::map<const Face*, Face*> faceMap;

    
    std::map<std::pair<double, double>, Vertex*, VertexComparator> coordToVertex{VertexComparator{Geometry::eps}};  
    std::map<std::pair<Vertex*, Vertex*>, HalfEdge*> vertexToHalfEdge;

    for (const auto& v : vertex) {
        coordToVertex[{v->getX(), v->getY()}] = v.get();
    }

    for (const auto& h : halfEdge) {
        auto vvh = std::make_pair(h->getOrigin(),h->getEndPoint());
        vertexToHalfEdge[vvh] = h.get();
    }

    for (const auto& v: other.vertex) {
        auto coord = std::make_pair(v->getX(), v->getY());
        if (coordToVertex.find(coord) != coordToVertex.end()) {
            vertexMap[v.get()] = coordToVertex[coord];
        } else {
            auto ve = std::make_unique<Vertex>(*v);
            vertexMap[v.get()] = ve.get();
           // coordToVertex[coord] = ve.get();
            vertex.push_back(std::move(ve));
        }
    }
    
    for (const auto& h: other.halfEdge) {
        if (vertexToHalfEdge.find(std::make_pair(vertexMap[h->getOrigin()], vertexMap[h->getEndPoint()])) != vertexToHalfEdge.end()) {
            halfEdgeMap[h.get()] = vertexToHalfEdge[std::make_pair(vertexMap[h->getOrigin()], vertexMap[h->getEndPoint()])];
        } else {
            auto ha = std::make_unique<HalfEdge>(*h);
            halfEdgeMap[h.get()] = ha.get();
            ha->setOrigin(vertexMap[h->getOrigin()]);
            halfEdge.push_back(std::move(ha));
        }
    }


    for (const auto& f: other.face) {
        auto fa = std::make_unique<Face>(*f);
        faceMap[f.get()] = fa.get();
        fa->setOuterComponent(halfEdgeMap[f->getOuterComponent()]);
        face.push_back(std::move(fa));
    }

    for (const auto& v: other.vertex) {
        auto coord = std::make_pair(v->getX(), v->getY());
        if (coordToVertex.find(coord) == coordToVertex.end()) vertexMap[v.get()]->setIncidentEdge(halfEdgeMap[v->getIncidentEdge()]);
    }

    for (const auto& h: other.halfEdge) {
        if (vertexToHalfEdge.find(std::make_pair(h->getOrigin(), h->getEndPoint())) == vertexToHalfEdge.end()) {
            halfEdgeMap[h.get()]->setNext(halfEdgeMap[h->getNext()]);
            halfEdgeMap[h.get()]->setPrev(halfEdgeMap[h->getPrev()]);
            halfEdgeMap[h.get()]->setTwin(halfEdgeMap[h->getTwin()]);
            halfEdgeMap[h.get()]->setIncidentFace(faceMap[h->getIncidentFace()]);
        }
    }
}

void Dcel::merge(Dcel& dest, Dcel& a, Dcel& b) {
    dest.add(a);
    dest.add(b);
    
    double cur_y = 100;
    Sweepline T(cur_y);
    EventQueue<Event> Q;
   
    std::map<std::pair<double, double>, Vertex*, VertexComparator> coordToVertex{VertexComparator{Geometry::eps}};  
    std::vector<std::pair<Vertex*, std::vector<HalfEdge*>>> splits;
    for (const auto& v : dest.vertex) { // переписать
        coordToVertex[{v->getX(), v->getY()}] = v.get();
        std::vector<HalfEdge*> starts;
        std::vector<HalfEdge*> ends;
        std::cout << v->getX() << " " << v->getY() << "\n";
        for (const auto& h : dest.halfEdge) {
            if (h->getOrigin() == v.get()) {
                h->print();
                if (h->getEndPoint()->getY() < v->getY() ) {
                    starts.push_back(h.get());
                } else {
                    ends.push_back(h.get());
                }
            }
        }
        Event s = Event::eventStart(v.get());
        s.setEdges(starts);
        Event e = Event::eventEnd(v.get());
        e.setEdges(ends);
        if (!starts.empty())Q.push(s);
        if (!ends.empty())Q.push(e);
    }
   // Q.print();
    while(!Q.isEmpty()) {
        Event event = Q.pop();
       // event.print();
        cur_y = event.getVertex()->getY();
        T.setY(cur_y);
        switch (event.getType()) {
            case Event::Type::START: {
                for (HalfEdge* h: event.getEdges()) {
                    T.insert(h);
                    HalfEdge* left = T.less(event.getVertex()->getX());
                    HalfEdge* right = T.bigger(event.getVertex()->getX());
                    Vertex inter(0,0);
                    if (left && HalfEdge::segmentsIntersect(left, h, inter)) {

                        if ((left->getId() != h->getId()) && coordToVertex.find(std::make_pair(inter.getX(), inter.getY())) == coordToVertex.end()) {
                            auto v = std::make_unique<Vertex>(inter.getX(), inter.getY());
                            Event i = Event::eventInt(v.get());
                            coordToVertex[std::make_pair(inter.getX(), inter.getY())] = v.get();
                            i.setEdges({left, h});
                            Q.push(i);
                            dest.vertex.push_back(std::move(v));
                        }
                    }
                    if (right && HalfEdge::segmentsIntersect(right, h, inter)) {

                        if ((right->getId() != h->getId())&& coordToVertex.find(std::make_pair(inter.getX(), inter.getY())) == coordToVertex.end()) {
                            auto v = std::make_unique<Vertex>(inter.getX(), inter.getY());
                            coordToVertex[std::make_pair(inter.getX(), inter.getY())] = v.get();
                            Event i = Event::eventInt(v.get());
                            i.setEdges({right, h});
                            Q.push(i);
                            dest.vertex.push_back(std::move(v));
                        }
                    }
                }              
                break;
            }
            case Event::Type::END :{
                for (HalfEdge* h : event.getEdges()) {
                    T.erase(h);
                }
                HalfEdge* left = T.less(event.getVertex()->getX());
                HalfEdge* right = T.bigger(event.getVertex()->getX());
                Vertex inter(0,0);
                if (left && right &&  HalfEdge::segmentsIntersect(left, right, inter)) {
                    if ((left->getId() != right->getId())&& coordToVertex.find(std::make_pair(inter.getX(), inter.getY())) == coordToVertex.end()) {
                        auto v = std::make_unique<Vertex>(inter.getX(), inter.getY());

                        coordToVertex[std::make_pair(inter.getX(), inter.getY())] = v.get();
                        Event i = Event::eventInt(v.get());
                        i.setEdges({left, right});
                        Q.push(i);
                        dest.vertex.push_back(std::move(v));
                    }
                }
                break;
            }
            case Event::Type::INTERSECION: {
                
                //splits.push_back({event.getVertex(), event.getEdges()});
                event.getVertex()->setIncidentEdge(event.getEdges()[0]);
                
                for (HalfEdge* h: event.getEdges()) {
                    auto new_h = std::make_unique<HalfEdge>(*h);
                    new_h->setOrigin(h->getOrigin());
                    new_h->setNext(h);
                    new_h->setPrev(h->getPrev());
                    new_h->setIncidentFace(h->getIncidentFace());
                    

                    auto new_twin = std::make_unique<HalfEdge>(*h);
                    
                    new_h->setTwin(new_twin.get());
                    new_twin->setTwin(new_h.get());
                    new_twin->setIncidentFace(h->getTwin()->getIncidentFace());
                    new_twin->setPrev(h->getTwin());
                    new_twin->setNext(h->getPrev()->getTwin());
                    new_twin->setOrigin(event.getVertex());

                    h->getPrev()->setNext(new_h.get());
                    h->getPrev()->getTwin()->setPrev(new_twin.get());
                    h->getTwin()->setNext(new_twin.get());
                    h->setOrigin(event.getVertex());
                    h->setPrev(new_h.get());
                    if(new_h->getOrigin()->getIncidentEdge() == h) new_h->getOrigin()->setIncidentEdge(new_h.get());
                    event.getVertex()->setIncidentEdge(h);
                    T.erase(h);
                    dest.halfEdge.push_back(std::move(new_h));
                    dest.halfEdge.push_back(std::move(new_twin));
                }
                T.setY(cur_y-Geometry::eps);
                for (HalfEdge* h: event.getEdges()) {
                    T.insert(h);
                }
                T.setY(cur_y);
                for (HalfEdge* h: event.getEdges()) {
                    HalfEdge* left = T.less(event.getVertex()->getX()-Geometry::eps);
                    HalfEdge* right = T.bigger(event.getVertex()->getX()+Geometry::eps);
                    Vertex inter(0,0);
                    if (left && HalfEdge::segmentsIntersect(left, h, inter)) {
                        if ((left->getId() != h->getId()) && coordToVertex.find(std::make_pair(inter.getX(), inter.getY())) == coordToVertex.end()) {
                            auto v = std::make_unique<Vertex>(inter.getX(), inter.getY());
                            coordToVertex[std::make_pair(inter.getX(), inter.getY())] = v.get();
                            Event i = Event::eventInt(v.get());
                            i.setEdges({left, h});
                            Q.push(i);
                            dest.vertex.push_back(std::move(v));
                        }
                    }
                    if (right && HalfEdge::segmentsIntersect(right, h, inter)) {
                        if ((right->getId() != h->getId())&& coordToVertex.find(std::make_pair(inter.getX(), inter.getY())) == coordToVertex.end()) {
                            auto v = std::make_unique<Vertex>(inter.getX(), inter.getY());
                            coordToVertex[std::make_pair(inter.getX(), inter.getY())] = v.get();
                            Event i = Event::eventInt(v.get());
                            i.setEdges({right, h});
                            Q.push(i);
                            dest.vertex.push_back(std::move(v));
                        }
                    }
                } 
                break;
            }
        }
    }
}


void Dcel::dfs() const {
    if (vertex.empty()) {
        std::cout << "DCEL is empty!" << std::endl;
        return;
    }
    std::cout << "DFS\n";
    std::stack<const HalfEdge*> s;
    std::set<const HalfEdge*> u;
    const Vertex* start = vertex[0].get();
    u.insert(start->getIncidentEdge());
    s.push(start->getIncidentEdge());
    while (!s.empty()) {
        const HalfEdge* h = s.top();
        s.pop();
        h->print();
        if (u.find(h->getNext()) == u.end()) {
            s.push(h->getNext());
            u.insert(h->getNext());
        }
            
    }

}

void Dcel::fix() {
    std::map<Vertex*, std::vector<HalfEdge*>> s;
    for (auto& edge : halfEdge) {
        s[edge->getOrigin()].push_back(edge.get());
    }

}

void Dcel::test(Dcel& a, Dcel& b) {
    std::map<Vertex*, std::vector<HalfEdge*>> s;
    for (auto& edge : halfEdge) {
        s[edge->getOrigin()].push_back(edge.get());
    }
    vertex[5]->print();
    for (auto &now: halfEdge) {
        now->print();
    }
    return;
    this->add(a);
    this->add(b);
    this->print();
    for (const auto& v : vertex) {
        std::cout << "\nV = ";
        v->print();
        std::cout << "Incident = ";
        v->getIncidentEdge()->print();
        std::cout << "Next = ";
        v->getIncidentEdge()->getNext()->print();
        std::cout << "Prev = ";
        v->getIncidentEdge()->getPrev()->print();
        std::cout << "________________________________\n";
    }
}

