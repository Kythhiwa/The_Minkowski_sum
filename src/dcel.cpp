#include "dcel.h"
#include "vertex.h"
#include "halfEdge.h"
#include "face.h"
#include "sweepline.h"
#include "eventqueue.h"
#include "eventI.h"
#include "config.h"
#include <cinttypes>
#include <csignal>
#include "eventT.h"
int Dcel::id = 0;

Dcel::Dcel() = default;
Dcel::Dcel(std::vector<std::pair<double, double>> points) : id_(id++){
    if (points.size() < 3) {
        throw std::invalid_argument("[ERROR]Point count < 3\n");
    }
    for (const auto& [x, y] : points) {
        vertex.push_back(std::make_unique<Vertex>(x, y));
    }
    face.push_back(std::make_unique<Face>(Face::Type::INNER));
    face.push_back(std::make_unique<Face>(Face::Type::OUTER));
    
    for (size_t i = 0; i < vertex.size(); ++i) {
        vertex[i]->setId(id_);
        size_t next_i = (i + 1) % vertex.size();

        auto out = std::make_unique<HalfEdge>();
        auto in = std::make_unique<HalfEdge>();
        out->setId(id_);
        in->setId(id_);

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
                h->getOrigin()->setY(h->getOrigin()->getY() + 10* Geometry::eps);
            } else {
                h->getEndPoint()->setY(h->getEndPoint()->getY() +10* Geometry::eps);
            }
        }
    }
    for (size_t i = 0; i < face.size(); ++i) {
        face[i]->setId(id_);
    }
}

const std::vector<Vertex*> Dcel::getVertex() const {
    std::vector<Vertex*> a;
    for (auto& v: vertex) {
        a.push_back(v.get());
    }
    return a;
}

const std::vector<HalfEdge*> Dcel::getHalfEdge() const {
    std::vector<HalfEdge*> a;
    for (auto& h: halfEdge) {
        a.push_back(h.get());
    }
    return a;
}

const std::vector<Face*> Dcel::getFace() const {
    std::vector<Face*> a;
    for (auto& f : face) {
        a.push_back(f.get());
    }
    return a;
}


void Dcel::setHoles(std::vector<std::pair<double, double>> points) {
    
    if (points.size() < 3) {
        throw std::invalid_argument("[ERROR]Point count < 3\n");
    }
    std::vector<std::unique_ptr<Vertex>> vert;
    std::vector<std::unique_ptr<HalfEdge>> half;
    std::vector<std::unique_ptr<Face>> fac;
    for (const auto& [x, y] : points) {
        vert.push_back(std::make_unique<Vertex>(x, y));
    }
    fac.push_back(std::make_unique<Face>(Face::Type::HOLES));
    fac.push_back(std::make_unique<Face>(Face::Type::INNER));
    
    for (size_t i = 0; i < vert.size(); ++i) {
        vert[i]->setId(id_);
        size_t next_i = (i + 1) % vert.size();

        auto out = std::make_unique<HalfEdge>();
        auto in = std::make_unique<HalfEdge>();
        out->setId(id_);
        in->setId(id_);

        out->setOrigin(vert[i].get());
        in->setOrigin(vert[next_i].get());
    
        out->setTwin(in.get());
        in->setTwin(out.get());

        out->setIncidentFace(fac[1].get());
        in->setIncidentFace(fac[0].get());

        vert[i]->setIncidentEdge(out.get());
        half.push_back(std::move(out));
        half.push_back(std::move(in));
    }
       
    for (size_t i = 0; i < half.size(); i += 2) {
        size_t next_i_in = (i + 2) % half.size();
        half[i]->setNext(half[next_i_in].get());
        half[next_i_in]->setPrev(half[i].get());

        size_t next_i_ou = (i + 3) % half.size();
        half[i + 1]->setPrev(half[next_i_ou].get());
        half[next_i_ou]->setNext(half[i + 1].get());
    }
    fac[0]->setOuterComponent(half[1].get());
    fac[1]->setOuterComponent(half[0].get());
    
    for (auto& h: half) {
        if (h->isHorizontal()){
            if (h->getOrigin()->getX() > h->getEndPoint()->getX()) {
                h->getOrigin()->setY(h->getOrigin()->getY() + 10*Geometry::eps);
            } else {
                h->getEndPoint()->setY(h->getEndPoint()->getY() +10* Geometry::eps);
            }
        }
    }
    for (size_t i = 0; i < fac.size(); ++i) {
        fac[i]->setId(id_);
    }
    for (auto& f : face) {
        if (f->getType() == Face::Type::INNER) {
            f->setHole(half[1].get());
            break;
        }
    }
    //fac[0]->setOuterComponent(halfEdge[1].get());
    for (auto& v : vert){
        vertex.push_back(std::move(v));
    }
    for (auto& h : half){
        halfEdge.push_back(std::move(h));
    }
    
    for (auto& f : fac){
        face.push_back(std::move(f));
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

        std::cout << "Face " << fi << "( ";
        switch (face[fi]->getType()) {
            case Face::Type::INNER:
                std::cout << "INNER";
                break;
            case Face::Type::OUTER:
                std::cout << "OUTER";
                break;
            case Face::Type::HOLES:
                std::cout << "HOLES";
                break;
        }
        std::cout << ") ";
        if (isCounterClockwise(face[fi].get())) std::cout << " CCW ";
        else std::cout << " CW ";
        HalfEdge* start = face[fi]->getOuterComponent();
        HalfEdge* current = start;
        
        do {
            std::cout << "V" << vertex_ids[current->getOrigin()] << " -> ";
            visited_edges.insert(current);
            current = current->getNext();
        } while (current != start && !visited_edges.count(current));

        std::cout << "[loop]\n";
    }
    std::cout << "\nDFS: \n";
    std::unordered_set<HalfEdge*> vis;
    for (const auto& h : halfEdge) {
        if (vis.find(h.get()) != vis.end()) continue;
        HalfEdge* start = h.get();
        HalfEdge* cur = start;
        do {
            std::cout << "V" << vertex_ids[cur->getOrigin()] << " -> ";
            vis.insert(cur);
            cur = cur->getNext();
        } while (cur != start);
        std::cout << "[loop]\n";
    }
   
   
}


struct VertexComparator {
    double eps;
    
    VertexComparator(double epsilon = Geometry::eps) : eps(epsilon) {}
    
    bool operator()(const std::pair<double, double>& a, 
                   const std::pair<double, double>& b) const {
        if (std::abs(a.first - b.first) >= eps) {
            return a.first < b.first;
        }
        if (std::abs(a.second - b.second) >= eps) {
            return a.second < b.second;
        }
        return false;
    }
};

double calculateAngle(const HalfEdge* edge) {
    double dx = edge->getEndPoint()->getX() - edge->getOrigin()->getX();
    double dy = edge->getEndPoint()->getY() - edge->getOrigin()->getY();
    return std::atan2(dy, dx);
}

bool compareEdgesByAngle(const HalfEdge* a, const HalfEdge* b) {
    return calculateAngle(a) < calculateAngle(b);
}

void sortByAngle(std::vector<HalfEdge*>& e) {
    std::sort(e.begin(), e.end(), compareEdgesByAngle);
}

bool Dcel::isCounterClockwise(Face* face) const{
    if (!face || !face->getOuterComponent()) return false;

    double area = 0.0;
    HalfEdge* startEdge = face->getOuterComponent();
    HalfEdge* edge = startEdge;
    
    do {
        Vertex* v1 = edge->getOrigin();
        Vertex* v2 = edge->getTwin()->getOrigin();
        
        area += (v2->getX() - v1->getX()) * (v2->getY() + v1->getY());
        edge = edge->getNext();
    } while (edge != startEdge);

    return area < 0; // Если area отрицательная - CCW, положительная - CW
}
Face* Dcel::findFacePoint(double x, double y) const {
    Face* fac = nullptr;
    double s = 10000000000;
    for (const auto& f : face){
        Face* cur = f.get();
        if (!cur) continue;
        if (isCounterClockwise(cur)) continue;
        HalfEdge *start = cur->getOuterComponent();
        if (!start) continue;
        int cross = 0;
        HalfEdge *edge = start;
        double c = 100000000000;
        do {
            Vertex* v1 = edge->getOrigin();
            Vertex* v2 = edge->getTwin()->getOrigin();

            double x1 = v1->getX(), y1 = v1->getY();
            double x2 = v2->getX(), y2 = v2->getY();

            if ((y1 > y) != (y2 > y)) {
                double intersect_x = (y - y1) * (x2 - x1) / (y2 - y1) + x1;
                if (x < intersect_x) {
                    cross++;
                    c = std::min(c, intersect_x);
                }
            }
            edge = edge->getNext();
        } while (edge != start);
    
        if (cross % 2 == 1) {
            if (c < s) {
                s = c;
                fac = cur;
            }
        }
    }

    return fac;
}


void Dcel::add(const Dcel& other) {
    std::map<const Vertex*, Vertex*> vertexMap;
    std::map<const HalfEdge*, HalfEdge*> halfEdgeMap;
    std::map<const Face*, Face*> faceMap;

    std::map<std::pair<double, double>, Vertex*, VertexComparator> existVertex;
    for (const auto& v: vertex) {
        existVertex[{v->getX(), v->getY()}] = v.get();
    }

    std::set<HalfEdge*> del;
    for (const auto& v: other.vertex) {
        auto key = std::make_pair(v->getX(), v->getY());
        auto it = existVertex.find(key);
        if (it != existVertex.end()) {
            vertexMap[v.get()] = it->second;
            vertexMap[v.get()]->print();
        } else {
            auto ve = std::make_unique<Vertex>(*v);
            vertexMap[v.get()] = ve.get();
            vertex.push_back(std::move(ve));
        }
    }
    for (const auto& h: other.halfEdge) {
        auto key1 = std::make_pair(h->getOrigin()->getX(), h->getOrigin()->getY());
        auto key2 = std::make_pair(h->getEndPoint()->getX(), h->getEndPoint()->getY());
        auto it1 = existVertex.find(key1);
        auto it2 = existVertex.find(key2);
        if (it1 != existVertex.end() && it2 != existVertex.end()) {
            HalfEdge* cur = it1->second->getIncidentEdge();
            do {
                cur = cur->getTwin()->getNext();
            } while (cur->getEndPoint() != it2->second);
             std::cout << "#################dd\n";
            cur->print();
            halfEdgeMap[h.get()] = cur;
            del.insert(cur);
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
         auto key = std::make_pair(v->getX(), v->getY());
        auto it = existVertex.find(key);
        if (it != existVertex.end()) continue;

        vertexMap[v.get()]->setIncidentEdge(halfEdgeMap[v->getIncidentEdge()]);
    }

    for (const auto& h: other.halfEdge) {
        auto key1 = std::make_pair(h->getOrigin()->getX(), h->getOrigin()->getY());
        auto key2 = std::make_pair(h->getEndPoint()->getX(), h->getEndPoint()->getY());
        auto it1 = existVertex.find(key1);
        auto it2 = existVertex.find(key2);
        if (it1 != existVertex.end() && it2 != existVertex.end()) continue;
        halfEdgeMap[h.get()]->setNext(halfEdgeMap[h->getNext()]);
        halfEdgeMap[h.get()]->setPrev(halfEdgeMap[h->getPrev()]);
        halfEdgeMap[h.get()]->setTwin(halfEdgeMap[h->getTwin()]);
        halfEdgeMap[h.get()]->setIncidentFace(faceMap[h->getIncidentFace()]);
    }
    std::map<Vertex*, std::vector<HalfEdge*>> s;
    for (const auto& h : halfEdge) {
       s[h->getEndPoint()].push_back(h.get());
    }
    for (auto [v, vh] : s) {
        sortByAngle(vh);
        for (size_t i = 0; i < vh.size(); i++) {
            vh[i]->setNext(vh[(i + 1) % vh.size()]->getTwin());
            vh[i]->getTwin()->setPrev(vh[(i - 1 + vh.size()) % vh.size()]);
        }
    }
}

void Dcel::copy(const Dcel& other) {
    std::map<const Vertex*, Vertex*> vertexMap;
    std::map<const HalfEdge*, HalfEdge*> halfEdgeMap;
    std::map<const Face*, Face*> faceMap;
    std::set<HalfEdge*> del;
    for (const auto& v: other.vertex) {
        auto ve = std::make_unique<Vertex>(*v);
        vertexMap[v.get()] = ve.get();
        vertex.push_back(std::move(ve));
    }

    for (const auto& h: other.halfEdge) {       
        auto ha = std::make_unique<HalfEdge>(*h);
        halfEdgeMap[h.get()] = ha.get();
        ha->setOrigin(vertexMap[h->getOrigin()]);
        halfEdge.push_back(std::move(ha));
    }

    for (const auto& f: other.face) {
        auto fa = std::make_unique<Face>(*f);
        faceMap[f.get()] = fa.get();
        
        fa->setOuterComponent(halfEdgeMap[f->getOuterComponent()]);
        face.push_back(std::move(fa));
    }

    for (const auto& v: other.vertex) {
        vertexMap[v.get()]->setIncidentEdge(halfEdgeMap[v->getIncidentEdge()]);
    }

    for (const auto& h: other.halfEdge) {
        halfEdgeMap[h.get()]->setNext(halfEdgeMap[h->getNext()]);
        halfEdgeMap[h.get()]->setPrev(halfEdgeMap[h->getPrev()]);
        halfEdgeMap[h.get()]->setTwin(halfEdgeMap[h->getTwin()]);
        halfEdgeMap[h.get()]->setIncidentFace(faceMap[h->getIncidentFace()]);
    }

}

bool isPointInsidePolygon(const std::vector<std::pair<double, double>>& polygon, double x, double y) {
    bool inside = false;
    size_t n = polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon[i].first, yi = polygon[i].second;
        double xj = polygon[j].first, yj = polygon[j].second;

        if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

std::pair<double, double> Dcel::getInnerPoint(HalfEdge* startEdge) const {
    Vertex* v1 = startEdge->getOrigin();
    Vertex* v2 = startEdge->getTwin()->getOrigin();
    
    double center_x = (v1->getX() + v2->getX()) / 2.0;
    double center_y = (v1->getY() + v2->getY()) / 2.0;
    
    double dir_x = v2->getX() - v1->getX();
    double dir_y = v2->getY() - v1->getY();
    
    double norm_x = dir_y;
    double norm_y = -dir_x;
    
    double length = std::sqrt(norm_x * norm_x + norm_y * norm_y);
    if (length > Geometry::eps) {
        norm_x /= length;
        norm_y /= length;
    }
    
    const double offset = Geometry::eps * 10; 
    
    double inner_x = center_x + norm_x * offset;
    double inner_y = center_y + norm_y * offset;
    
    return {inner_x, inner_y};
}

void Dcel::solve(Dcel& dest, Dcel& a, Dcel& b) {
    std::cout << "ASDASFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFfD\n";
    dest.face.clear();
    b.print();
    std::set<HalfEdge*> us;
    for (const auto& h : dest.halfEdge) {
        if (us.find(h.get()) != us.end()) continue;
        auto fac = std::make_unique<Face>(dest.vertex[0]->getId());
        
        HalfEdge* start = h.get();
        HalfEdge* cur = start;
        std::pair<double, double> p = dest.getInnerPoint(start);
        //std::cout << p.first << " " << p.second << "\n";
        fac->setOuterComponent(start);
        std::cout << "START\n";
        do {
            cur->print();
            us.insert(cur);
            cur->setIncidentFace(fac.get());
            cur = cur->getNext();
        
        } while(start != cur);
        std::cout << "END\n";

        std::cout  << std::setprecision(10)<< p.first << " " << p.second << "\n";
        Face *f = a.findFacePoint(p.first, p.second), *s = b.findFacePoint(p.first, p.second);
        if (f != nullptr) {
            HalfEdge* start = f->getOuterComponent();
            HalfEdge* cur = start;
            std::cout << "START_FACE_GO\n";
            do {
                cur->print();
                cur = cur->getNext();
            } while(cur != start);
            std::cout << (int)f->getType();
            std::cout << "END_FACE_GO\n";
        } else {
            std::cout << "FNULL\n";
        }
        if (s != nullptr) {
            HalfEdge* start = s->getOuterComponent();
            HalfEdge* cur = start;
            std::cout << "START_FACE_GO\n";
            do {
                cur->print();
                cur = cur->getNext();
            } while(cur != start);
            std::cout << (int)s->getType();
            std::cout << "END_FACE_GO\n";
        } else {
            std::cout << "FNULL\n";
        }

        if (f == nullptr && s == nullptr ){
            if (dest.isCounterClockwise(fac.get())) {
                fac->setType(Face::Type::OUTER);
                std::cout << "OUTER\n";
                std::cout << p.first << ":" << p.second <<"\n";
            } else {
                std::cout << "HOLES\n";
                 std::cout << p.first << ":" << p.second <<"+\n";

                fac->setType(Face::Type::HOLES);
            }
        }

        else if (f == nullptr) {
            if (s->getType() == Face::Type::INNER) {
                fac->setType(Face::Type::INNER);
                std::cout << "INNER1\n";
                std::cout << p.first << ":" << p.second <<"\n";
            }
            if (s->getType() == Face::Type::HOLES)  {
                fac->setType(Face::Type::HOLES);
                std::cout << "HOLES1\n";
                std::cout << p.first << ":" << p.second <<"\n";
         
            }
        }
        else if (s == nullptr) {
            if (f->getType() == Face::Type::INNER) {
                fac->setType(Face::Type::INNER);
                std::cout << "INNER2\n";
                std::cout << p.first << ":" << p.second <<"\n";
            }
            if (f->getType() == Face::Type::HOLES){
                std::cout << "HOLES2\n";
                std::cout << p.first << ":" << p.second <<"HOLES\n";
                fac->setType(Face::Type::HOLES);
            }

        }
        else if (f->getType() == Face::Type::INNER && s->getType() == Face::Type::INNER) fac->setType(Face::Type::INNER);
        else if (f->getType() == Face::Type::HOLES && s->getType() == Face::Type::HOLES) fac->setType(Face::Type::HOLES);
        else if (f->getType() == Face::Type::INNER && s->getType() == Face::Type::HOLES) fac->setType(Face::Type::INNER);
        else if (f->getType() == Face::Type::HOLES && s->getType() == Face::Type::INNER) fac->setType(Face::Type::INNER);
        dest.face.push_back(std::move(fac));
    }
    for (const auto& v : dest.vertex) {
        v->setId(dest.id_);
    }
    for (const auto& h : dest.halfEdge) {
        h->setId(dest.id_);
    }
    for (const auto& f : dest.face) {
        f->setId(dest.id_);
    }
    std::vector<std::unique_ptr<Vertex>> vert;
    std::vector<std::unique_ptr<HalfEdge>> half;
    std::vector<std::unique_ptr<Face>> fac;
    for (const auto& f : dest.face) {
        if (f->getType() == Face::Type::OUTER || f->getType() == Face::Type::HOLES) {
            HalfEdge* cur = f->getOuterComponent();
            std::vector<std::unique_ptr<Vertex>> ve;
            std::vector<std::unique_ptr<HalfEdge>> hal;
            std::vector<std::unique_ptr<Face>> fa;
            do{
                auto v = std::make_unique<Vertex> (*cur->getOrigin());
                ve.push_back(std::move(v));
                cur = cur->getNext();
            } while(cur != f->getOuterComponent());
            if (f->getType() == Face::Type::OUTER) {
                fa.push_back(std::make_unique<Face>(Face::Type::INNER));
                fa.push_back(std::make_unique<Face>(Face::Type::OUTER));
            } else {
                fa.push_back(std::make_unique<Face>(Face::Type::INNER));

                fa.push_back(std::make_unique<Face>(Face::Type::HOLES));
            }
            
            for (size_t i = 0; i < ve.size(); ++i) {
                ve[i]->setId(dest.id_);
                size_t next_i = (i + 1) % ve.size();

                auto out = std::make_unique<HalfEdge>();
                auto in = std::make_unique<HalfEdge>();
                out->setId(dest.id_);
                in->setId(dest.id_);

                out->setOrigin(ve[i].get());
                in->setOrigin(ve[next_i].get());
            
                out->setTwin(in.get());
                in->setTwin(out.get());

                out->setIncidentFace(fa[1].get());
                in->setIncidentFace(fa[0].get());

                ve[i]->setIncidentEdge(out.get());
                hal.push_back(std::move(out));
                hal.push_back(std::move(in));
            }
               
            for (size_t i = 0; i < hal.size(); i += 2) {
                size_t next_i_in = (i + 2) % hal.size();
                hal[i]->setNext(hal[next_i_in].get());
                hal[next_i_in]->setPrev(hal[i].get());

                size_t next_i_ou = (i + 3) % hal.size();
                hal[i + 1]->setPrev(hal[next_i_ou].get());
                hal[next_i_ou]->setNext(hal[i + 1].get());
            }
            fa[0]->setOuterComponent(hal[1].get());
            fa[1]->setOuterComponent(hal[0].get());
            

            for (auto& i : ve) {
                vert.push_back(std::move(i));
            }

            for (auto& i : hal) {
                half.push_back(std::move(i));
            }
            for (auto& i : fa) {
                fac.push_back(std::move(i));
            }
        }
    }
      dest.vertex.clear();

    dest.halfEdge.clear();

    dest.face.clear();
    for (auto &now : vert) {
        now->setId(dest.id_);
        dest.vertex.push_back(std::move(now));
    }
    for (auto &now : half) {

        now->setId(dest.id_);
       dest.halfEdge.push_back(std::move(now));
     }
    for (auto &now : fac) {

        now->setId(dest.id_);
        dest.face.push_back(std::move(now));
    }
}

void Dcel::merge(Dcel& dest, Dcel& a) {
    dest.id_ = id++;
    Dcel b;
    b.add(dest);
    dest.add(a);
    double cur_y = 1000;
    Sweepline T(cur_y);
    EventQueue<Event> Q;
   
    std::map<std::pair<double, double>, Vertex*, VertexComparator> coordToVertex{VertexComparator{Geometry::eps}};  
    std::map<Vertex*, std::vector<HalfEdge*>> starts;
    std::map<Vertex*, std::vector<HalfEdge*>> ends;
    
    for (const auto& v : dest.vertex) {
        coordToVertex[{v->getX(), v->getY()}] = v.get();
    }

    for (const auto& h : dest.halfEdge) {
         if (h->getEndPoint()->getY() < h->getOrigin()->getY() ) {
            starts[h->getOrigin()].push_back(h.get());
        } else {
            ends[h->getOrigin()].push_back(h.get());
        }
    }
    

    for (const auto [v, vh]: starts) {
        Event s = Event::eventStart(v);
        s.setEdges(vh);
        if (!vh.empty()) Q.push(s);
    }

    for (const auto [v, vh]: ends) {
        Event e = Event::eventEnd(v);
        e.setEdges(vh);
        if (!vh.empty()) Q.push(e);
    }
    std::map<Vertex*, std::vector<HalfEdge*>> sp;
    while(!Q.isEmpty()) {

        Event event = Q.pop();
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
                            auto v = std::make_unique<Vertex>(inter.getX(), inter.getY(), dest.id_);
                            Event i = Event::eventInt(v.get());
                            coordToVertex[std::make_pair(inter.getX(), inter.getY())] = v.get();
                            i.setEdges({left, h});
                            Q.push(i);
                            dest.vertex.push_back(std::move(v));
                        }
                    }
                    if (right && HalfEdge::segmentsIntersect(right, h, inter)) {

                        if ((right->getId() != h->getId())&& coordToVertex.find(std::make_pair(inter.getX(), inter.getY())) == coordToVertex.end()) {
                            auto v = std::make_unique<Vertex>(inter.getX(), inter.getY(), dest.id_);
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
                        auto v = std::make_unique<Vertex>(inter.getX(), inter.getY(), dest.id_);

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
                
                
                for (HalfEdge* h: event.getEdges()) {
                    T.erase(h);
                    h->print();
                }
                std::vector<HalfEdge*> splits;
                T.setY(cur_y-Geometry::eps);
                for (HalfEdge* h: event.getEdges()) {
                    T.insert(h);
                    auto new_h = std::make_unique<HalfEdge>(*h);
                    new_h->setOrigin(h->getOrigin());
                    new_h->setNext(h);
                    new_h->setPrev(h->getPrev());
                    new_h->setIncidentFace(h->getIncidentFace());
                    

                    auto new_twin = std::make_unique<HalfEdge>(*h->getTwin());
                    
                    //new_h->setId(dest.id_);
                   // new_twin->setId(dest.id_);

                    new_h->setTwin(new_twin.get());

                    new_twin->setOrigin(event.getVertex());
                    new_twin->setTwin(new_h.get());
                    new_twin->setIncidentFace(h->getTwin()->getIncidentFace());
                    new_twin->setPrev(h->getTwin());
                    new_twin->setNext(h->getTwin()->getNext());

                    h->getPrev()->setNext(new_h.get());
                    h->getTwin()->getNext()->setPrev(new_twin.get());
                    h->getTwin()->setNext(new_twin.get());
                    h->setOrigin(event.getVertex());
                    h->setPrev(new_h.get());
                    if(new_h->getOrigin()->getIncidentEdge() == h) new_h->getOrigin()->setIncidentEdge(new_h.get());
                    if(event.getVertex()->getIncidentEdge() == nullptr) event.getVertex()->setIncidentEdge(h);
                    splits.push_back(new_h.get());
                    dest.halfEdge.push_back(std::move(new_h));
                    dest.halfEdge.push_back(std::move(new_twin));
                    splits.push_back(h->getTwin());
                }

                sortByAngle(splits);
                for (size_t i = 0; i < splits.size(); i++) {
                    splits[i]->setNext(splits[(i + 1) % splits.size()]->getTwin());
                    splits[i]->getTwin()->setPrev(splits[(i - 1 + splits.size()) % splits.size()]);

                    
                }
                T.setY(cur_y);
                for (HalfEdge* h: event.getEdges()) {
                    HalfEdge* left = T.less(event.getVertex()->getX()-Geometry::eps);
                    HalfEdge* right = T.bigger(event.getVertex()->getX()+Geometry::eps);
                    Vertex inter(0,0);
                    if (left && HalfEdge::segmentsIntersect(left, h, inter)) {
                        if ((left->getId() != h->getId()) && coordToVertex.find(std::make_pair(inter.getX(), inter.getY())) == coordToVertex.end()) {
                            auto v = std::make_unique<Vertex>(inter.getX(), inter.getY(), dest.id_);
                            coordToVertex[std::make_pair(inter.getX(), inter.getY())] = v.get();
                            Event i = Event::eventInt(v.get());
                            i.setEdges({left, h});
                            Q.push(i);
                            dest.vertex.push_back(std::move(v));
                        }
                    }
                    if (right && HalfEdge::segmentsIntersect(right, h, inter)) {
                        if ((right->getId() != h->getId())&& coordToVertex.find(std::make_pair(inter.getX(), inter.getY())) == coordToVertex.end()) {
                            auto v = std::make_unique<Vertex>(inter.getX(), inter.getY(), dest.id_);
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





    
    solve(dest, a, b);

}


Event_T::Type determVertType(Vertex *v, Vertex *next, Vertex *prev, bool isHole) {
    double crossProduct = (prev->getX() - v->getX()) * (next->getY() - v->getY()) - 
                         (prev->getY() - v->getY()) * (next->getX() - v->getX());
    
    bool isConvex = (crossProduct > 0); // CCW - выпуклая вершина
    bool isReflex = (crossProduct < 0);
    bool prevAbove = prev->getY() > v->getY();
    bool nextAbove = next->getY() > v->getY();
    bool prevBelow = prev->getY() < v->getY();
    bool nextBelow = next->getY() < v->getY();

    if (prevBelow && nextBelow) {
        if (isConvex && !isHole) {
            return Event_T::Type::START;
        } else {
            return Event_T::Type::SPLIT;
        }
    }else if (prevAbove && nextAbove) {
        if (isConvex && !isHole) {
            return Event_T::Type::END;
        } else {
            return Event_T::Type::MERGE;
        }
    } else {
        return Event_T::Type::REGULAR;
    }
}

bool isRightChain(Event_T e) {
    if (e.getNext()->getEndPoint()->getY() < e.getPrev()->getOrigin()->getY()) {
        return false;
    }
    return true;
}

void Dcel::setDiag(Event_T x, Event_T y) {
    Vertex*a = x.getVertex();
    Vertex*b = y.getVertex();
    HalfEdge *prev = x.getPrev(), *next = y.getPrev();

    auto out = std::make_unique<HalfEdge>();
    auto in = std::make_unique<HalfEdge>();
    out->setPrev(prev);
    out->setNext(next->getTwin());
    in->setPrev(next);
    in->setNext(prev->getTwin());

    out->setId(a->getId());
    in->setId(a->getId());
    out->setTwin(in.get());
    in->setTwin(out.get());
    out->setOrigin(a);
    in->setOrigin(b);
    halfEdge.push_back(std::move(out));
    halfEdge.push_back(std::move(in));
}

void Dcel::normalize(Dcel& a) {
    std::map<Vertex*, std::vector<HalfEdge*>> s;
    for (const auto& h : halfEdge) {
       s[h->getTwin()->getOrigin()].push_back(h.get());
    }
    for (auto [v, vh] : s) {
        sortByAngle(vh);
        for (size_t i = 0; i < vh.size(); i++) {
            vh[i]->setNext(vh[(i + 1) % vh.size()]->getTwin());
            vh[i]->getTwin()->setPrev(vh[(i - 1 + vh.size()) % vh.size()]);
        }
    }
    face.clear();
    std::set<HalfEdge*> us;
    for (const auto& h : halfEdge) {
        if (us.find(h.get()) != us.end()) continue;
        auto fac = std::make_unique<Face>(vertex[0]->getId());
        
        HalfEdge* start = h.get();
        HalfEdge* cur = start;
        std::pair<double, double> p = getInnerPoint(start);
        std::cout << p.first << " " << p.second << "\n";
        fac->setOuterComponent(start);
        do {

            us.insert(cur);
            cur->setIncidentFace(fac.get());
            cur = cur->getNext();
        
        } while(start != cur);
        Face *f = a.findFacePoint(p.first, p.second);
        if (f == nullptr){
            if (isCounterClockwise(fac.get())) {
                fac->setType(Face::Type::OUTER);

            std::cout << p.first << ":" << p.second <<"+OUT++\n";
            } else {
                 std::cout << p.first << ":" << p.second <<"+HO++\n";

                fac->setType(Face::Type::HOLES);
            }
        }
        else if (f->getType() == Face::Type::INNER) fac->setType(Face::Type::INNER);
        else if (f->getType() == Face::Type::HOLES) fac->setType(Face::Type::HOLES);
        face.push_back(std::move(fac));
    }
    for (const auto& v : vertex) {
        v->setId(id_);
    }
    for (const auto& h : halfEdge) {
        h->setId(id_);
    }
    for (const auto& f : face) {
        f->setId(id_);
    }

}

struct VertexYComparator {
    bool operator()(Vertex* a, Vertex* b) const {
        if (a->getY() != b->getY()) {
            return a->getY() > b->getY();
        }
        return a->getX() < b->getX();
    }
};

bool IsValidDiagonal(Vertex* current, Vertex* prev, Vertex* prevprev, bool f) {

    double dx1 = prev->getX() - prevprev->getX();
    double dy1 = prev->getY() - prevprev->getY();
    
    double dx2 = current->getX() - prevprev->getX();
    double dy2 = current->getY() - prevprev->getY();
    
    double cross = dx1 * dy2 - dy1 * dx2;
    if (std::abs(cross) < Geometry::eps) return false;
    if (f) {
        return cross < 0;
    }
    return cross > 0;
}


void Dcel::addDiag(HalfEdge* f, HalfEdge *s) {
    Vertex* a = f->getOrigin();
    Vertex* b = s->getOrigin();
    auto out = std::make_unique<HalfEdge>(*f);
    auto in = std::make_unique<HalfEdge>(*f);
    out->setTwin(in.get());
    in->setTwin(out.get());
    out->setOrigin(a);
    in->setOrigin(b);
    in->setNext(f);
    in->setPrev(s->getPrev());
    s->getPrev()->setNext(in.get());
    out->setNext(s);
    out->setPrev(f->getPrev());
    f->getPrev()->setNext(out.get());
    f->setPrev(in.get());
    s->setPrev(out.get());
    in->setIncidentFace(f->getIncidentFace());
    out->setIncidentFace(s->getIncidentFace());
    halfEdge.push_back(std::move(out));
    halfEdge.push_back(std::move(in));
}

std::vector<std::vector<std::pair<double, double>>> Dcel::triang() {
    Dcel a;
    a.add(*this);
    std::vector<std::vector<std::pair<double, double>>> res;
    double cur_y = 1000;
    Sweepline T(cur_y);
    EventQueue<Event_T> Q;
    std::map<Vertex*, Event_T> type;
    for (auto& f : face) {
        if (f->getType() == Face::Type::OUTER) {
            HalfEdge *start = f->getOuterComponent();
            HalfEdge *cur = start;
            do {
                Event_T::Type t = determVertType(cur->getOrigin(), cur->getPrev()->getOrigin(), cur->getEndPoint(), false);
                type[cur->getOrigin()] = Event_T(cur->getOrigin(), t, cur->getPrev(), cur);
                Q.push(Event_T(cur->getOrigin(), t, cur->getPrev(), cur));
                cur = cur->getNext();
            } while(cur!= start);
        } else if (f->getType() == Face::Type::HOLES) {
            HalfEdge *start = f->getOuterComponent();
            HalfEdge *cur = start;
            do {
                Event_T::Type t = determVertType(cur->getOrigin(), cur->getPrev()->getOrigin(), cur->getEndPoint(), true);
                type[cur->getOrigin()] = Event_T(cur->getOrigin(), t, cur->getPrev(), cur);
                Q.push(Event_T(cur->getOrigin(), t, cur->getPrev(), cur));

                cur = cur->getNext();
            } while(cur!= start);
        }
    }
    Q.print();
    std::map<HalfEdge*, Vertex*> helper;
    while (!Q.isEmpty()) {
        Event_T event = Q.pop();
        cur_y = event.getVertex()->getY();
        T.setY(cur_y);
        
        switch (event.getType()) {
            case Event_T::Type::START: 
                T.insert(event.getNext());
                helper[event.getNext()] = event.getVertex();
                break;
            case Event_T::Type::SPLIT: {
                HalfEdge* left = T.less(event.getVertex()->getX());
                if (left != nullptr) {
                    std::cout << "DIAG\n";
                    helper[left]->print();
                    event.getVertex()->print();
                    std::cout << "ENDDIAG\n";
                    setDiag(type[helper[left]], event);
                    helper[left] = event.getVertex();
                    helper[event.getNext()] = event.getVertex();
                    T.insert(event.getNext());
                }
                break;
            } case Event_T::Type::END: {
                if (type[helper[event.getPrev()]].getType() == Event_T::Type::MERGE) {
                    std::cout << "DIAG\n";
                    helper[event.getPrev()]->print();
                    event.getVertex()->print();
                    std::cout << "ENDDIAG\n";
                    setDiag(type[helper[event.getPrev()]], event);
                }
                T.erase(event.getPrev());
                break;
            } case Event_T::Type::MERGE : {
                 if (type[helper[event.getPrev()]].getType() == Event_T::Type::MERGE) {
                    std::cout << "DIAG\n";
                    helper[event.getPrev()]->print();
                    event.getVertex()->print();
                    std::cout << "ENDDIAG\n";
                    setDiag(type[helper[event.getPrev()]], event);
                }
                T.erase(event.getPrev());

                HalfEdge* left = T.less(event.getVertex()->getX());
                if (left != nullptr) {
                    if (type[helper[left]].getType() == Event_T::Type::MERGE) {
                        std::cout << "DIAG\n";
                        helper[left]->print();
                        event.getVertex()->print();
                        std::cout << "ENDDIAG\n";
                        setDiag(type[helper[left]], event);
                    }
                }
                helper[left] = event.getVertex();
                break;
            } case Event_T::Type::REGULAR : {
                if (!isRightChain(event)) {
                    if (type[helper[event.getPrev()]].getType() == Event_T::Type::MERGE) {
                        std::cout << "DIAG\n";
                        helper[event.getPrev()]->print();
                        event.getVertex()->print();
                        std::cout << "ENDDIAG\n";
                        setDiag(type[helper[event.getPrev()]], event);
                    }
                    T.erase(event.getPrev());
                    T.insert(event.getNext());
                    helper[event.getNext()] = event.getVertex();
                } else {
                    HalfEdge* left = T.less(event.getVertex()->getX());
                    if (left != nullptr) {
                        if (type[helper[left]].getType() == Event_T::Type::MERGE) {
                            std::cout << "DIAG\n";
                            helper[left]->print();
                            event.getVertex()->print();
                            std::cout << "ENDDIAG\n";

                            setDiag(type[helper[left]], event);
                        }
                    }
                    helper[left] = event.getVertex();

                }
                break;
            }
            
        }
        
    }
    
    this->normalize(a);
    std::set<HalfEdge*> us;
    std::vector<HalfEdge*> hh;
    for (const auto& now : halfEdge) hh.push_back(now.get());
    for (const auto& h : hh) {
        if (us.find(h) != us.end() || h->getIncidentFace()->getType() != Face::Type::INNER ) continue;
        triangulateMonotonePolygon(h, us);
        this->normalize(a);
    }
        

    return res;
}


void Dcel::triangulateMonotonePolygon(HalfEdge* h, std::set<HalfEdge*> &us) {
    std::vector<Vertex*> V;
    HalfEdge* start = h;
    HalfEdge* cur = start;
    std::set<Vertex*> rightChain;
    std::set<Vertex*> leftChain;
    std::map<Vertex*, HalfEdge*> m;
    do {
        m[cur->getOrigin()] = cur;
        us.insert(cur);
        V.push_back(cur->getOrigin());
        cur = cur->getNext();
    } while (start != cur);
    std::sort(V.begin(), V.end(), VertexYComparator());
    Vertex* H = V[0];
    Vertex* L = V[V.size() - 1];
    cur = m[H];
    std::cout << "VECTOR\n";
    std::cout << std::setprecision(16);
    for (auto now : V) now->print();
    std::cout << "ENDVECTOR\n";

    rightChain.insert(H);
    rightChain.insert(L);

    leftChain.insert(H);
    leftChain.insert(L);
    do {
        rightChain.insert(cur->getOrigin());
        cur = cur->getNext();
    } while(cur->getOrigin() != L);
    cur = m[L];
    do {
        leftChain.insert(cur->getOrigin());
        cur = cur->getNext();
    } while(cur->getOrigin() != H);
    std::cout  << "LEFTCHAIN\n";
    for (const auto& q: leftChain) {
        q->print();
    }
    std::cout  << "RIghtCHAIN\n";
    for (const auto& q: rightChain) {
        q->print();
    }

    std::stack<Vertex*> S;
    S.push(V[0]);
    S.push(V[1]);
    for (int i = 2; i < V.size(); i++) {

        bool f =  ((leftChain.find(V[i]) != leftChain.end()) && (leftChain.find(S.top()) != leftChain.end()));
        bool f1 =  ((rightChain.find(V[i]) != rightChain.end()) && (rightChain.find(S.top()) != rightChain.end()));
        if (!f && !f1) {
            std::cout << "NO EQUAL CHAIN: \n";
            V[i]->print();
            S.top()->print();
            while (!S.empty()) {
                if (S.size() != 1) {

                    std::cout << "DIAG\n";
                    S.top()->print();
                    V[i]->print();
                    if (m[S.top()]->getEndPoint() != V[i] && m[V[i]]->getEndPoint() != S.top()) addDiag(m[S.top()], m[V[i]]);
                }
                S.pop();
            }
            S.push(V[i - 1]);
            S.push(V[i]);
        } else {
            std::cout << "ON EQUAL CHAIBN: \n";
            V[i]->print();
            S.top()->print();

            Vertex* last = S.top();
            S.pop();
            f = ((leftChain.find(last) != leftChain.end()) && (leftChain.find(V[i]) != leftChain.end()));
            while (IsValidDiagonal(V[i], S.top(), last, f) && !S.empty()) {
                last = S.top();
                S.pop();
                std::cout << "DIAn";
                last->print();
                V[i]->print();
                 if (m[last]->getEndPoint() != V[i] && m[V[i]]->getEndPoint() != last) addDiag(m[V[i]], m[last]);
                if (S.empty()) break;
            }

            S.push(last);
            S.push(V[i]);
        }
    }
   // return;
    std::cout << "LAST\n";
    if (!S.empty()) {
        S.pop();
        Vertex* lastVertex = V[V.size()-1];
        
        while (S.size() > 1) {
            S.top()->print();
            lastVertex->print();
            addDiag(m[S.top()], m[lastVertex]);
            S.pop();
        }
    }  

    std::cout << "LASTrrr\n";

}
