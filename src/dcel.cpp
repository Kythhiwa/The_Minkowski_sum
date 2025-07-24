#include "dcel.h"


Vertex::Vertex(double x, double y) : x(x), y(y), incidentEdge(nullptr) {}


bool Vertex::operator<(const Vertex& other) const {
    if (y != other.y) return y > other.y;
    return x < other.x;
}

bool Vertex::operator==(const Vertex& other) const {
    return x == other.x && y == other.y;
}

Vertex* HalfEdge::endPoint() const {
    if (next->origin == nullptr) {
        throw "The segment has an incorrect endpoint\n";
    }
    return next->origin;
}

bool HalfEdge::isHotizontal() const {
    return origin->y == endPoint()->y;
}

bool DCEL::Event::operator<(const DCEL::Event &other) const {
    return *point < *other.point;
}


bool DCEL::SegmentComparator::operator()(HalfEdge *a, HalfEdge *b) const {
    double xa = getXAtSweepY(a);
    double xb = getXAtSweepY(b);
}

bool DCEL::segmentIntersect(const Vertex *p1, const Vertex *p2, const Vertex *q1, const Vertex *q2, Vertex &intersection) {
    double r_x = p2->x - p1->x;
    double r_y = p2->y - p1->y;
    double s_x = q2->x - q1->x;
    double s_y = q2->y - q1->y;


    double r_cross_s = r_x * s_y - r_y * s_x;
    double qp_x = q1->x - p1->x;
    double qp_y = q1->y - p1->y;
    double qp_cross_r = qp_x * r_y - qp_y * r_x;
    
    if (r_cross_s == 0 && qp_cross_r == 0) {
            // Проверяем перекрытие проекций на оси
            double t0 = (qp_x * r_x + qp_y * r_y) / (r_x * r_x + r_y * r_y);
            double t1 = t0 + (s_x * r_x + s_y * r_y) / (r_x * r_x + r_y * r_y);
            
            if (std::max(t0, t1) < 0 || std::min(t0, t1) > 1) return false;
            
            // Находим точку пересечения
            double t = std::max(0.0, std::min(t0, t1));
            intersection.x = p1->x + t * r_x;
            intersection.y = p1->y + t * r_y;
            return true;
        }

        // Параллельные отрезки без пересечения
        if (r_cross_s == 0) return false;

        double t = (qp_x * s_y - qp_y * s_x) / r_cross_s;
        double u = (qp_x * r_y - qp_y * r_x) / r_cross_s;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            intersection.x = p1->x + t * r_x;
            intersection.y = p1->y + t * r_y;
            return true;
        }

        return false;
}

DCEL::DCEL(std::vector<std::pair<double,double>> p) {
    if (p.size() < 3) {
        throw std::invalid_argument("[ERROR]Point count < 3\n");
    }   
    for (const auto& [x, y] : p) {
        vertex.push_back(std::make_unique<Vertex>(x, y));
    }

    face.push_back(std::make_unique<Face>()); //outer
    face.push_back(std::make_unique<Face>()); //inner

    for (size_t i = 0; i < vertex.size(); ++i) {
        size_t next_i = (i + 1) % vertex.size();

        auto out = std::make_unique<HalfEdge>();
        auto in = std::make_unique<HalfEdge>();
        
        out->origin = vertex[i].get();
        in->origin = vertex[next_i].get();

        out->twin = in.get();
        in->twin = out.get();

        out->incidentFace = face[1].get();
        in->incidentFace = face[0].get();

        vertex[i]->incidentEdge = out.get();
        halfEdge.push_back(std::move(out));
        halfEdge.push_back(std::move(in));
    }

    for (size_t i = 0; i < halfEdge.size(); i += 2) {
        size_t next_i_in = (i + 2) % halfEdge.size();
        halfEdge[i]->next = halfEdge[next_i_in].get();
        halfEdge[next_i_in]->prev = halfEdge[i].get();

        size_t next_i_ou = (i + 3) % halfEdge.size();
        halfEdge[i + 1]->prev = halfEdge[next_i_ou].get();
        halfEdge[next_i_ou]->next = halfEdge[i + 1].get();
    }
    face[0]->outerComponent = halfEdge[1].get();
    face[1]->outerComponent = halfEdge[0].get();
}


void DCEL::print() const {
    std::cout << "=== DCEL Structure ===" << std::endl;
    
    std::cout << "\nVertices (" << vertex.size() << "):\n";
    std::unordered_map<Vertex*, size_t> vertex_ids;
    for (size_t i = 0; i < vertex.size(); ++i) {
        vertex_ids[vertex[i].get()] = i;
        std::cout << "  V" << i << ": (" << vertex[i]->x << ", " << vertex[i]->y << ")\n";
    }

    std::cout << "\nHalf-Edges (" << halfEdge.size() << "):\n";
    std::unordered_map<HalfEdge*, size_t> edge_ids;
    for (size_t i = 0; i < halfEdge.size(); ++i) {
        edge_ids[halfEdge[i].get()] = i;
        auto* e = halfEdge[i].get();
        std::cout << "  E" << i << ": V" << vertex_ids[e->origin] 
                  << " -> V" << vertex_ids[e->twin->origin] << "\n";
    }

    std::cout << "\nFace Traversal:\n";
    std::unordered_set<HalfEdge*> visited_edges;

    for (size_t fi = 0; fi < face.size(); ++fi) {
        if (!face[fi]->outerComponent) continue;

        std::cout << "Face " << fi << " (" 
                  << (fi == 0 ? "outer" : "inner") << "): ";

        HalfEdge* start = face[fi]->outerComponent;
        HalfEdge* current = start;
        
        do {
            std::cout << "V" << vertex_ids[current->origin] << " -> ";
            visited_edges.insert(current);
            current = current->next;
        } while (current != start && !visited_edges.count(current));

        std::cout << "[loop]\n";
    }
}

void DCEL::merge(DCEL& other) {
    for (auto &v : other.vertex) {
        vertex.push_back(std::move(v));
    }
    other.vertex.clear();
    for (auto &h : other.halfEdge) {
        halfEdge.push_back(std::move(h));
    }
    other.halfEdge.clear();
    for (auto &f : other.face) {
        face.push_back(std::move(f));
    }
    other.face.clear();
    

}

