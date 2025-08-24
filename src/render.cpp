#include "render.h"
#include "dcel.h"
#include "GL/gl.h"
#include "GL/glut.h"
#include "vertex.h"
#include "halfEdge.h"
#include "face.h"

void Render::render(const Dcel &dcel) {
    
    renderEdge(dcel, 0.65, 0.7, 0.8, 2);
    renderVertex(dcel, 0.98, 0.35, 0.38, 7);
}


void Render::renderColor(const Dcel& dcel, double r, double g, double b) {
    Dcel k;
    k.copy(dcel);
    k.triang();
    glColor3f(r, g, b);
    std::set<HalfEdge*> us;
    for (const auto& h : k.getHalfEdge()) {
        if (us.find(h) != us.end() || h->getIncidentFace()->getType() != Face::Type::INNER) continue;
        HalfEdge* cur = h;
        glBegin(GL_TRIANGLE_FAN);
        do {
            glVertex2d(cur->getOrigin()->getX(), cur->getOrigin()->getY());
            us.insert(cur);
            cur = cur->getNext();
        } while(cur != h);
        glEnd();
    }

}

void Render::renderVertex(const Dcel& dcel, double r, double g, double b, double size) {
    glColor3f(r,g,b);
    glPointSize(size);
    glBegin(GL_POINTS);
    for (const auto& v: dcel.getVertex()) {
        glVertex2d(v->getX(), v->getY());
    }
    glEnd();
    glColor3f(1.0, 0.9, 0.9); 
    glRasterPos2f(0.0f, 0.0f); 
    
    int vertexNumber = 0;
    for (const auto& v : dcel.getVertex()) {
        std::string numText = std::to_string(vertexNumber++);
        
        glRasterPos2f(v->getX() + 0.05f, v->getY() + 0.05f);
        
        for (char c : numText) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
        }
    }
}


void Render::renderEdge(const Dcel& dcel, double r, double g, double b, double size) {
    glColor3f(r,g,b);
    glLineWidth(size);
    glBegin(GL_LINES);
    for (const auto &h: dcel.getHalfEdge()) {
       // if (h->getIncidentFace()->getType() == Face::Type::INNER &&  h->getTwin()->getIncidentFace()->getType() == Face::Type::INNER) continue;
        glVertex2d(h->getOrigin()->getX(), h->getOrigin()->getY());
        glVertex2d(h->getEndPoint()->getX(), h->getEndPoint()->getY());
    }

    glEnd();
}


