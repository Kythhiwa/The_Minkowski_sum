#include "render.h"
#include "dcel.h"
#include "GL/gl.h"
#include "vertex.h"
#include "halfEdge.h"

void Render::render(const Dcel &dcel) {
    renderEdge(dcel, 0.65, 0.7, 0.8, 2);
    renderVertex(dcel, 0.98, 0.35, 0.38, 7);
}


void Render::renderVertex(const Dcel& dcel, double r, double g, double b, double size) {
    glColor3f(r,g,b);
    glPointSize(size);
    glBegin(GL_POINTS);
    for (const auto& v: dcel.getVertex()) {
        glVertex2d(v->getX(), v->getY());
    }
    glEnd();
}


void Render::renderEdge(const Dcel& dcel, double r, double g, double b, double size) {
    glColor3f(r,g,b);
    glLineWidth(size);
    glBegin(GL_LINES);
    for (const auto &h: dcel.getHalfEdge()) {
        glVertex2d(h->getOrigin()->getX(), h->getOrigin()->getY());
        glVertex2d(h->getEndPoint()->getX(), h->getEndPoint()->getY());
    }

    glEnd();
}

