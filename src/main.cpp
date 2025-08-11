#include <iostream> 
#include "dcel.h"
#include "vertex.h"
#include "halfEdge.h"
#include "sweepline.h"
#include "eventqueue.h" 
#include "eventI.h"
#include "config.h"
#include "GL/glut.h"
#include "render.h"
#include "genran.h"

Dcel c;


void init(int a) {
    std::vector<std::pair<double,double>> p1 ={
        {-2, 3}, {-2, 0}, {2, 0}
    };
    std::vector<std::pair<double,double>> p2 ={
        {-1, -2}, {1, -2}, {2, 3}
    };
    p1 =  Random::getRandPolygon(6);
    p2 =  Random::getRandPolygon(5);
    Dcel d1(p1, 1);
    Dcel d2(p2, 2);

    Dcel d3(Random::getRandPolygon(7), 3);
    Dcel::merge(c, d1, d2);
    Dcel::merge(c, d3, d3);
    c.print();

}
void display() {
    glClearColor(0.08f, 0.09f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    Render::render(c);
    glutSwapBuffers();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(1000, 1000);
    glutCreateWindow("DCEL Viewer");
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double a = 10;
    gluOrtho2D(-a, a, -a, a); // Подберите значения под ваш DCEL
    glMatrixMode(GL_MODELVIEW);
    init(a);
    glutDisplayFunc(display);
    

    glutMainLoop();
}
