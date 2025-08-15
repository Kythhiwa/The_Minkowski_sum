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
#include "face.h"
#include "iomanip"
Dcel c;


void init(int a) {
    std::vector<std::pair<double,double>> p11 ={
        {-2, 2}, {-2, -2}, {2, -2}, {2, 2}
    };
    std::vector<std::pair<double,double>> p21 ={
        {-1, 1}, {-1, -1}, {1, -1}, {1, 1}
    };
   // Треугольник 1 (CCW)
std::vector<std::pair<double,double>> p1 = {
    {-2, 0},  // ← левая вершина
    {2, 0},   // → правая вершина
    {0, 3}    // ↑ верхняя вершина
};

// Треугольник 2 (CCW) - пересекает первый
std::vector<std::pair<double,double>> p2 = {
    {0, -1},  // ↓ нижняя вершина
    {2, 2},  // ↖ верх-лево
    {-2, 2}    // ↗ верх-право
};
// Пересечения: (0,1.5) и (0,0)
    Dcel g(p11, 4);
    Dcel g1(p21, 1231);
    
    g.setHoles(p21);
    std::vector<std::pair<double, double>> p3 = {
        {-0.5, 0.3},  {-0.5, 0.1},  {0.8, 0.1},  {0.8, 0.3}
    };
    
    //p1 =  Random::getRandPolygon(6);
    Dcel f(p3, 5);
    //c.add(g);
    //Dcel::merge(c, g, f);
    
    //c.print();

    
    p1 =  Random::getRandPolygon(4);
    p2 =  Random::getRandPolygon(5);
    Dcel d1(p1, 1);
    Dcel d2(p2, 2);
    Dcel d3(Random::getRandPolygon(7), 3);
    //d1.setHoles(p3);
    Dcel::merge(c, d1, d2);
    //Dcel::merge(c, d3, d3);
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
    gluOrtho2D(-a, a, -a, a); 
    glMatrixMode(GL_MODELVIEW);
    init(a);
    glutDisplayFunc(display);
    

    glutMainLoop();
}
