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
        {3, 2.5},
        {3, 0.5},
        {5, 0.5},
        {5, 2.5}
    };
    std::vector<std::pair<double,double>> p21 ={
        {-1, 1}, {-1, -1}, {1, -1}, {1, 1}
    };
   
    std::vector<std::pair<double,double>> p1 = {
        {2, 3},
        {2, 0},
        {6, 0},
    };

    std::vector<std::pair<double,double>> p2 = {
        {2, 0},
        {3, -5},
        {6, 0}

    };
    //p1 =  Random::getRandPolygon(7);
    //p2 =  Random::getRandPolygon(8);
    for (auto [x, y] : p1) {
        std::cout << "{"<< x << ", " << y << "}, \n";
    }
std::cout << "\n";
    for (auto [x, y] : p2) {
        std::cout << "{"<< x << ", " << y << "}, \n";
    }
std::cout << "\n";



    Dcel d1(p1);
    Dcel d2(p2);
    //d1.seHoles(p3);
    //
    auto r = d1.getInnerPoint(d1.getHalfEdge()[1]);
    std::cout << r.first << ":"<< r.second << "\n"; 
    //return ;
   // c.add(d1);
    Dcel::merge(c, d1, d2);
    //Dcel::merge(c, q, d3);
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
