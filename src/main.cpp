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
         {-5, 5}, {-5, -5}, {5, -5}, {5, 5}

    };
    std::vector<std::pair<double,double>> p21 ={
        {-1, 1}, {-1, -1}, {1, -1}, {1, 1}
    };
    std::vector<std::pair<double,double>> p22 ={
        {-2, 2}, {-2, -2}, {2, -4}, {2, 2}
    };
   std::vector<std::pair<double,double>> p1 = {
        {2, 3},
        {-1, 0},
        {7, -5},
    };

    std::vector<std::pair<double,double>> p2 = {
        {1, 0},
        {3, -5},
        {6, 0.1}

    };      
    //p1 =  Random::getRandPolygon(7);
    //p2 =  Random::getRandPolygon(8);
    



    Dcel d1(p1);
    Dcel d2(p2);

  
    Dcel::merge(c, d1);
   Dcel::merge(c, d2);
    Dcel d3(p21);
    Dcel d4(p11);
    d4.setHoles(p22);
   Dcel::merge(c, d3);
   Dcel::merge(c, d4);
    c.print();
    std::cout << "\n";
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
