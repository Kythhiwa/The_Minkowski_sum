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
        {-0.5, 1.2}, {-1.1, -0.5}, {1.1, -1}, {1, 1}
    };
    std::vector<std::pair<double,double>> p22 ={
        {-0.5, 1.9}, {-2.1, -2}, {2.1, -2}, {2, 2.1}
    };
   std::vector<std::pair<double,double>> p1 = {
       {1.71563, 4.54753},
{4.18194, 3.8618},
{6.96768, 2.89032},
{6.16384, 4.96291},
{7.46877, 6.78016},
{3.44629, 7.70362},
{2.65846, 6.49823}    };

    std::vector<std::pair<double,double>> p2 = {
        {2, -2},
        {6, -3},
        {8, 2},
        {6, -1},
        {4, 0.5},
        {2, 0}

    };      
    //p1 =  Random::getRandPolygon(7);
    p2 =  Random::getRandPolygon(8);
    //p21 =  Random::getRandPolygon(8);



    Dcel d1(p1);
    Dcel d2(p2);
   for (const auto& v : d1.getVertex()) {
       std::cout << "{" << v->getX() << ", " << v->getY() << "},\n";
   }
   std::cout << "\n";

   for (const auto& v : d2.getVertex()) {
     //  std::cout << "{" << v->getX() << ", " << v->getY() << "},\n";
   }
  // d1.setHoles(p21);
   d1.triang();
   //Dcel::merge(c, d1);
   c.add(d1);
   // Dcel::merge(c, d2);
    //Dcel d4(p11);
    //d4.setHoles(p22);
    c.print();
   // std::cout << "\n";
    std::cout <<"SOSSSSSSS\n";
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
