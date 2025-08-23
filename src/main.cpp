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
        {-0.5, 1.2}, {-1.1, -0.12}, {1.1, -0.2}, {1, 1}
    };
    std::vector<std::pair<double,double>> p22 ={
        {-0.5, 1.9}, {-2.1, -2}, {2.1, -2}, {2, 2.1}
    };
 std::vector<std::pair<double,double>> p23 ={
        {-0.5, -0.2}, {-1.1, -1}, {1.1, -1}, {1, -0.4}
    };
   std::vector<std::pair<double,double>> p1 = {
     {-6.00557, 1.87718},
{-6.64225, 1.34807},
{-6.7516, -0.01312},
{-5.58177, 0.80768},
{-4.11862, -1.87297},
{-2.65421, 1.173},
{-1.59741, 1.24971},
{0.23506, 1.3772},
{-2.64088, 2.66352},
{-1.75529, 5.18902},
{-1.47118, 6.29882},
{-3.55387, 5.35467},
{-4.2099, 5.74956},
{-5.1809, 4.41496},
{-7.4306, 5.13742},
{-6.47592, 3.669}   };

    std::vector<std::pair<double,double>> p2 = {
      {-3.8764, 1.68825},
{-2.32782, -0.02096},
{-0.57302, 0.84513},
{1.55101, -1.35237},
{2.97127, -0.32902},
{1.86292, 2.16004},
{4.41909, 1.29622},
{2.97387, 3.12549},
{2.35848, 3.62141},
{3.15896, 4.57045},
{3.26114, 6.08002},
{0.55724, 4.15846},
{-0.58207, 7.32685},
{-1.79997, 6.30074},
{-3.32542, 5.63532},
{-0.90788, 3.9878},
{-0.59359, 3.18017}  };      
    p1 =  Random::getRandPolygon(17);
    p2 =  Random::getRandPolygon(16);
    //p21 =  Random::getRandPolygon(8);



    Dcel d1(p1);
    Dcel d2(p2);
   for (const auto& v : d1.getVertex()) {
       std::cout << "{" << v->getX() << ", " << v->getY() << "},\n";
   }
   std::cout << "\n";

   for (const auto& v : d2.getVertex()) {
       std::cout << "{" << v->getX() << ", " << v->getY() << "},\n";
   }
  // d1.setHoles(p21);
   //d1.setHoles(p23);

//   Dcel::merge(c, d1);

   //return;
   //d1.triang();
   //d2.triang();
   Dcel::merge(c, d1);
    //c.triang();
    //c.add(d1);
   // c.add(d2);
    Dcel::merge(c, d2);
    //Dcel d4(p11);
    //d4.setHoles(p22);
    c.triang();

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
