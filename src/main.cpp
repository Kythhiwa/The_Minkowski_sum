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
Dcel canvas;




void init(int a) {
    double x = 5.000000018, y = 5.000000019;
    std::cout << (x < y);
    std::vector<std::pair<double,double>> p11 ={
         
       
        {5, 0},
       {5, 2},
      {3, 3},
      {2, 2.5},
      {1, 3},
      {2, -1.1}


    };
    std::vector<std::pair<double,double>> p21 ={
        {-0.5, 1.2}, {-1.1, -0.12}, {1.1, -0.2}, {1, 1}, {0, 0.3}
    };
    std::vector<std::pair<double,double>> p22 ={
        
        {1, -1},
        {3, -1.1},
    {5, 0},
    {3, 2},
    {1, 3}
    };
 std::vector<std::pair<double,double>> p23 ={
      
     {3, -0.99999999},
     {5, 1.000000005e-08},
     {5, 2},
     {3, 3},
     {2, 2.5},
     {2, 0.5}
    };
   std::vector<std::pair<double,double>> p1 = {
        {-1, 1}, {-1, -1}, {1, -1}, {1, 1}, {0, 0.5} 
   };

    std::vector<std::pair<double,double>> p2 = {
      {2, 2}, {2, 0}, {4, 1} 
    };
    std::vector<std::pair<double, double>> p4 = {

      {2, 1.2},
{4, 2.2},
{4, 5.01},
{2, 6.01},
{1.8, 3.21},
{1.8, 1.21}      };
std::vector<std::pair<double, double>> p3 = {

{1.8, 1.01},
{2.3, 1.21},
{4.3, 2.21},
{2.3, 3.21},
{2, 3.2},
{1.8, 3.01}      };

       //p23 =  Random::getRandPolygon(8);

   
     p1 =  Random::getRandPolygon(9);
    p2 =  Random::getRandPolygon(5);

    Dcel d1(p1);
    Dcel d2(p2);
    Dcel d3(p3);
    Dcel d4(p4);
    std::vector<std::pair<double,double>> p5 = {
        {-1, 1}, {-1, -1}, {1, -1}, {1, 1} 
   };

    std::vector<std::pair<double,double>> p6 = {
      {-1, 0.5}, {-1, -0.5}, {1, 0} 
    };
p5 =  Random::getRandPolygon(8);

p6 =  Random::getRandPolygon(8);

    Dcel d5(p21);
    Dcel d6(p6);
   

   
        //return;
  //d1.setHoles(p21);
   //d1.setHoles(p23);
   // d1.triang();
    //d1.triang();
   std::cout << std::setprecision(10);
    Dcel res;
     Dcel::minkowskiSum(res, d1, d5);
    //canvas.copy(res);
    //d1.reflect();
    //Dcel t;
        //Dcel::merger(d5,d6, res);
    //t.copy(res);
    //res.clear();
    //Dcel::merger(t, d2, res);
    //t.copy(res);
   //canvas.copy(res);
   //canvas.copy(d3);
   d5.reflect();
   //canvas.copy(res);
   canvas.copy(res);
   //d2.triang();
  canvas.copy(d5);
   canvas.copy(d1);
   // t.~t();
    canvas.print();
}


void display() {
    glClearColor(0.08f, 0.09f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    Render::render(canvas);
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
