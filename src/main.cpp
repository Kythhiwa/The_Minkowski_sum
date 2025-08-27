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



void test1(std::vector<std::pair<double,double>> &p1, std::vector<std::pair<double,double>> &p2) {
    p1 = {
        {-0.3, 0},{0.3, 0},{0.15, 0.3},{-0.15, 0.3}
    };
    p2 = {
        {2,2},{10,2},{9.1, 7},{8, 4},{7, 7},{6, 4},{5, 7},{4, 4},{3, 7}
    };    
}

void test2(std::vector<std::pair<double,double>> &p1, std::vector<std::pair<double,double>> &p2) {
    p1 = {
        {-1, 1}, {-1, -1}, {1, -1}, {1, 1}, {0, 0.5} 
    };
    p2 = {
      {2, 3}, {2.3, 1}, {4, 2} 
    };
}
void testR(std::vector<std::pair<double,double>> &p1, std::vector<std::pair<double,double>> &p2) {
    p1 = {
        {-1, 1}, {-1, -1}, {1, -1}, {1, 1}, {0, 0.5} 
    };
    p2 =  Random::getRandPolygon(8);
}

void test3(std::vector<std::pair<double,double>> &p1, std::vector<std::pair<double,double>> &p2) {
    p1 = {
        {-0.3, -0.3}, {0.3, -0.3}, {-0.3, 2} 
    };
    p2 = {
      {2.5, 2.5}, {6, 2.5}, {6, 6}, {4, 6}, {2.5, 4} 
    };
}
void test4(std::vector<std::pair<double,double>> &p1, std::vector<std::pair<double,double>> &p2) {
    p1 = {
        {0,0}, {2, 3}, {0, 2}, {-1, 4}, {-2, 2}
    };
    p2 = {
      {2.5, 2.5}, {6, 2.5}, {6, 6}, {4, 6}, {2.5, 4} 
    };
}
void test5(std::vector<std::pair<double,double>> &p1, std::vector<std::pair<double,double>> &p2) {
    p1 = {
        {0,0}, {2, 3}, {0, 2}, {-1, 4}, {-2, 2}
    };
    p2 = {
        {6,6}, {5, 4}, {6, 2}, {8, 6}, {7, 7}, {4, 6} 
    };
}

void solve() {
    std::vector<std::pair<double,double>> p1;
    std::vector<std::pair<double,double>> p2;
    
    test5(p1, p2);

    Dcel d1(p1);
    Dcel d2(p2);
    Dcel res;
    Dcel::minkowskiSum(res, d1, d2);
    d1.reflect();
    canvas.copy(res);
    canvas.copy(d1);
    canvas.copy(d2);

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
    glutCreateWindow("Msum");
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double a = 12;
    gluOrtho2D(-a, a, -a, a); 
    glMatrixMode(GL_MODELVIEW);
    //init(a);
    solve();
    glutDisplayFunc(display);
    

    glutMainLoop();
}
