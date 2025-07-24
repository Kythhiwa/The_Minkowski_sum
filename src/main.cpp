#include <iostream>
#include "dcel.h"
#include <map>
#include <GL/glut.h>
#include <vector>




int main() {

    std::vector<std::pair<double, double> >points = {
        {-2, 3}, {-2, 0}, {2, 0}
    };
    std::vector<std::pair<double, double> >points1 = {
        {3, 3}, {-1, -2}, {3, -2}
    };

    DCEL a(points);
    DCEL b(points1);
    a.merge(b);
    a.print();
}
