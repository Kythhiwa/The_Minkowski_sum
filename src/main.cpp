#include <iostream> 
#include "dcel.h"
#include "vertex.h"
#include "halfEdge.h"
#include "sweepline.h"
#include "eventqueue.h" 
#include "eventI.h"
#include "config.h"

int main() {
    std::vector<std::pair<double,double>> p1 ={
        {-2, 3}, {-2, 0}, {2, 0}
    };
    std::vector<std::pair<double,double>> p2 ={
        {-1, -2}, {1, -2}, {2, 3}
    };
    Dcel d1(p1, 1);
    Dcel d2(p2, 2);

    Dcel c;
    Dcel::merge(c, d1, d2);
   // c.test(d1, d2);
    c.print();
    c.dfs();
}
