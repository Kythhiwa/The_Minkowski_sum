#ifndef RENDER_H
#define RENDER_H

class Dcel;

class Render {
public:
    static void render(const Dcel& dcel);
    static void renderVertex(const Dcel& dcel, double r = 1.0, double g = 0.0, double b = 0.0, double size = 5.0);
    static void renderEdge(const Dcel& dcel, double r = 0.0, double g = 1.0, double b = 0.0, double size = 3.0);
};











#endif
