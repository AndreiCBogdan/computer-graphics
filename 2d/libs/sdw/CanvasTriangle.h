#include "CanvasPoint.h"
#include <iostream>
#include <vector>

class CanvasTriangle
{
  public:
    CanvasPoint vertices[3];
    Colour colour;
    CanvasPoint top;
    CanvasPoint middle;
    CanvasPoint middleIntersect;
    CanvasPoint bottom;
    float topBottomGradient;
    float topBottomIntersection;
    float topMiddleGradient;
    float topMiddleIntersection;
    float middleBottomGradient;
    float middleBottomIntersection;


    CanvasTriangle()
    {
    }

    CanvasTriangle(CanvasPoint v0, CanvasPoint v1, CanvasPoint v2)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = Colour(255,255,255);
    }

    CanvasTriangle(CanvasPoint v0, CanvasPoint v1, CanvasPoint v2, Colour c)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = c;
    }

    void calculateTriangleMeta(){
        CanvasPoint a = vertices[0];
        CanvasPoint b = vertices[1];
        CanvasPoint c = vertices[2];
        std::vector<float> ys = {a.y, b.y, c.y};
        float max = *max_element(ys.begin(), ys.end());
        if (a.y == max){
            bottom =a;
            if (b.y > c.y){
                middle = b;
                top = c;
            } else {
                middle = c;
                top = b;
            }
        } else if (b.y == max){
            bottom = b;
            if (a.y>c.y){
                middle = a;
                top = c;
            } else {
                middle = c;
                top = a;
            }
        } else {
            bottom = c;
            if (a.y>b.y){
                middle = a ;
                top = b;
            }
            else {
                top =a;
                middle = b;
            }
        }
        middleIntersect.y = middle.y;
        topBottomGradient = (bottom.y-top.y)/(bottom.x-top.x);
        topBottomIntersection = top.y-top.x*(bottom.y - top.y)/(bottom.x-top.x);
        middleIntersect.x = (middleIntersect.y - topBottomIntersection) /topBottomGradient;
        topMiddleGradient = (middle.y-top.y)/(middle.x-top.x);
        topMiddleIntersection = top.y-top.x*(middle.y - top.y)/(middle.x-top.x);
        middleBottomGradient = (bottom.y-middle.y)/(bottom.x-middle.x);
        middleBottomIntersection = middle.y-middle.x*(bottom.y - middle.y)/(bottom.x-middle.x);
    }

    



};

std::ostream& operator<<(std::ostream& os, const CanvasTriangle& triangle)
{
    os << triangle.vertices[0]  << triangle.vertices[1]  << triangle.vertices[2] << std::endl;
    return os;
}
