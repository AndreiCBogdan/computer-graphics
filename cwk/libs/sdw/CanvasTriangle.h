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
    vector<CanvasPoint> topStarts;
    vector<CanvasPoint> topEnds;
    vector<CanvasPoint> bottomStarts;
    vector<CanvasPoint> bottomEnds;


    CanvasTriangle()
    {}

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
        //Need to prevent div by zero
        middleIntersect.y = middle.y;
        // float topMiddleGradient;
        // float topMiddleIntersection;
        // float middleBottomGradient;
        // float middleBottomIntersection;

        if ( bottom.x-top.x == 0 ){
            middleIntersect.x = top.x;
        } else {
            float topBottomGradient = (bottom.y-top.y)/(bottom.x-top.x);
            float topBottomIntersection = top.y-top.x*(bottom.y - top.y)/(bottom.x-top.x);
            if (topBottomGradient == 0){
                middleIntersect.x = (bottom.x + top.x)/2;
            } else {
                middleIntersect.x = (middleIntersect.y - topBottomIntersection) /topBottomGradient;
            }
        }
        if ( bottom.depth - top.depth == 0){
            middleIntersect.depth = top.depth;
        } else {
            float topBottomGradient = (bottom.y-top.y)/(bottom.depth-top.depth);
            float topBottomIntersection = top.y-top.depth*(bottom.y - top.y)/(bottom.depth-top.depth);
            if (topBottomGradient == 0){
                middleIntersect.depth = (top.depth+bottom.depth)/2;
            } else {
                middleIntersect.depth = (middleIntersect.y - topBottomIntersection) /topBottomGradient;
            }
        }
        int topHeight = middle.y - top.y;
        topStarts = interpolatePoints(top, middle, topHeight+1);
        // cout << topStarts.size() << endl;
        topEnds = interpolatePoints(top, middleIntersect, topHeight+1);
        int bottomHeight = bottom.y - middle.y;
        bottomStarts = interpolatePoints(middle, bottom, bottomHeight+1);
        bottomEnds = interpolatePoints(middleIntersect, bottom, bottomHeight+1);




        // topMiddleGradient = (middle.y-top.y)/(middle.x-top.x);
        // topMiddleIntersection = top.y-top.x*(middle.y - top.y)/(middle.x-top.x);
        // middleBottomGradient = (bottom.y-middle.y)/(bottom.x-middle.x);
        // middleBottomIntersection = middle.y-middle.x*(bottom.y - middle.y)/(bottom.x-middle.x);
    }

    vector<CanvasPoint> interpolatePoints(CanvasPoint start, CanvasPoint end, int steps){
        vector<CanvasPoint> points;
        for (int i=0; i<steps; i++){
            CanvasPoint p;
            if (steps == 1) steps = 2;
            p.x = start.x+((end.x-start.x)*i/(steps-1));
            p.y = start.y+((end.y-start.y)*i/(steps-1));
            p.depth = start.depth+((end.depth-start.depth)*i/(steps-1));
            // cout << "R " << v.x << endl;
            // cout << "G " << v.y << endl;
            // cout << "B " << v.z << endl;
            points.push_back(p);
        }
        return points;
    }



};



std::ostream& operator<<(std::ostream& os, const CanvasTriangle& triangle)
{
    os << triangle.vertices[0]  << triangle.vertices[1]  << triangle.vertices[2] << std::endl;
    return os;
}
