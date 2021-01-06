#include <glm/glm.hpp>
#include <iostream>


class RayTriangleIntersection
{
  public:
    glm::vec3 intersectionPoint;
    float distanceFromCamera;
    float distanceFromLight;
    ModelTriangle intersectedTriangle;
    Colour intersectionPointColour;

    RayTriangleIntersection()
    {
    }

    RayTriangleIntersection(glm::vec3 point, float distance, ModelTriangle triangle)
    {
        intersectionPoint = point;
        distanceFromCamera = distance;
        intersectedTriangle = triangle;
    }

    RayTriangleIntersection(glm::vec3 point, float distance, ModelTriangle triangle, Colour colour)
    {
        intersectionPoint = point;
        distanceFromCamera = distance;
        intersectedTriangle = triangle;
        intersectionPointColour = colour;
    }

    RayTriangleIntersection(glm::vec3 point, float distance, float lightDistance, ModelTriangle triangle)
    {
        intersectionPoint = point;
        distanceFromCamera = distance;
        distanceFromLight = lightDistance;
        intersectedTriangle = triangle;
    }
};

// std::ostream& operator<<(std::ostream& os, const RayTriangleIntersection& intersection)
// {
//     os << "Intersection is at " << intersection.intersectionPoint << " on triangle " << intersection.intersectedTriangle << " at a distance of " << intersection.distanceFromCamera << std::endl;
//     return os;
// }
