#include <glm/glm.hpp>
#include "Colour.h"
#include <string>

class ModelTriangle
{
  public:
    glm::vec3 vertices[3];
    Colour colour;
    std::string objectName;
    glm::vec3 normals[3];
    glm::vec2 texture[3];
    float emission;
    int transparency;
    float reflectivity;
    float IOR;
    glm::vec3 sigma;


    ModelTriangle()
    {
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, Colour trigColour)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = trigColour;
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, Colour trigColour, std::string o)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = trigColour;
      objectName = o;
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, Colour trigColour, glm::vec3 n0, glm::vec3 n1, glm::vec3 n2){
        vertices[0] = v0;
        vertices[1] = v1;
        vertices[2] = v2;
        colour = trigColour;
        normals[0] = n0;
        normals[1] = n1;
        normals[2] = n2;
    }
    void getSurfaceProperties(int &t, float &ior, glm::vec3 &s, float &e, float &r, glm::vec3 &c){
        t   = transparency;
        ior = IOR;
        s   = sigma;
        e   = emission;
        r   = reflectivity;
        c   = glm::vec3(colour.red, colour.green, colour.blue);
    }
};

std::ostream& operator<<(std::ostream& os, const ModelTriangle& triangle)
{
    os << "(" << triangle.vertices[0].x << ", " << triangle.vertices[0].y << ", " << triangle.vertices[0].z << ")" << std::endl;
    os << "(" << triangle.vertices[1].x << ", " << triangle.vertices[1].y << ", " << triangle.vertices[1].z << ")" << std::endl;
    os << "(" << triangle.vertices[2].x << ", " << triangle.vertices[2].y << ", " << triangle.vertices[2].z << ")" << std::endl;
    os << "Colour " << triangle.colour << std::endl;
    os << "Object " << triangle.objectName << std::endl;
    os << "emission " << triangle.emission << std::endl;
    os << "transparency " << triangle.transparency << std::endl;
    os << "reflectivity  " << triangle.reflectivity << std::endl;
    os << "IOR " << triangle.IOR << std::endl;
    os << std::endl;
    return os;
}
