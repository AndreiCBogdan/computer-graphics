#include <string>
#include <glm/glm.hpp>


class ModelSphere{
    public:
        glm::vec3 centre;
        float radius;
        Colour colour;
        float emission;
        int transparency;
        float reflectivity;
        float IOR;
        glm::vec3 sigma;

        ModelSphere(){

        }

        ModelSphere(glm::vec3 c, float r, Colour col){
            centre = c;
            radius = r;
            colour = col;
        }

        ModelSphere(glm::vec3 c, float r, Colour col, float e, int tran, float refl, float ior, glm::vec3 sig){
            centre = c;
            radius = r;
            colour = col;
            emission = e;
            transparency = tran;
            reflectivity = refl;
            IOR = ior;
            sigma = sig;
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


std::ostream& operator<<(std::ostream& os, const ModelSphere& sphere)
{
    os << "(" << sphere.centre.x << ", " << sphere.centre.y << ", " << sphere.centre.z << ")" << std::endl;
    os << "radius: " << sphere.radius << std::endl;
    os << "Colour: " << sphere.colour << std::endl;
    os << "emission " << sphere.emission << std::endl;
    os << "transparency " << sphere.transparency << std::endl;
    os << "reflectivity  " << sphere.reflectivity << std::endl;
    os << "IOR " << sphere.IOR << std::endl;
    os << std::endl;
    return os;
}
