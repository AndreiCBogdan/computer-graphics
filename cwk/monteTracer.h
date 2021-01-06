// followed https://www.scratchapixel.com/lessons/3d-basic-rendering/global-illumination-path-tracing
#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <RayTriangleIntersection.h>
#include <ModelSphere.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <glm/gtc/matrix_access.hpp>
#include <time.h>
#include <random>
#include <DrawingWindow.h>

using namespace std;
using namespace glm;

#define WIDTH 1000
#define HEIGHT 1000


bool solutionOnTriangle(vec3 i);
vec3 getNormal(ModelTriangle triangle);
void createCoordinateSystem(const vec3 &N, vec3 &Nt, vec3 &Nb);
vec3 uniformSampleHemisphere(const float &r1, const float &r2);
void monteTracer();
bool findRoots(float &a, float &b, float &c, float &t0, float &t1);
vec3 trace(vec3 &origin, vec3 &direction, int depth, float &absorbFactor, bool insideObject, vector<ModelTriangle> triangles);
void initialiseSphere();
vec3 colourAdjustment(vec3 colour);
void scaleTriangles(vector<ModelTriangle>&triangles);

struct Intersection {
    vec3 position;
    float distance;
    int triangleIndex;
    int sphereIndex;
};

#define DIFFUSE      0
#define TRANSPARENT  1
#define RUSSIAN      0.60f
#define SAMPLES      1
#define MIN_DEPTH    3
#define FACTOR       1 //determines how much to decrease the samples by for the next iteration
#define PI           3.14159265

vec3 lightPosition(-0.2334011,4.5,-3.043968);
mat3 cameraOrientation(vec3(1.0,0.0,0.0),vec3(0.0,1.0,0.0),vec3(0.0,0.0,1.0));
vec3 cameraPosition(-0.25,2.5,8);
float focalLength = 550;
int iterations;
int max_depth = 0;

default_random_engine engine(std::random_device{}());
uniform_real_distribution<float> distribution(0.0f, 1.0f);
vec3 image[WIDTH][HEIGHT];
vector<ModelSphere> spheres;



void initialiseSphere(){
    // /quicker than a million triangles + produces a nicer sphere
    //should be sitting on top of red-box            *centre*                *radius*     *colour*      *emission*    *reflect* *ior*      *beer-lambert*
    ModelSphere sphere = ModelSphere(vec3(0.68599, 2.12765,-1.92991), 3.49318f, Colour(187, 39,187), 190.0f, DIFFUSE, 0.0f, 1.000277f,vec3(0.0f,0.0f, 0.0f));
    spheres.push_back(sphere);
}


bool triangleIntersection(vec3 origin, vec3 direction, ModelTriangle triangle, Intersection &intersection){
    vec3 e0 = vec3(triangle.vertices[1] - triangle.vertices[0]);
    vec3 e1 = vec3(triangle.vertices[2] - triangle.vertices[0]);
    vec3 SPVector = vec3(origin-triangle.vertices[0]);
    mat3 DEMatrix(-direction, e0, e1);
    vec3 possibleSolution = glm::inverse(DEMatrix) * SPVector;
    intersection.distance = std::numeric_limits<float>::max();
    //  && possibleSolution.x > 0.0001f
    if (solutionOnTriangle(possibleSolution)){
        if (possibleSolution.x < intersection.distance){
            intersection.position = triangle.vertices[0] + possibleSolution.y*e0 + possibleSolution.z*e1;
            intersection.distance = length(origin - intersection.position);
            return true;
        }
    }
    return false;
}

bool sphereIntersection(vec3 origin, vec3 direction, ModelSphere sphere, Intersection &intersection){
    float t0, t1;
    vec3  L = origin - sphere.centre;
    float a = dot(direction, direction);
    float b = 2 * dot(direction, L);
    float c = dot(L, L) - (sphere.radius * sphere.radius);
    if (!findRoots(a, b, c, t0, t1)) return false;

    if (t0 > t1) std::swap(t0, t1);
    if (t0 < 0) {
        t0 = t1; // if t0 is negative, let's use t1 instead
        if (t0 < 0) return false; // both t0 and t1 are negative
    }

    intersection.distance = t0;
    intersection.position = origin + direction * t0;
    return true;

}
// //CHANGE VARIABLE NAMES
bool findRoots(float &a, float &b, float &c, float &t0, float &t1){
    float d = b * b - 4 * a * c;
    if (d < 0) {
        return false;
    }
    else if (d == 0) {
        t0 = t1 = - 0.5 * b / a;
    }
    else {
        float q = (b > 0) ? -0.5 * (b + sqrt(d)) : -0.5 * (b - sqrt(d));
        t0 = q / a;
        t1 = c / q;
    }
    if (t0 > t1) swap(t0, t1);

    return true;
}

bool getClosestIntersection(vec3 origin, vec3 direction, const vector<ModelTriangle> &triangles, const vector<ModelSphere> &spheres, Intersection &closestIntersection){
    bool found = false;
    float minDistance = std::numeric_limits<float>::max();
    Intersection intersection;
    for( int i = 0; i < triangles.size(); i++ ) {
        if (triangleIntersection(origin, direction, triangles.at(i), intersection)) {
            if (intersection.distance < minDistance) {
                found = true;
                closestIntersection = intersection;
                closestIntersection.triangleIndex =  i;
                closestIntersection.sphereIndex   = -99;
                minDistance = intersection.distance;
            }
        }
    }

    for( int i = 0; i < spheres.size(); i++ ) {
        if (sphereIntersection(origin, direction, spheres.at(i), intersection)) {
            if (intersection.distance < minDistance) {
                found = true;
                closestIntersection = intersection;
                closestIntersection.triangleIndex = -99;
                closestIntersection.sphereIndex   =  i;
                minDistance = intersection.distance;
            }
        }
    }
    return found;
}

vec3 trace(vec3 &origin, vec3 &direction, int depth, float &absorbFactor, bool insideObject, vector<ModelTriangle> cornell){
    if(depth > max_depth) max_depth = depth;

    //Russian roulette
    float prob = 1.0f;
    if (depth > MIN_DEPTH) {
        prob = RUSSIAN;
        float r = distribution(engine);
        if (r < 1.0f - prob) {
            return vec3(0,0,0);
        }
    }

    Intersection closestIntersection;
    if(!getClosestIntersection(origin, direction, cornell, spheres, closestIntersection)){
        return vec3(0,0,0);
    }

    vec3 normal;
    int   type=0;
    float ior;
    vec3  sigma; // beer lambert
    float emission;
    float reflectivity;
    vec3  surfaceColor;
    vec3  hitColour;
    float random;

    if(closestIntersection.triangleIndex > -1){
        ModelTriangle triangle = cornell.at(closestIntersection.triangleIndex);
        normal                 = getNormal(triangle);
        triangle.getSurfaceProperties(type, ior, sigma, emission, reflectivity, surfaceColor);

    }
    else if (closestIntersection.sphereIndex > -1) {
        ModelSphere sphere = spheres.at(closestIntersection.sphereIndex);
        normal       = normalize(closestIntersection.position - sphere.centre);
        sphere.getSurfaceProperties(type, ior,sigma, emission, reflectivity, surfaceColor);

    }
    else{
        cout << "this shouldnt happen boi" << endl;
    }

    if(type == TRANSPARENT){
        float cosI = glm::dot(normalize(direction),normal);
        float n1 = 1, n2 = ior;
        float eta = n1/n2;
        bool entering = true;
        if(cosI > 0){
            cosI = -cosI;
            normal = -normal;
            swap(n1, n2);
            entering = false;
        }
        float r0    = pow((n1-n2)/(n1-n2), 2);
        float cos0  = -dot(normal, direction);
        float rprob = r0 + (1.0f - r0) * pow(1 - cos0, 5);
        float k     =  1.0f - eta * eta * (1.0f -pow(cos0,2));
        random      = distribution(engine);
        //refraction
        if(k >= 0 && random > rprob){

            vec3 refractedRay = glm::refract(direction, normal, eta);
            //soft shadows
            vec3 refractedPoint = closestIntersection.position - normal*0.0001f;
            hitColour = trace(refractedPoint,refractedRay, depth+1, absorbFactor, insideObject, cornell);
            //inside
            if(insideObject){
                absorbFactor += length(origin - refractedPoint);
            }
            else{
                //Beer lamber law
                vec3 absorbedColour = exp((-sigma)* absorbFactor);
                hitColour *= absorbedColour;
                //reset
                absorbFactor = 0.0f;
            }
            //Reflection
        } else{
            vec3 reflectedRay = glm::reflect(direction, normal);
            vec3 reflectedPoint = closestIntersection.position+normal*0.0001f;
            hitColour = trace(reflectedPoint, reflectedRay, depth+1, absorbFactor, !insideObject, cornell);
            if(insideObject){
                absorbFactor += length(origin- reflectedPoint);
            }
        }
        hitColour = hitColour/prob;

    }
    else if(type == DIFFUSE){
        vec3 N, Nb, Nt;
        N = normal;
        createCoordinateSystem(N, Nt, Nb);
        //direct light
        vec3 emittedLight = emission * vec3(1,1,1);
        vec3 hitPosition = closestIntersection.position + normal * 0.00001f;
        float fresnel = 0.0f;
        if(reflectivity != 0){
            //Shclicks
            float n1 = 1.002f; // air
            float n2 = ior;
            float r0    = pow((n1-n2)/(n1-n2), 2);
            float cos0  = -dot(normal, direction);
            float rprob = r0 + (1.0f - r0) * pow(1 - cos0, 5);
            fresnel = rprob * reflectivity;
        }
        random = distribution(engine);

        vec3 indirectLight = vec3(0,0,0);
        if(random >= fresnel){
            //for now till we find out what drop factor does
            int samples = std::max( (int) (SAMPLES / pow(FACTOR, depth)), 1 );
            float pdf = 1 / (2* PI);
            for(int i = 0; i < samples; i++ ){
                float r1 = distribution(engine);
                float r2 = distribution(engine);
                vec3 sample = uniformSampleHemisphere(r1, r2);
                // direction
                vec3 sampleWorld = vec3(
                    sample.x * Nb.x + sample.y * N.x + sample.z  *Nt.x,
                    sample.x * Nb.y + sample.y * N.y + sample.z  *Nt.y,
                    sample.x * Nb.z + sample.y * N.z + sample.z  *Nt.z
                );
                // printVec3(sampleWorld);
                indirectLight += r1 * trace(hitPosition, sampleWorld, depth+1, absorbFactor, insideObject, cornell);
                // printVec3(indirectLight);
            }
            indirectLight /= samples * pdf * prob;
            // SURFACE COLOUR IS A COLOUR
            hitColour = (indirectLight + emittedLight) * surfaceColor / (float) PI;
        }
        else{
            //specular
            vec3 reflectedRay =  glm::reflect(direction, normal);
            indirectLight = trace(hitPosition, reflectedRay, depth+1, absorbFactor, insideObject, cornell);
            indirectLight = indirectLight/prob;
            hitColour = indirectLight;
        }
    }
    return hitColour;
}

void createCoordinateSystem(const glm::vec3 &N, glm::vec3 &Nt, glm::vec3 &Nb){
    if (std::abs(N.x) > std::abs(N.y)) {
        Nt = vec3(N.z, 0, -N.x)/ sqrtf(N.x * N.x + N.z * N.z);
    } else {
        Nt = vec3(0, -N.z, N.y)/ sqrtf(N.y * N.y + N.z * N.z);
    }
    Nb = cross(N, Nt);
}

vec3 uniformSampleHemisphere(const float &r1, const float &r2){
    float sinTheta = sqrtf(1 - r1 * r1);
    float phi = 2 * PI * r2;
    float x = sinTheta * cosf(phi);
    float z = sinTheta * sinf(phi);
    return vec3(x, r1, z);
}

void monteTracer(vector<ModelTriangle> triangles, DrawingWindow window){
    iterations +=1;

    #pragma omp parallel for
    for(int y = 0; y < HEIGHT; y++ ){
        for(int x = 0; x < WIDTH; x++){

            float yOffset = distribution(engine) - 0.5f;
            float xOffset = distribution(engine) - 0.5f;
            vec3 ray = cameraOrientation * vec3(WIDTH/2.0f -x + xOffset, y-(HEIGHT/2.0f) + yOffset, focalLength);
            float absorbHolder;
            vec3 colour = trace(cameraPosition, ray, 0, absorbHolder, false, triangles);
            image[x][y] += colour;
            //convert vec3 to colour
            vec3 adjusted = colourAdjustment(image[x][y]/(float) iterations);
            Colour finalColour  = Colour(adjusted.x, adjusted.y, adjusted.z);
            window.setPixelColour(x, y, finalColour.getPacked());
        }
    }
}
vec3 colourAdjustment(vec3 colour){
    //filmic adjustment
    float A = 2.51f;
    float B = 0.03f;
    float C = 2.43f;
    float D = 0.59f;
    float E = 0.14f;
    float F = 0.30;
    vec3 tonedColour;
    tonedColour.x = ((colour.x*(A*colour.x+C*B)+D*E)/(colour.x*(A*colour.x+B)+D*F))-E/F;
    tonedColour.y = ((colour.y*(A*colour.y+C*B)+D*E)/(colour.y*(A*colour.y+B)+D*F))-E/F;
    tonedColour.z = ((colour.z*(A*colour.z+C*B)+D*E)/(colour.z*(A*colour.z+B)+D*F))-E/F;
    return tonedColour;
}
//
void scaleTriangles( vector<ModelTriangle>& triangles ) {
    float MAX  = 0.0f;
    float yMax, zMin;
    yMax = numeric_limits<float>::min();
    zMin = numeric_limits<float>::max();
    float scalefactor = 1.1f;
    for( int i = 0; i < triangles.size(); ++i ) {
        for( int j = 0; j < 3; ++j){
            if (abs(triangles[i].vertices[j].x) > MAX){
                MAX = abs(triangles[i].vertices[j].x);
            }
            else if(abs(triangles[i].vertices[j].y) > MAX){
                MAX = abs(triangles[i].vertices[j].y);
            }
            else if(abs(triangles[i].vertices[j].z) > MAX){
                MAX = abs(triangles[i].vertices[j].z);
            }

            if(triangles[i].vertices[j].y > yMax){
                yMax = triangles[i].vertices[j].y;
            }

            if(triangles[i].vertices[j].z < zMin){
                zMin = triangles[i].vertices[j].z;
            }
        }
    }
    yMax *= 1 / (scalefactor * MAX);
    zMin *= 1 / (scalefactor * MAX);

    for( int i = 0; i < triangles.size(); ++i ) {
        for( int j = 0; j < 3; ++j ){
            triangles[i].vertices[j] *= 1 / (scalefactor * MAX);
            triangles[i].vertices[j].z += (-0.5f - zMin);
            triangles[i].vertices[j].y += (1.0f - yMax);
        }
	}
}

bool solutionOnTriangle(vec3 i){
    return (0.0<=i.y && i.y<=1.0 && 0.0<=i.z && i.z<=1.0 && (i.y+i.z<=1.0));
}

vec3 getNormal(ModelTriangle triangle){
    vec3 e0 = vec3(triangle.vertices[1] - triangle.vertices[0]);
    vec3 e1 = vec3(triangle.vertices[2] - triangle.vertices[0]);
    vec3 normal = normalize(glm::cross(e0,e1));
    return normal;
}
