#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>

using namespace std;
using namespace glm;

#define WIDTH 320
#define HEIGHT 240

void draw();
void update();
void handleEvent(SDL_Event event);
vector<float> interpolate(float start, float end, int steps);
void greyScaleX();
void rainbow();
uint32_t vec3ToUint(vec3 v);
vec3 uintToVec3(uint32_t u);
vector<vec3> interpolate3(vec3 start, vec3 end, int step);

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);

int main(int argc, char* argv[])
{
    interpolate(2.2, 8.5, 7);
    SDL_Event event;
    while(true)
    {
        // We MUST poll for events - otherwise the window will freeze !
        if(window.pollForInputEvents(&event)) handleEvent(event);
        update();
        draw();
        // Need to render the frame at the end, or nothing actually gets shown on the screen !
        window.renderFrame();
    }
}

void draw()
{
    window.clearPixels();
    rainbow();
}

void redNoise(){
    for(int y=0; y<window.height ;y++) {
        for(int x=0; x<window.width ;x++) {
            float red = rand() % 255;
            float green = 0.0;
            float blue = 0.0;
            uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
            window.setPixelColour(x, y, colour);
        }
    }
}

void update()
{
    // Function for performing animation (shifting artifacts or moving the camera)
}

void greyScaleX(){
    vector<float> greyValues = interpolate(255, 0, window.width);
    for(int x=0; x<window.width ;x++) {
        float red = greyValues.at(x);
        float green = greyValues.at(x);
        float blue = greyValues.at(x);
        //Can simplify
        uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
        for(int y=0; y<window.height ;y++) {
            window.setPixelColour(x, y, colour);
        }
    }
}

vec3 uintToVec3(uint32_t u){
    vec3 v;
    // (255<<24) + (int(v.x)<<16) + (int(v.y)<<8) + int(v.z);
    //Red
    v.x = int((u-(255<<24))     >> 16);
    //Green
    v.y = int( (u-(255<<24)-(int(v.x)<<16)           ) >> 8 );
    //Blue
    v.z = int(     u-(255<<24)-(int(v.x)<<16)-(int(v.y)<<8)        );
    // cout << "R " << v.x << endl;
    // cout << "G " << v.y << endl;
    // cout << "B " << v.z << endl;
    return v;
}

uint32_t vec3ToUint(vec3 v){
    // cout << "R " << v.x << endl;
    // cout << "G " << v.y << endl;
    // cout << "B " << v.z << endl;
    uint32_t u = (255<<24) + (int(v.x)<<16) + (int(v.y)<<8) + int(v.z);
    return u;
}

void rainbow(){
    //Left column
    vec3 topleft = vec3(255.0, 0.0, 0.0);
    vec3 bottomleft = vec3(255, 255.0, 0);
    vector<vec3> leftCol = interpolate3(topleft, bottomleft, window.height);
    for (int y=0; y<window.height; y++){
        window.setPixelColour(0, y, vec3ToUint(leftCol.at(y)));
    }
    //Right column
    vec3 topright = vec3(0.0, 0.0, 255.0);
    vec3 bottomright = vec3(0.0, 255.0, 0.0);
    vector<vec3> rightCol = interpolate3(topright, bottomright, window.height);
    for (int y=0; y<window.height; y++){
        window.setPixelColour(window.width-1, y, vec3ToUint(rightCol.at(y)));
    }
    // Everything inbetween
    for(int y=0; y<window.height ;y++) {
        vec3 left = uintToVec3(window.getPixelColour(0, y));
        vec3 right = uintToVec3(window.getPixelColour(window.width-1, y));
        vector<vec3> row = interpolate3(left, right, window.width-2);
        for (int x=1; x<window.width-2; x++){
            window.setPixelColour(x, y, vec3ToUint(row.at(x)));
        }
    }
}

vector<float> interpolate(float start, float end, int steps){
    vector<float> v = {};
    for (int i=0; i<steps; i++){
        float num = start+((end-start)*i/(steps-1));
        // cout << num << endl;
        v.push_back(num);
    }
    return v;
}

vector<vec3> interpolate3(vec3 start, vec3 end, int steps){
    vector<vec3> vectors;
    for (int i=0; i<steps; i++){
        vec3 v;
        v.x = start.x+((end.x-start.x)*i/(steps-1));
        v.y = start.y+((end.y-start.y)*i/(steps-1));
        v.z = start.z+((end.z-start.z)*i/(steps-1));
        // cout << "R " << v.x << endl;
        // cout << "G " << v.y << endl;
        // cout << "B " << v.z << endl;
        vectors.push_back(v);
    }
    return vectors;
}

void handleEvent(SDL_Event event)
{
    if(event.type == SDL_KEYDOWN) {
        if(event.key.keysym.sym == SDLK_LEFT) cout << "LEFT" << endl;
        else if(event.key.keysym.sym == SDLK_RIGHT) cout << "RIGHT" << endl;
        else if(event.key.keysym.sym == SDLK_UP) cout << "UP" << endl;
        else if(event.key.keysym.sym == SDLK_DOWN) cout << "DOWN" << endl;
    }
    else if(event.type == SDL_MOUSEBUTTONDOWN) cout << "MOUSE CLICKED" << endl;
}
