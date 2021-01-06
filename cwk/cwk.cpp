#include "monteTracer.h"
#include "Image.h"
#include <Utils.h>



struct BBOX{
    vec3 min;
    vec3 max;
    string objectName;
};


void draw();
void update();
void handleEvent(SDL_Event event);
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour);
void drawStrokeTriangle(CanvasTriangle t);
Image loadImage(string filename);
void savePPM(int index);
vector<ModelTriangle> loadObj(string path);
void rasterizeModel();
void printMat3(mat3 m);
void printVec3(vec3 v);
void draw(vector<ModelTriangle> model);
void raytraceModel(vector<ModelTriangle> model);
void raytraceModelAA(vector<ModelTriangle> triangles);
void drawWireframes();
void drawWuStrokeTriangle(CanvasTriangle t);
void initialiseLights(vector<ModelTriangle>triangles, int numberOfLights);
vector<ModelTriangle> loadSphere(string filename, float scalefactor);
vector<ModelTriangle> initialiseModels(vector<ModelTriangle> &cornell, vector<ModelTriangle> &sphere, vector<ModelTriangle> &logo);
Colour getReflection(vector<ModelTriangle> &triangles, vec3 source, vec3 ray, int index);
Colour glass(vector<ModelTriangle>triangles, vec3 ray, vec3 intersection, vec3 normal, int depth);
vec3 canvasToWorld(CanvasPoint canvasPoint);
vec3 pointToCanvas(vec3 point);
CanvasPoint vec3ToCanvas(vec3 point);
void spinAndBounce();
void rotateInY(float a);
void rotateInX(float a);
BBOX createBox(string object);
vec3 getSceneCentre(vector<ModelTriangle> triangles);
void showImage(Image image);

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
Image texture = Image();
int mode = 1;
int shadowMode = 1;
int mirrorBox = 1;
int glassBox = 0;
int lightValue = 10;
float lightIntensity = 1.0f;
vector<vec3> lightPositions;
float zArray[WIDTH][HEIGHT];
vec3 sceneCentre;
vector<ModelTriangle> model;
bool backface = false;



int main(int argc, char* argv[]){
    texture = loadImage("checkerboard.ppm");
    vector<ModelTriangle> logo = loadObj("logo/logo.obj");
    vector<ModelTriangle> cornell = loadObj("cornell-box/cornell-box.obj");
    sceneCentre = getSceneCentre(cornell);
    vector<ModelTriangle> sphere = loadSphere("logo/sphere.obj", 0.25f);
    model = initialiseModels(cornell, sphere, logo);
    initialiseLights(model, lightValue);
    for(int i=0; i<lightPositions.size(); i++){
        lightPositions.at(i).y = lightPositions.at(i).y -0.5f;
        lightPositions.at(i).z = lightPositions.at(i).z +0.5f;
    }
    SDL_Event event;
    while(true)
    {
        // We MUST poll for events - otherwise the window will freeze !
        if(window.pollForInputEvents(&event)) {
            handleEvent(event);
        }
        draw();
    }
}

void draw(){
    if (mode != 0){
        window.clearPixels();
        if (mode==1){
            drawWireframes();
            // bounce();
        } else if(mode==2){
            rasterizeModel();

        } else if(mode==3){
            raytraceModel(model);
            mode = 0;
        }
        else if(mode==4){
            raytraceModelAA(model);
            mode = 0;
        }
        else if(mode==5){
            scaleTriangles(model);
            monteTracer(model, window);
        }
        window.renderFrame();
    }
}

//File Handling and Initialisation
Image loadImage(string filename){
    string pSix;
    string data;
    string widthAndHeight;
    string colourRange;
    ifstream file;
    file.open(filename);
    if(file.fail()){
        cout << "SOMETHING WRONG HAS HAPPENED" << endl;
    }
    getline(file, pSix);
    getline(file,data);
    if((!filename.compare("water.ppm")) || (!filename.compare("checkerboard.ppm"))){
        string moreData;
        getline(file, moreData);
    }
    getline(file, widthAndHeight);
    getline(file, colourRange);
    int spacePosition = widthAndHeight.find(" ");
    int newlinePosition = widthAndHeight.find("\n");
    int width = stoi(widthAndHeight.substr(0,spacePosition));
    int height = stoi(widthAndHeight.substr(spacePosition,newlinePosition));
    vector<Colour> pixels;
    int numOfPixels = width * height;
    int red;
    int green;
    int blue;
    for (size_t i = 0; i < numOfPixels; i++ ){
        red = file.get();
        green = file.get();
        blue = file.get();
        Colour pixel = Colour(red, green, blue);
        pixels.push_back(pixel);
    }
    Image i = Image(width, height, pixels);
    return i;
}

void showImage(Image image){
    for(int x =  0; x < image.width; x++){
        for(int y = 0; y < image.height; y++){
            window.setPixelColour(x,y, image.getPixel(x, y).getPacked());
        }
    }
}


void savePPM(int index){
    char red;
    char green;
    char blue;
    uint32_t pixel;
    time_t seconds;
    seconds = time (NULL);
    string filename;
    if(mode == 3 || mode ==4) filename = "./frames/rayFrame" + to_string(index) + ".ppm";
    else filename = "./frames/frame" + to_string(index) + ".ppm";
    ofstream out(filename, std::ios::out | std::ios::binary);
    if(!out){
        std::cerr << "Cannot open:" << filename << endl;
        exit(1);
    }
    out << "P6\n";
    out << "# Created by zackdrei\n";
	out << to_string(WIDTH) << " " << to_string(HEIGHT) << "\n";
	out << "255\n";
    for(int y=0; y<HEIGHT; y++){
        for(int x=0; x<WIDTH; x++){
            pixel = window.getPixelColour(x,y);
            red = char((pixel >> 16 ) & 255);
			green = char((pixel >> 8) & 255);
			blue = char((pixel & 255));
			out << red << green << blue;
        }
    }
    out.close();
    cout << "saved" << endl;
}


vector<Colour> loadMtl(string path){
    string nameLine;
    string propertiesLine;
    ifstream file;
    file.open(path);
    if (file.fail()){
        cout << ".MTL FILE COULD NOT BE OPENED" << endl;
    }
    vector<Colour> colours;
    while (true) {
        getline(file, nameLine);
        //Char 32 is space
        char space = char(32);
        string name = split(nameLine, space)[1];
        // cout << name << endl;
        getline(file, propertiesLine);
        string redString = split(propertiesLine, space)[1];
        string greenString = split(propertiesLine, space)[2];
        string blueString = split(propertiesLine, space)[3];
        Colour c = Colour(name, redString, greenString, blueString);
        colours.push_back(c);
        // cout << c << endl;
        string emptyLine;
        getline(file, emptyLine);
        if( file.eof() ){
            file.close();
            break;
        }
    }

    return colours;
}


Colour getColourByName(vector<Colour> colours, string name){
    for (int i = 0; i<colours.size(); i ++){
        if (!name.compare(colours.at(i).name)){
            return colours.at(i);
        }
    }
    cout << "Colour not found" << endl;
    return Colour();
}


vector<ModelTriangle> loadObj(string path){
    ifstream file;
    file.open(path);
    char space = char(32);
    vector<string> materialFileNames;
    vector<vec3> points;
    vector<Colour> colours;
    vector<ModelTriangle> triangles;
    vector<TexturePoint> texturePoints;
    //Get vertices, and material files
    while (true){
        string line;
        getline(file, line);
        string* items = split(line, space);
        if (!items[0].compare("mtllib")){
            cout<< items[1] << endl;
            vector<Colour> newColours = loadMtl(items[1]);
            colours.insert(colours.end(), newColours.begin(), newColours.end() );
        } else if (!items[0].compare("v")){
            vec3 point = vec3( stof(items[1]), stof(items[2]), stof(items[3]));
            if(!path.compare("logo/logo.obj")){

                point = vec3((stof(items[1])/100)-3, (stof(items[2])/100), (stof(items[3])/100)-1);
            }
            points.push_back(point);
        }
        else if(!items[0].compare("vt")){
            TexturePoint point = TexturePoint(stof(items[1]), stof(items[2]));
            texturePoints.push_back(point);
        }
        if (file.eof() ) break;
    }
    // cout << "Vertices and material files loaded" << endl;
    //Get triangles, and their objects & materials, reset file pointer first
    file.clear();
    file.seekg(0, ios::beg);
    string objectName;
    while (true){
        string line;
        getline(file, line);
        string* items = split(line, space);
        string materialName;
        Colour currentColour;
        if (!items[0].compare("o")){
            objectName = items[1];
            // BAD FIX for reading in extra character
            if(items[1].size() == 11) objectName = "hack_space";
        }
        if (!items[0].compare("usemtl")){
            materialName = items[1];
            currentColour = getColourByName(colours, materialName);
        }
        if (!items[0].compare("f")){
            // number after / represents indices for texture points
            int a, b, c;
            int at, bt, ct;
            const char *chh = line.c_str();
            //cornellbox has no texture points
            if(!path.compare("cornell-box/cornell-box.obj")){
                sscanf(chh, "f %i/ %i/ %i/",&a, &b, &c);
            }
            else {
                sscanf(chh, "f %i/%i %i/%i %i/%i",&a, &at, &b, &bt, &c, &ct);
                at --;
                bt --;
                ct --;
            }
            a --;
            b --;
            c --;

            ModelTriangle t = ModelTriangle(points.at(a), points.at(b), points.at(c), currentColour, objectName);
            string logo = "hack_space";
            if(!objectName.compare(logo)){
                // cout<< "here" << endl;
                t.colour = Colour(0, 0, 255);
                t.texture[0] = vec2(texturePoints.at(at).x, texturePoints.at(at).y);
                t.texture[1] = vec2(texturePoints.at(bt).x, texturePoints.at(bt).y);
                t.texture[2] = vec2(texturePoints.at(ct).x, texturePoints.at(ct).y);
            }

            triangles.push_back(t);
        }
        if (file.eof() ) break;
    }
    // cout << "triangles, objects and materials loaded" << endl;
    return triangles;
}


vector<ModelTriangle> loadSphere(string filename, float scalefactor){
    ifstream file;
    file.open(filename);
    if(file.fail()){
        cout << ".OBJ FILE COULD NOT BE OPENED" << endl;
    }
    char space = char(32);
    vector<string> materialFileNames;
    vector<vec3> points;
    vector<Colour> colours;
    vector<ModelTriangle> triangles;
    vector<TexturePoint> texturePoints;
    vector<vec3> normals;
    while(!file.eof()){
        string line;
        getline(file, line);
        string* items = split(line, space);
        if (!items[0].compare("mtllib")){
            vector<Colour> newColours = loadMtl(items[1]);
            colours.insert(colours.end(), newColours.begin(), newColours.end() );
        } else if (!items[0].compare("v")){
            vec3 point;
            point.x = stof(items[1]) * scalefactor;
            point.y = stof(items[2]) * scalefactor;
            point.z = stof(items[3]) * scalefactor;
            points.push_back(point);
        }
        else if(!items[0].compare("vt")){
            TexturePoint point = TexturePoint(stof(items[1]), stof(items[2]));
            texturePoints.push_back(point);
        }
        else if(!items[0].compare("vn")){
            vec3 normalPoint = vec3(stof(items[1]), stof(items[2]), stof(items[3]));
            normals.push_back(normalPoint);
        }
    }
    file.clear();
    file.seekg(0, ios::beg);
    string objectName;
    while(!file.eof()){
        string line;
        getline(file, line);
        string* items = split(line, space);
        string materialName;
        Colour currentColour;
        if (!items[0].compare("o")){
            objectName = items[1];
        }
        if (!items[0].compare("usemtl")){
            materialName = items[1];
            currentColour = getColourByName(colours, materialName);
        }
        if (!items[0].compare("f")){
            // number after / represents indices for texture points
            int a, b, c;
            //Texture points -- currently not doing anything with them
            int at, bt, ct;
            // normals
            int an, bn, cn;
            const char *chh = line.c_str();
            sscanf(chh, "f %i/%i/%i %i/%i/%i %i/%i/%i",&a, &at, &an, &b, &bt, &bn, &c, &ct, &cn);
            a --;
            b --;
            c --;
            at --;
            bt --;
            ct --;
            an --;
            bn --;
            cn --;
            currentColour = Colour(200, 0, 200);
            ModelTriangle t = ModelTriangle(points.at(a), points.at(b), points.at(c), currentColour, normals.at(an), normals.at(bn), normals.at(cn));
            t.objectName = objectName;
            triangles.push_back(t);
        }
    }
    return triangles;
}


vector<ModelTriangle> initialiseModels(vector<ModelTriangle> &cornell, vector<ModelTriangle> &sphere, vector<ModelTriangle> &logo){
    // find min y coordinate of sphere
    float minY = 9999;
    vec3 sphereMin;
    for(int i = 0; i < sphere.size(); i++){
        ModelTriangle s = sphere.at(i);
        for(int j = 0; j < 3; j++){
            if(s.vertices[j].y < minY){
                minY = s.vertices[j].y;
                sphereMin = s.vertices[j];
            }
        }
    }
    vec3 logoMin;
    for(int i = 0; i < logo.size(); i++){
        ModelTriangle t = sphere.at(i);
        for(int j = 0; j < 3; j++){
            if(t.vertices[j].y < minY){
                minY = t.vertices[j].y;
                logoMin = t.vertices[j];
            }
        }
    }
    //find point on red box to place sphere
    float maxY = -9999;
    float maxX = -9999;
    float minX = 9999;
    float maxZ = -9999;
    float minZ = 9999;
    vector<ModelTriangle> model;
    vec3 backCentre(0,0,0);
    for(int i = 0; i < cornell.size(); i++){
        ModelTriangle t = cornell.at(i);
        // t.colour = Colour(255,255,255);
        if(!t.objectName.compare("back_wall")){
            for (int j = 0; j<3; j++){
                backCentre += t.vertices[j];
            }

        }
        if(!t.objectName.compare("short_box")){
            // top face
            if (i == 17 || i == 12){
                for(int j = 0; j < 3; j++){
                    if(t.vertices[j].y > maxY){
                        maxY = t.vertices[j].y;
                    }
                    if(t.vertices[j].x > maxX){
                        maxX = t.vertices[j].x;
                    }
                    if(t.vertices[j].x < minX){
                        minX = t.vertices[j].x;
                    }
                    if(t.vertices[j].z > maxZ){
                        maxZ = t.vertices[j].z;
                    }
                    if(t.vertices[j].z < minZ){
                        minZ = t.vertices[j].z;
                    }
                }


            }
        }
    }
    backCentre /= 6.0f;
    vector<ModelTriangle> adjustedSphere;
    vec3 centre(0,0,0);
    vec3 adjustment = vec3((maxX+minX)/2 - sphereMin.x, maxY - sphereMin.y, (maxZ+minZ)/2 - sphereMin.z);
    for(int  i = 0; i < sphere.size(); i++){
        ModelTriangle t = sphere.at(i);
        vec3 A = t.vertices[0] + adjustment;
        vec3 B = t.vertices[1] + adjustment;
        vec3 C = t.vertices[2] + adjustment;
        centre += A;
        centre += B;
        centre += C;
        ModelTriangle triangle = ModelTriangle(A, B, C, t.colour);
        triangle.objectName = t.objectName;
        triangle.normals[0] = t.normals[0];
        triangle.normals[1] = t.normals[1];
        triangle.normals[2] = t.normals[2];
        adjustedSphere.push_back(triangle);
    }
    // float scalefactorLogo = 0.5f;
    // vec3 adjustmentLogo = vec3((maxX+minX)/2 - logoMin.x, maxY - logoMin.y, (maxZ+minZ)/2 - logoMin.z +0.5f);
    for(int i = 0; i < logo.size(); i++){
        ModelTriangle t = logo.at(i);
        for(int j = 0; j < 3; j++){
            t.vertices[j].z -= abs(t.vertices[j].z - backCentre.z-0.1f);
            t.colour = Colour(160, 38, 200);
            // t.vertices[j].x *= scalefactorLogo;
            // t.vertices[j].y *= scalefactorLogo;
            // t.vertices[j].z *= scalefactorLogo;
            // t.vertices[j] += adjustmentLogo;
        }
        logo.at(i) = t;

    }
    model.reserve(cornell.size() + adjustedSphere.size() + logo.size());
    model.insert(model.end(), cornell.begin(), cornell.end());
    // only load the logo for the rasterizer
    if(mode ==1 || mode == 3){
        model.insert(model.end(), adjustedSphere.begin(), adjustedSphere.end());
    }
    if(mode == 2){
        model.insert(model.end(), logo.begin(), logo.end());
    }
    return model;
}


void initialiseLights(vector<ModelTriangle>triangles, int numberOfLights){
    float minX = 9999;
    float maxX = -9999;
    float minY = 9999;
    float maxY = -9999;
    float minZ = 9999;
    float maxZ = -9999;
    for(int i=0; i<triangles.size(); i++){
        ModelTriangle t = triangles.at(i);
        if(!t.objectName.compare("light")){
            for (int j=0; j<3; j++){
                if(t.vertices[j].x < minX){
                    minX = t.vertices[j].x;
                }
                if(t.vertices[j].y < minY){
                    minY = t.vertices[j].y;
                }
                if(t.vertices[j].z < minZ){
                    minZ = t.vertices[j].z;
                }
                //max
                if(t.vertices[j].x > maxX){
                    maxX = t.vertices[j].x;
                }
                if(t.vertices[j].y > maxY){
                    maxY = t.vertices[j].y;
                }
                if(t.vertices[j].z > maxZ){
                    maxZ= t.vertices[j].z;
                }
            }
        }
    }
    vector<vec3> bottomLights;
    vector<vec3> rightLights;
    vector<vec3> leftLights;
    vector<vec3> topLights;
    for(int x=0; x<numberOfLights; x++){
        vec3 p;
        p.x = minX +((maxX-minX)*x/(numberOfLights-1));
        p.y = minY +((maxY-minY)*x/(numberOfLights-1));
        p.z = minZ +((maxZ-minZ)*x/(numberOfLights-1));
        bottomLights.push_back(vec3(p.x, p.y, minZ));
        topLights.push_back(vec3(p.x, p.y, maxZ));
        leftLights.push_back(vec3(minX, p.y, p.z));
        rightLights.push_back(vec3(maxX, p.y, p.z));
    }
    for(int j=0; j<leftLights.size(); j++){
        vec3 left = leftLights.at(j);
        vec3 right = rightLights.at(j);
        for(int k=0; k<numberOfLights;k++){
            vec3 light;
            light.x = left.x +((right.x-left.x)*k/(numberOfLights-1));
            light.y = left.y +((right.y-left.y)*k/(numberOfLights-1));
            light.z = left.z +((right.z-left.z)*k/(numberOfLights-1));
            lightPositions.push_back(light);
        }
    }
    cout << "number of lights: " << lightPositions.size() << endl;
}


BBOX createBox(string object){
    vec3 min;
    vec3 max;
    float minx = 99, miny = 99, minz = 99;
    float maxx = -99, maxy = -99, maxz = -99;
    for(int i = 0; i < model.size(); i++){
        ModelTriangle t = model.at(i);
        if(!t.objectName.compare(object)){
            for(int j = 0; j < 3; j++){
                if(t.vertices[j].x < minx) minx = t.vertices[j].x;
                if(t.vertices[j].y < miny) miny = t.vertices[j].y;
                if(t.vertices[j].z < minz) minz = t.vertices[j].z;
                if(t.vertices[j].x > maxx) maxx = t.vertices[j].x;
                if(t.vertices[j].y > maxy) maxy = t.vertices[j].y;
                if(t.vertices[j].z > maxz) maxz = t.vertices[j].z;
            }
        }
    }
    min = vec3(minx, miny, minz);
    max = vec3(maxx, maxy, maxz);
    BBOX box;
    box.min = min;
    box.max = max;
    box.objectName = object;
    return box;
}

// Rasterizer

void drawLine(CanvasPoint from, CanvasPoint to, Colour colour){
    int xDiff = to.x - from.x;
    int yDiff = to.y - from.y;
    float numberOfSteps = std::max(abs(xDiff), abs(yDiff));
    float xStepSize = xDiff/numberOfSteps;
    float yStepSize = yDiff/numberOfSteps;
    for (float i=0.0; i<numberOfSteps; i++) {
        float x = from.x + (xStepSize*i);
        float y = from.y + (yStepSize*i);
        if( (x > 0 && x < WIDTH)&& (y > 0 && y < HEIGHT)){
            window.setPixelColour(round(x), round(y), colour.getPacked());
        }

    }
}

float vertexShader(CanvasPoint point, Colour colour){
    vec3 lightVector = lightPosition - canvasToWorld(point);
    float radius = length(lightVector);
    lightVector = normalize(lightVector);
    float brightness = 5/(0.5 * PI * radius*radius);

    if(brightness > 1){
        brightness = 1.0f;
    }
    if(brightness < 0.2f){
        brightness = 0.2f;
    }
    vec3 adjustedColour = vec3(colour.red, colour.green, colour.blue);
    adjustedColour *= brightness;
    return brightness;
}

float edgeFunction(const CanvasPoint &a, const CanvasPoint &b, const CanvasPoint &c){
    return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

void drawPolygon(CanvasTriangle triangle, bool hasTexture, bool perspective){
    int xmin = numeric_limits<int>::max();
    int ymin = numeric_limits<int>::max();
    int xmax = numeric_limits<int>::min();
    int ymax = numeric_limits<int>::min();
    for( int i = 0; i < 3; i++ ) {
        if (triangle.vertices[i].x < xmin) {
            xmin = triangle.vertices[i].x;
        }
        if (triangle.vertices[i].x > xmax) {
            xmax = triangle.vertices[i].x;
        }
        if (triangle.vertices[i].y < ymin) {
            ymin = triangle.vertices[i].y;
        }
        if (triangle.vertices[i].y > ymax) {
            ymax = triangle.vertices[i].y;
        }
    }
    CanvasPoint v0 = triangle.vertices[0];
    CanvasPoint v1 = triangle.vertices[1];
    CanvasPoint v2 = triangle.vertices[2];

    vec2 t0 = vec2(triangle.vertices[0].texturePoint.x, triangle.vertices[0].texturePoint.y);
    vec2 t1 = vec2(triangle.vertices[1].texturePoint.x, triangle.vertices[1].texturePoint.y);
    vec2 t2 = vec2(triangle.vertices[2].texturePoint.x, triangle.vertices[2].texturePoint.y);

    float b0 = vertexShader(triangle.vertices[0], triangle.colour);
    float b1 = vertexShader(triangle.vertices[1], triangle.colour);
    float b2 = vertexShader(triangle.vertices[2], triangle.colour);


    vec2 edge0 =  vec2(v2.x - v1.x, v2.y - v1.y);
    vec2 edge1 =  vec2(v0.x - v2.x, v0.y - v2.y);
    vec2 edge2 =  vec2(v1.x - v0.x, v1.y - v1.y);
    for(int y = ymin; y < ymax; y++){
        for(int x = xmin; x < xmax; x++){
            CanvasPoint p(x+0.5f, y+0.5f);
            float area = edgeFunction(v0, v1, v2);
            float w0   = edgeFunction(v1, v2,  p)/ area;
            float w1   = edgeFunction(v2, v0,  p)/ area;
            float w2   = edgeFunction(v0, v1,  p)/ area;

            bool overlaps = true;
            float currentDepth = (w0 *triangle.vertices[0].depth) + (w1* triangle.vertices[1].depth) + (w2*triangle.vertices[2].depth);
            float currentBrightness = (w0*b0) + (w1*b1) + (w2*b2);
            if(currentBrightness > 1) currentBrightness = 1.0f;
            if(currentBrightness < 0.2) currentBrightness = 0.2f;
            Colour currentColour = Colour(triangle.colour.red, triangle.colour.green, triangle.colour.blue, currentBrightness*lightIntensity);
            float z = 0, u = 0, v = 0;
            if(perspective){
                z = 1/(w0*(1/v0.depth) +  w1*(1/v1.depth) + w2*(1/v2.depth));
                u = (w0 * ( t0.x / v0.depth )) + (( t1.x / v1.depth ) * w1) + (( t2.x / v2.depth ) * w2);
                v = (w0 * ( t0.y / v0.depth )) + (( t1.y / v1.depth ) * w1) + (( t2.y / v2.depth ) * w2);
                u = u*z;
                v = v*z;
                u = u * texture.width;
                v = v * texture.height;


            }
            vec2 currentTexturePoint = (w0*t0) + (w1*t1) + (w2*t2);
            int xt = currentTexturePoint.x * texture.width;
            int yt = currentTexturePoint.y * texture.height;
            overlaps &= (w0 == 0 ? ((edge0.y == 0 && edge0.x > 0) ||  edge0.y > 0) : (w0 > 0));
            overlaps &= (w1 == 0 ? ((edge1.y == 0 && edge1.x > 0) ||  edge1.y > 0) : (w1 > 0));
            overlaps &= (w1 == 0 ? ((edge2.y == 0 && edge2.x > 0) ||  edge2.y > 0) : (w2 > 0));
            if(overlaps){
                if((x>0 && x< WIDTH) && (y> 0 && y < HEIGHT)){
                    if(1/currentDepth < zArray[x][y]){
                        zArray[x][y] = 1/currentDepth;
                        if(hasTexture && perspective){
                            if((u > 0 && u < texture.width) && (v > 0 && v < texture.height)){
                                Colour textureColour = texture.getPixel(u,v);
                                Colour adjusted = Colour(textureColour.red, textureColour.green, textureColour.blue, currentBrightness);
                                window.setPixelColour(x, y, adjusted.getPacked());
                            }
                            else{
                                if(u > texture.width) u = u - texture.width;
                                if(v > texture.height) v = v - texture.height;
                                Colour textureColour = texture.getPixel((u),(v));
                                window.setPixelColour(x, y, textureColour.getPacked());
                            }

                        }
                        else if(hasTexture && (xt>0 && xt < texture.width && yt> 0 && yt < texture.height) && !perspective){
                            Colour textureColour = texture.getPixel(xt,yt);
                            Colour adjusted = Colour(textureColour.red, textureColour.green, textureColour.blue, 1.0f);
                            window.setPixelColour(x, y, textureColour.getPacked());
                        }
                        else{
                            window.setPixelColour(x, y, currentColour.getPacked());
                        }

                    }
                }
            }
        }
    }
}

void drawStrokeTriangle(CanvasTriangle t){
    drawLine(t.vertices[0], t.vertices[1], t.colour);
    drawLine(t.vertices[0], t.vertices[2], t.colour);
    drawLine(t.vertices[1], t.vertices[2], t.colour);
}

CanvasTriangle triangleToCanvas(ModelTriangle t){
  CanvasTriangle projection;
  for(int i = 0; i<3; i++){
    float xWorld = t.vertices[i].x;
    float yWorld = t.vertices[i].y;
    float zWorld = t.vertices[i].z;
    vec3 a;
    // if(cameraPosition.z < -2) a = vec3(cameraPosition.x - xWorld, cameraPosition.y - yWorld,zWorld- cameraPosition.z);
    // else
    a = vec3(xWorld-cameraPosition.x,yWorld-cameraPosition.y, zWorld- cameraPosition.z);
    a = a * cameraOrientation;
    float ratio = floor(focalLength/a.z);
    int xp = round(-ratio*a.x + WIDTH/2);
    int yp = round(ratio*(a.y) +HEIGHT/2);
    CanvasPoint projectedPoint(xp,yp, a.z);
    projectedPoint.texturePoint = TexturePoint(t.texture[i].x, t.texture[i].y);
    projection.vertices[i] = projectedPoint;
    projection.colour = t.colour;
  }
  return projection;
}

void drawBox(BBOX box){
    Colour white(250,250,250);
    vec3 min = box.min;
    vec3 max = box.max;
    drawLine(vec3ToCanvas(pointToCanvas(min)), vec3ToCanvas(pointToCanvas(vec3(min.x, max.y, min.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(min)), vec3ToCanvas(pointToCanvas(vec3(max.x, min.y, min.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(min)), vec3ToCanvas(pointToCanvas(vec3(min.x, min.y, max.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(vec3(min.x, max.y, min.z))), vec3ToCanvas(pointToCanvas(vec3(max.x, max.y, min.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(vec3(max.x, min.y, min.z))), vec3ToCanvas(pointToCanvas(vec3(max.x, max.y, min.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(max)), vec3ToCanvas(pointToCanvas(vec3(max.x, max.y, min.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(max)), vec3ToCanvas(pointToCanvas(vec3(min.x, max.y, max.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(max)), vec3ToCanvas(pointToCanvas(vec3(max.x, min.y, max.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(vec3(min.x, min.y, max.z))), vec3ToCanvas(pointToCanvas(vec3(min.x, max.y, max.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(vec3(min.x, max.y, max.z))), vec3ToCanvas(pointToCanvas(vec3(min.x, max.y, min.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(vec3(min.x, min.y, max.z))), vec3ToCanvas(pointToCanvas(vec3(max.x, min.y, max.z))), white);
    drawLine(vec3ToCanvas(pointToCanvas(vec3(max.x, min.y, max.z))), vec3ToCanvas(pointToCanvas(vec3(max.x, min.y, min.z))), white);
}

void drawWireframes(){
    for(int i = 0; i < model.size(); i++){
        ModelTriangle t = model.at(i);
        float sumX = 0;
        float sumY = 0;
        float sumZ = 0;
        for(int j = 0; j < 3; j++){
            sumX += t.vertices[j].x;
            sumY += t.vertices[j].y;
            sumZ += t.vertices[j].z;
        }
        vec3 triangleMiddle(sumX/3.0f, sumY/3.0f, sumZ/3.0f);
        vec3 direction = cameraPosition - triangleMiddle;
        float distance = glm::length(direction);
        if(distance < 3 || distance > 30){
            continue;
        }
        CanvasTriangle canvas = triangleToCanvas(t);
        drawWuStrokeTriangle(canvas);
    }
}

vec3 pointToCanvas(vec3 point){
    float xP = point.x;
    float yP = point.y;
    float zP = point.z;
    vec3 a;
    // if(cameraPosition.z < -2) a = vec3(cameraPosition.x - xP, cameraPosition.y - yP,zP - cameraPosition.z);
    // else
    a = vec3(xP-cameraPosition.x, yP-cameraPosition.y, zP- cameraPosition.z);
    a = a * cameraOrientation;
    float ratio = floor(focalLength/a.z);
    int xp = round(-ratio*a.x + WIDTH/2);
    int yp = round(ratio*a.y + HEIGHT/2);
    vec3 projectedPoint(xp, yp, a.z);
    return projectedPoint;
}

CanvasPoint vec3ToCanvas(vec3 v){
    return CanvasPoint(v.x, v.y, v.z);
}

vec3 canvasToWorld(CanvasPoint canvasPoint){
    float ratio = floor(focalLength/canvasPoint.depth);
    float xWorld = (canvasPoint.x - WIDTH/2)/-ratio;
    float yWorld = (canvasPoint.y - HEIGHT/2)/ratio;
    float zWorld = canvasPoint.depth;
    vec3 adjusted(xWorld, yWorld, zWorld);
    adjusted = adjusted * glm::inverse(cameraOrientation) ;
    adjusted = adjusted + cameraPosition;
    return adjusted;
}

ModelTriangle calculateVertexNormals(vector<ModelTriangle> triangles, ModelTriangle t, int currentIndex){
    vec3 A = t.vertices[0];
    vec3 B = t.vertices[1];
    vec3 C = t.vertices[2];
    vector<vec3>vertexNormals;
    vec3 v0Normal(0,0,0);
    vec3 v1Normal(0,0,0);
    vec3 v2Normal(0,0,0);
    int count = 0;
    for(int i = 0; i < triangles.size(); i++){
        ModelTriangle triangle = triangles.at(i);
        if(i == currentIndex) continue;

        if( (A == triangle.vertices[0]) || (A == triangle.vertices[1]) || (A == triangle.vertices[2]) ){
            v0Normal += getNormal(triangle);
            count += 1;
        }
        if( (B == triangle.vertices[0]) || (B == triangle.vertices[1]) || (B == triangle.vertices[2]) ){
            v1Normal += getNormal(triangle);
        }
        if( (C == triangle.vertices[0]) || (C == triangle.vertices[1]) || (C == triangle.vertices[2]) ){
            v2Normal += getNormal(triangle);
        }
        t.normals[0] = normalize(v0Normal);
        t.normals[1] = normalize(v1Normal);
        t.normals[2] = normalize(v2Normal);
    }
    return t;
}

void rasterizeModel(){
    for(int y = 0; y < HEIGHT; y++){
        for(int x = 0; x < WIDTH; x++){
            zArray[x][y] = std::numeric_limits<float>::infinity();
        }
    }

    for(int i = 0; i < model.size(); i++){
        bool hasTexture = false;
        bool perspective = false;
        ModelTriangle t = model.at(i);
        vec3 e0 = vec3(t.vertices[1] - t.vertices[0]);
        vec3 e1 = vec3(t.vertices[2] - t.vertices[0]);
        vec3 normal = normalize(glm::cross(e0,e1));
        vec3 zero(0,0,0);
        if((t.normals[0] == zero) && (t.normals[1] == zero) && (t.normals[2] == zero)){
            t = calculateVertexNormals(model, t, i);
        }
        float sumX = 0;
        float sumY = 0;
        float sumZ = 0;
        for(int j = 0; j < 3; j++){
            sumX += t.vertices[j].x;
            sumY += t.vertices[j].y;
            sumZ += t.vertices[j].z;
        }
        vec3 triangleMiddle(sumX/3.0f, sumY/3.0f, sumZ/3.0f);
        vec3 lightVector = lightPosition - triangleMiddle;
        lightVector = normalize(lightVector);
        vec3 direction = cameraPosition - triangleMiddle;
        float distance = glm::length(direction);
        float triangleOrientation = glm::dot(normal, direction);
        if(triangleOrientation < 0){
            normal = normal * -1.0f;
            continue;
        }
        if(distance < 3 || distance > 30){
            continue;
        }
        CanvasTriangle projectedTriangle = triangleToCanvas(t);
        projectedTriangle.vertices[0].normal = t.normals[0];
        projectedTriangle.vertices[1].normal = t.normals[1];
        projectedTriangle.vertices[2].normal = t.normals[2];
        if(i == 6){
            projectedTriangle.vertices[1].texturePoint = TexturePoint(0,0);
            projectedTriangle.vertices[2].texturePoint = TexturePoint(0,1);
            projectedTriangle.vertices[0].texturePoint = TexturePoint(0.5,1);
        }
        if(i == 7){
            projectedTriangle.vertices[0].texturePoint = TexturePoint(0.5,1);
            projectedTriangle.vertices[1].texturePoint = TexturePoint(0.5,0);
            projectedTriangle.vertices[2].texturePoint = TexturePoint(0,0);
        }
        if(!model.at(i).objectName.compare("floor")){
            hasTexture = true;
            perspective = true;
        }
        drawPolygon(projectedTriangle, hasTexture, perspective);
    }
}

//**** Helper functions for Wu line algorithm *****
float ipart(float x){
    return floor(x);
}

float roundPart(float x){
    return ipart(x+0.5);
}

float fpart(float x){
    return x-floor(x);
}

float rfpart(float x){
    return 1 - fpart(x);
}

void drawPoint(int x, int y, Colour colour, float brightness){
    Colour adjustedColour = Colour(colour.red, colour.green, colour.blue, brightness);
    if((x > 0 && x < WIDTH) && (y > 0 && y < HEIGHT )){
        window.setPixelColour(x, y, adjustedColour.getPacked());
    }
}

void drawWuLine(CanvasPoint from, CanvasPoint to, Colour colour){
    float x0 = from.x;
    float y0 = from.y;
    float x1 = to.x;
    float y1 = to.y;
    int steep = abs(y1-y0) > abs(x1-x0);

    if(steep){
        swap(x0, y0);
        swap(x1, y1);
    }
    if(x0 > x1){
        swap(x0, x1);
        swap(y0, y1);

    }
    //compute the slope
    float dx = x1-x0;
    float dy = y1-y0;
    float gradient = dy/dx;
    if(dx == 0.0){
        gradient = 1;
    }
    //first endpoint
    int xend = round(x0);
    float yend = y0 + (gradient * (xend-x0));
    float xgap = fpart(x0+0.5);
    int xpxl1 = xend;
    int ypxl1 = ipart(yend);

    if(steep){
        drawPoint(ypxl1, xpxl1, colour, rfpart(yend)*xgap);
        drawPoint(ypxl1+1, xpxl1, colour, fpart(yend)*xgap);
    }
    else{
        drawPoint(xpxl1, ypxl1  , colour, rfpart(yend) * xgap);
        drawPoint(xpxl1, ypxl1+1, colour, fpart(yend) * xgap);
    }
    float yintersect =  yend + gradient; // first y intersection

    //second enpoint
    xend = round(x1);
    yend = y1 + (gradient *(xend - x1));
    xgap = fpart(x1 +0.5);
    int xpxl2 = xend;
    int ypxl2 = ipart(yend);

    if(steep){
        drawPoint(ypxl2, xpxl2, colour, rfpart(yend) * xgap);
        drawPoint(ypxl2+1, xpxl2, colour, fpart(yend) * xgap);
    }
    else{
        drawPoint(xpxl2, ypxl2, colour, rfpart(yend) * xgap);
        drawPoint(xpxl2, ypxl2+1, colour, fpart(yend) * xgap);
    }

    // main loop
    if(steep){
        for(int x=xpxl1+1; x<xpxl2-1; x++){
            drawPoint(ipart(yintersect), x, colour, rfpart(yintersect));
            drawPoint(ipart(yintersect)+1, x, colour, fpart(yintersect));
            yintersect = yintersect + gradient;
        }
    }
    else{
        for(int x=xpxl1+1; x<xpxl2-1; x++){
            drawPoint(x, ipart(yintersect), colour, rfpart(yintersect));
            drawPoint(x, ipart(yintersect)+1, colour, fpart(yintersect));
            yintersect = yintersect + gradient;
        }
    }
}

void drawWuStrokeTriangle(CanvasTriangle t){
    drawWuLine(t.vertices[0], t.vertices[1], t.colour);
    drawWuLine(t.vertices[0], t.vertices[2], t.colour);
    drawWuLine(t.vertices[1], t.vertices[2], t.colour);
}


//ANIMATION
void zoomAndLight(){
    // workout plane normal for ceiling to make sure light is always parallel to it
    vec3 planeNormal(0,0,0);
    for(int i=0; i< model.size(); i++){
        ModelTriangle triangle = model.at(i);
        if(!triangle.objectName.compare("ceiling")){
            planeNormal += getNormal(triangle);
        }
    }
    cameraPosition = vec3(9,2.5,30);
    planeNormal = normalize(planeNormal);
    vec3 lightSpeed(0,0,0);
    vec3 cameraSpeed(0,0,0);
    int frameCounter = 0;
    for(float t=0.0f; t<5.0f; t+=0.01f){
        frameCounter +=1;
        if(t < 1.0f){
            lightSpeed.x = -0.0135f;
            lightSpeed.y = 0;
            lightSpeed.z = -0.014f;

            cameraSpeed.z = -0.225;
            cameraSpeed.y = 0.0;
            cameraSpeed.x = -0.043;
        }
        else if(t >= 1.0f && t < 2.0f){
            lightSpeed.z =  0.04;
            lightSpeed.x = 0;
            lightSpeed.y = 0;
            cameraSpeed.z = 0;
            cameraSpeed.y = 0;
            cameraSpeed.x = -0.0323;
        }
        else if(t >= 2.0f && t < 3.0f){
            lightSpeed.x =  0.0411;
            lightSpeed.y = 0;
            lightSpeed.z = 0;
        }
        else if (t>= 3.0f && t < 4.0f){
            lightSpeed.z = -0.04;
            lightSpeed.x = 0;
            lightSpeed.y = 0;
        }
        else{
            lightSpeed.x = 0;
            lightSpeed.y = 0;
            lightSpeed.z = 0;
            cameraSpeed.z = 0.225;
            cameraSpeed.y = 0.0;
            cameraSpeed.x = -0.0425;
        }


        lightPosition += lightSpeed;
        cameraPosition += cameraSpeed;
        draw();
        // savePPM(frameCounter);
        window.renderFrame();
    }

}

void spinAndBounce(){
    int frameCounter = 0;
    cameraPosition = vec3(-11.5,2.5,27);
    ModelTriangle t1 = model.at(12);
    ModelTriangle t2 = model.at(17);
    vec3 centre = vec3(t1.vertices[0] + t1.vertices[1] + t1.vertices[2] + t2.vertices[0] + t2.vertices[1] + t2.vertices[2])/6.0f;

    float miny = 99;
    vec3 min;
    for(int i = 0; i < model.size(); i++){
        ModelTriangle t = model.at(i);
        if(!t.objectName.compare("sphere")){
            if(t.vertices[0].y < miny){
                miny = t.vertices[0].y;
                min = t.vertices[0];
            }
            if(t.vertices[1].y < miny){
                miny = t.vertices[1].y;
                min = t.vertices[1];
            }
            if(t.vertices[2].y < miny){
                miny = t.vertices[2].y;
                min = t.vertices[2];
            }
        }
    }
    float overallTime = 5.0f;
    float ts = 0.01f;
    float gravity = 0.010f;
    float vy = 0;
    vec3 cameraSpeed(0,0,0);
    vec3 dir(0,-1,0);
    int numberBounces = 0;
    mat3 rotateY(vec3(cos(PI/90),0,sin(PI/-90)),vec3(0.0,1.0,0),vec3(-sin(PI/-90),0,cos(PI/-90)));

    for(float t = 0; t < overallTime; t += ts ){
        vy += gravity;
        if(numberBounces >= 62 && length(min - centre) < 0.12){
            vy = 0;

        }
        if(t < 1.5){
            cameraSpeed.x = 0.049f;
            cameraSpeed.z = -0.126f;
            cameraSpeed.y = 0;
        }
        else if(t>=1.5 && t < 3.3){
            // cameraPosition.x += 0.098f;
            cameraSpeed.x = 0.04f;
            cameraSpeed.y = 0;
            cameraSpeed.z = 0;
        }
        else if(t>=3.3){
            cameraSpeed.x = 0.049f;
            cameraSpeed.z = 0.14f;
            cameraSpeed.y = 0;
        }
        else {
            cameraSpeed.x = 0;
            cameraSpeed.z = 0;
            cameraSpeed.y = 0;
        }

        for(int i = 0; i< model.size(); i++){
            ModelTriangle triangle = model.at(i);
            if(!triangle.objectName.compare("tall_box") && t < 4.12){
                for(int j = 0; j < 3; j++){
                    triangle.vertices[j].x += 1.12f;
                    triangle.vertices[j].y += -3.14f;
                    triangle.vertices[j].z += 3.76f;
                    triangle.vertices[j] = (triangle.vertices[j]* rotateY);
                    triangle.vertices[j].x += -1.12f;
                    triangle.vertices[j].y += 3.14f;
                    triangle.vertices[j].z += -3.76f;
                    model.at(i) = triangle;
                }
            }
            if(!triangle.objectName.compare("sphere")){
                for(int j=0; j<3; j++){
                    triangle.vertices[j] += (dir * vy);
                    model.at(i) = triangle;
                }
            }
        }
        min += (dir * vy);
        if(length(min - centre) < 0.12){
            numberBounces +=1;
            vy *= -1.0f;
        }
        cameraPosition += cameraSpeed;
        draw();
        window.renderFrame();
        // savePPM(frameCounter);
        frameCounter += 1;
    }

}

void panStop(){
    mode = 3;
    int frameCounter = 0;
    cameraPosition = vec3(-11.5,2.5,27);
    float overallTime = 5.0f;
    float ts = 0.01f;


    vec3 cameraSpeed(0,0,0);
    vec3 dir(0,-1,0);

    for(float t = 0; t < overallTime; t += ts ){

        if(t < 1.5){
            backface = true;
            cameraSpeed.x = 0.049f;
            cameraSpeed.z = -0.126f;
            cameraSpeed.y = 0;
        }
        else if(t>=1.5 && t < 2.5){
            backface = false;
            // mode = 4;
            lightIntensity -= 0.01f;
            // cameraPosition.x += 0.098f;
            cameraSpeed.x = 0.038;
            cameraSpeed.y = 0;
            cameraSpeed.z = 0;
        }
        else if(t>=2.5 && t < 2.6){
            cameraSpeed.x = 0;
            cameraSpeed.y = 0;
            cameraSpeed.z = 0;
        }
        else if(t>= 2.6 && t< 3.6){
            lightIntensity += 0.01f;
            cameraSpeed.x = 0.038;
            cameraSpeed.y = 0;
            cameraSpeed.z = 0;

        }
        else if(t>= 3.6){
            backface = true;
            cameraSpeed.x += 0.004f;
            cameraSpeed.z += 0.008f;
            cameraSpeed.y = 0;
        }
        else {
            cameraSpeed.x = 0;
            cameraSpeed.z = 0;
            cameraSpeed.y = 0;
        }
        cameraPosition += cameraSpeed;
        draw();
        window.renderFrame();
        savePPM(frameCounter);
        frameCounter += 1;
    }
    backface = false;

}

void panStop2(){
    mode = 3;
    int frameCounter = 1000;
    cameraPosition = vec3(-11.5,2.5,27);
    float overallTime = 4.5f;
    float ts = 0.01f;


    vec3 cameraSpeed(0,0,0);
    vec3 dir(0,-1,0);

    for(float t = 0; t < overallTime; t += ts ){

        if(t < 1.5){
            backface = true;
            cameraSpeed.x = 0.049f;
            cameraSpeed.z = -0.126f;
            cameraSpeed.y = 0;
        }
        else if(t>=1.5 && t < 3.5){
            if(t>1.7f) backface = false;
            if(t == 2.5) mode = 4;
            else mode = 3;
            cameraSpeed.x = 0.035f;
            cameraSpeed.y = 0;
            cameraSpeed.z = 0;
        }

        else if(t>= 3.5){
            if(t>3.6)backface = true;
            cameraSpeed.x += 0.002f;
            cameraSpeed.z += 0.010f;
            cameraSpeed.y = 0;
        }
        else {
            cameraSpeed.x = 0;
            cameraSpeed.z = 0;
            cameraSpeed.y = 0;
        }
        cout << t << endl;
        cameraPosition += cameraSpeed;
        draw();
        window.renderFrame();
        savePPM(frameCounter);
        frameCounter += 1;
    }
    backface = false;

}

void flythrough(){
    mode = 3;
    int frameCounter = 0;
    cameraPosition = vec3(0,1,14);
    vec3 cameraSpeed(0,0,0);
    for(float t = 0.0f; t < 6.0f; t+=0.01){
        float angle = -0.9f*PI/180.0f;
        if(t < 2.0f){
            cameraSpeed.x =  0.0125f;
            cameraSpeed.z = -0.095f;
            cameraSpeed.y =  0.0125f;
            if(t>=1.0f){
                angle = -1.35*PI/180.0f;
                rotateInY(angle);
                rotateInX(0.1*PI/180.0f);
            }
        }
        else if(t>=2.0f && t < 3.0f){
            if(t <=2.5f){
                rotateInY(0.9*PI/180.0f);
            }
            cameraSpeed.x = -0.05;
            cameraSpeed.y =  0.0f;
            cameraSpeed.z =  0.0f;
        }
        else if(t >= 3.0f && t < 4.0f){
            cameraSpeed.x = 0.0f;
            cameraSpeed.y = 0.0f;
            cameraSpeed.z = 0.065;
        }
        else{
            cameraSpeed = vec3(0,0,0);
        }
        cameraPosition += cameraSpeed;
        draw();
        window.renderFrame();
        // savePPM(frameCounter);
        frameCounter += 1;
    }
}

//RAYTRACER

float getBrightness(vec3 normal, vec3 lightToIntersection, vec3 ray){
    float brightness = 1/(4 * PI * length(lightToIntersection)*length(lightToIntersection));
    float angleOI = dot(normalize(lightToIntersection),normalize(normal));
    if (angleOI > 0){
        brightness += angleOI;
    }
    vec3 flippedRay = -1.0f* ray;
    vec3 reflected = lightToIntersection - (2.0f *normalize(normal) * dot(lightToIntersection,normalize(normal)));
    float angleRV = dot( normalize(flippedRay), normalize(reflected));
    if (angleRV > 0.0f){
        float intensity =  pow(angleRV, 24.0f);
        if(intensity < 0) intensity = 0;
        brightness += intensity;
    }
    if(brightness < 0.15){
        brightness = 0.15f;
    }
    return brightness;
}

bool inHardShadow(vector<ModelTriangle> triangles, vec3 surfacePoint, int currentTriangleIndex, vec3 light){
    vec3 shadowRay = light-surfacePoint;
    bool inShadow = false;
    float distanceFromLight = glm::length(shadowRay);
    for(int i=0; i<triangles.size(); i++){
        ModelTriangle triangle = triangles.at(i);
        vec3 e0 = vec3(triangle.vertices[1] - triangle.vertices[0]);
        vec3 e1 = vec3(triangle.vertices[2] - triangle.vertices[0]);
        vec3 SPVector = vec3(surfacePoint-triangle.vertices[0]);
        mat3 DEMatrix(-glm::normalize(shadowRay), e0, e1);
        vec3 possibleSolution = glm::inverse(DEMatrix) * SPVector;
        float t = possibleSolution.x;
        if(solutionOnTriangle(possibleSolution) && (i != currentTriangleIndex) && t>0.001f){
            if(abs(t) < (distanceFromLight) ){
                inShadow = true;
            }
        }
    }
    return inShadow;
}

float getSoftShadow(vec3 surfaceNormal, vec3 surfacePoint, vector<ModelTriangle> triangles, int currentTriangleIndex){
    float shadeValue = 0.0f;
    float shadowCount = 0.0f;
    for(int i=0; i<lightPositions.size(); i++){
        if(inHardShadow(triangles, surfacePoint,currentTriangleIndex,lightPositions.at(i))){
            shadowCount ++;
        }
    }
    shadeValue = shadowCount/lightPositions.size();
    return shadeValue;
}

vec3 calculateNormal(ModelTriangle triangle, vec3 possibleSolution){
    vec3 n0 = triangle.normals[0];
    vec3 n1 = triangle.normals[1];
    vec3 n2 = triangle.normals[2];
    vec3 normal = n0 + possibleSolution.y*(n1-n0) +possibleSolution.z*(n2-n0);
    return normal;
}

vec3 getReflectedRay(vec3 normal, vec3 ray){
    vec3 reflection  = normalize(ray - (2* dot(ray,normal) * normal));
    return reflection;
}

RayTriangleIntersection getClosestIntersection(vector<ModelTriangle> triangles, vec3 ray, vec3 startPosition, int depth){
    RayTriangleIntersection closestIntersection;
    closestIntersection.distanceFromCamera = std::numeric_limits<float>::infinity();
    for (int i=0; i<triangles.size(); i++){
        ModelTriangle triangle = triangles.at(i);
        vec3 e0 = vec3(triangle.vertices[1] - triangle.vertices[0]);
        vec3 e1 = vec3(triangle.vertices[2] - triangle.vertices[0]);
        vec3 SPVector = vec3(startPosition-triangle.vertices[0]);
        mat3 DEMatrix(-ray, e0, e1);
        vec3 possibleSolution = glm::inverse(DEMatrix) * SPVector;
        if (solutionOnTriangle(possibleSolution) && possibleSolution.x > 0.0001f){
            if (possibleSolution.x < closestIntersection.distanceFromCamera){

                vec3 intersection = triangle.vertices[0] + possibleSolution.y*e0 + possibleSolution.z*e1;
                vec3 lightToIntersection = (lightPosition - intersection);
                Colour adjustedColour = Colour(0,0,0);
                if((i == 26 || i == 31) && mirrorBox && depth > 0){
                    vec3 planeNormal = normalize(glm::cross(e0, e1));
                    vec3 reflectedRay = getReflectedRay(planeNormal, ray);
                    RayTriangleIntersection reflectedIntersection = getClosestIntersection(triangles, reflectedRay, intersection+(0.0001f * planeNormal), depth -1);
                    if(reflectedIntersection.distanceFromCamera < std::numeric_limits<float>::infinity()){
                        adjustedColour = reflectedIntersection.intersectionPointColour;
                    }
                    else{
                        adjustedColour = Colour(0,0,0);
                    }
                }
                else if(!triangle.objectName.compare("short_box") && glassBox && depth > 0){
                    vec3 triangleNormal = normalize(glm::cross(e0, e1));
                    adjustedColour = glass(triangles, ray, intersection, triangleNormal, depth -1);

                }
                else{
                    vec3 normal;
                    if(!triangle.objectName.compare("sphere")){
                        normal = calculateNormal(triangle, possibleSolution);
                    }
                    else{
                        normal = glm::cross(e0,e1);
                    }

                    float brightness = getBrightness(normal,lightToIntersection, ray);
                    if(shadowMode == 0){
                        bool inShadow = inHardShadow(triangles, intersection, i, lightPosition);
                        if(inShadow){
                            brightness = 0.15f;
                        }
                    }
                    else if(shadowMode == 1){
                        float shadeValue = getSoftShadow(normal, intersection, triangles, i);
                        float shadeProp = 1 - shadeValue;
                        brightness *= shadeProp;
                        if(brightness < 0.15){
                            brightness = 0.15f;
                        }

                    }
                    if(brightness > 1.0f){
                        brightness = 1.0f;
                    }//CHANGE
                    if(dot(normal, (cameraPosition - intersection)) < 0 &&(triangle.objectName.compare("sphere") && backface)){
                        brightness = 0.8f;
                    }
                    adjustedColour = Colour(triangle.colour.red, triangle.colour.green, triangle.colour.blue, brightness);

                }
                closestIntersection = RayTriangleIntersection(intersection, possibleSolution.x, triangle, adjustedColour);
            }
        }
    }
    return closestIntersection;
}

vec3 refract(vec3 ray, vec3 planeNormal, float refractiveIndex){
    float cosI = glm::dot(ray,planeNormal); // negative --> entering material, positive -->leaving
    float etai = 1, etat = refractiveIndex;
    vec3 n = planeNormal;
    if(cosI < 0.0f){
        cosI = -cosI;
    }
    else{
        std::swap(etai, etat);
        n = -planeNormal;
    }
    // check for total internal reflection
    float eta = etai/etat;
    float k = 1.0f - eta * eta * (1.0f - cosI * cosI);

    if(k < 0){
        return vec3(0,0,0);
    }
    vec3 refracted = eta * ray + (eta * cosI - sqrtf(k)) * n;
    return normalize(refracted);
}

float fresnel(vec3 ray, vec3 normal, float refractiveIndex){
    float cosi = dot(ray, normal);
    float etai = 1, etat = refractiveIndex;
    if (cosi > 0)std::swap(etai, etat);
    float sint = etai / etat * sqrtf(std::max(0.0f, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1) {
        return 1;
    }
    else {
        float cost = sqrtf(std::max(0.0f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp)*0.5f;
    }
    return 0;
}

Colour glass(vector<ModelTriangle>triangles, vec3 ray, vec3 intersection, vec3 normal, int depth){
    const float refractiveIndex = 1.3f;
    vec3 reflectedRay = getReflectedRay(normal, ray);
    RayTriangleIntersection reflectedIntersection = getClosestIntersection(triangles, reflectedRay, intersection, depth - 1);
    vec3 refractionRay = refract(ray, normal, refractiveIndex);
    if(refractionRay == vec3(0,0,0)){
        if(reflectedIntersection.distanceFromCamera < std::numeric_limits<float>::infinity()){
            return reflectedIntersection.intersectionPointColour;
        }
        else{
            return Colour(0,0,0);
        }

    }
    RayTriangleIntersection refractedIntersection = getClosestIntersection(triangles, refractionRay,intersection, depth -1);
    if(refractedIntersection.distanceFromCamera < std::numeric_limits<float>::infinity()){
        return refractedIntersection.intersectionPointColour;
    }
    else{
        if(reflectedIntersection.distanceFromCamera < std::numeric_limits<float>::infinity()){
                return reflectedIntersection.intersectionPointColour;
            }
            else{
                return Colour(0,0,0);
            }
    }
}

void raytraceModel(vector<ModelTriangle> triangles){
#pragma omp parallel for
    for(int y= 0; y< HEIGHT; y++){
        for (int x = 0; x < WIDTH; x++){
            vec3 point = vec3(WIDTH/2 -x, y-(HEIGHT/2), focalLength);
            vec3 ray = cameraPosition - point;
            ray = normalize(ray * glm::inverse(cameraOrientation));
            RayTriangleIntersection intersection = getClosestIntersection(triangles, ray, cameraPosition, 10);
            if (!std::isinf( intersection.distanceFromCamera )){
                window.setPixelColour(x,y, intersection.intersectionPointColour.getPacked());
            }

        }
    }
}

void raytraceModelAA(vector<ModelTriangle> triangles){
//Quincux sampling
#pragma omp parallel for
    for(int y = 0; y < HEIGHT; y++){
        for (int x = 0; x < WIDTH; x++ ){
            vec3 point = vec3(WIDTH/2 -x, y-(HEIGHT/2), focalLength);
            vec3 ray = cameraPosition - point;
            vec3 rayTopLeft = vec3(ray.x-0.5, ray.y, ray.z);
            vec3 rayTopRight = vec3(ray.x+0.5, ray.y, ray.z);
            vec3 rayBottomLeft = vec3(ray.x, ray.y-0.5, ray.z);
            vec3 rayBottomRight = vec3(ray.x, ray.y+0.5, ray.z);

            ray = normalize(ray * glm::inverse(cameraOrientation));
            rayTopLeft = normalize(rayTopLeft* glm::inverse(cameraOrientation));
            rayTopRight = normalize(rayTopRight* glm::inverse(cameraOrientation));
            rayBottomLeft = normalize(rayBottomLeft* glm::inverse(cameraOrientation));
            rayBottomRight = normalize(rayBottomRight* glm::inverse(cameraOrientation));

            RayTriangleIntersection intersection = getClosestIntersection(triangles, ray, cameraPosition, 1);
            RayTriangleIntersection intersectionTL = getClosestIntersection(triangles, rayTopLeft, cameraPosition, 1);
            RayTriangleIntersection intersectionTR = getClosestIntersection(triangles, rayTopRight, cameraPosition, 1);
            RayTriangleIntersection intersectionBL = getClosestIntersection(triangles, rayBottomLeft, cameraPosition, 1);
            RayTriangleIntersection intersectionBR = getClosestIntersection(triangles, rayBottomRight, cameraPosition, 1);

            if (!std::isinf( intersection.distanceFromCamera )){
                Colour average;
                average.red = (intersection.intersectionPointColour.red*1.4 + intersectionTL.intersectionPointColour.red*0.9 + intersectionTR.intersectionPointColour.red*0.9 + intersectionBL.intersectionPointColour.red*0.9 + intersectionBR.intersectionPointColour.red*0.9)/5.0;
                average.green = (intersection.intersectionPointColour.green*1.4 + intersectionTL.intersectionPointColour.green*0.9 + intersectionTR.intersectionPointColour.green*0.9 + intersectionBL.intersectionPointColour.green*0.9 + intersectionBR.intersectionPointColour.green*0.9)/5.0;
                average.blue = (intersection.intersectionPointColour.blue*1.4 + intersectionTL.intersectionPointColour.blue*0.9 + intersectionTR.intersectionPointColour.blue*0.9 + intersectionBL.intersectionPointColour.blue*0.9 + intersectionBR.intersectionPointColour.blue*0.9)/5.0;
                window.setPixelColour(x,y, average.getPacked());
            }
        }
    }
}

vec3 findTriangleCentre(ModelTriangle t){
    vec3 sum(0,0,0);
    for(int j = 0; j < 3; j++){
        sum += t.vertices[j];
    }
    sum /= 3.0f;
    return sum;
}

vec3 getSceneCentre(vector<ModelTriangle> triangles){
    vec3 sum(0,0,0);
    for(int i = 0; i < triangles.size(); i++){
        ModelTriangle t = triangles.at(i);
        sum += findTriangleCentre(t);
    }
    sum /= (float)(triangles.size());
    return sum;
}

void rotateInX(float a){

    mat3 m = mat3(vec3(1,      0,       0),
                  vec3(0, cos(a), -sin(a)),
                  vec3(0, sin(a), cos(a)));
    cameraOrientation = cameraOrientation * m ;
}

void rotateInY(float a){
    mat3 m = mat3(vec3(cos(a),  0, sin(a)),
                  vec3(0,       1,      0),
                  vec3(-sin(a), 0, cos(a)));
    cameraOrientation = cameraOrientation * m ;
}

void rotateInZ(float a){
    mat3 m = mat3(  vec3(cos(a), -sin(a), 0),
                    vec3(sin(a), cos(a), 0),
                    vec3(0, 0, 1));
    cameraOrientation = cameraOrientation * m;

}

void lookAt(vec3 p){
  vec3 forward = normalize(cameraPosition - p);
  vec3 right = normalize(cross(vec3(0,1,0), forward));
  vec3 up = normalize(cross(forward, right));
  cameraOrientation[0] = right;
  cameraOrientation[1] = up;
  cameraOrientation[2] = forward;
  printMat3(cameraOrientation);
}

void resetCamera(){
    cameraOrientation = mat3(       vec3(1,0,0),
                                    vec3(0,1,0),
                                    vec3(0,0,1));
    cameraPosition = vec3(-0.25,2.5,8);
    lightPosition = vec3(-0.2334011,4.8,-3.043968);
    lightIntensity = 1.0f;
}

void printMat3(mat3 m){
    for (int y =0; y<3;y++){
        cout << "(" << m[y][0] << "," << m[y][1] << "," << m[y][2] << ")" << endl;
    }
    cout << endl;
}

void printVec3(vec3 v){
  cout << "(" << v[0] << "," << v[1] << "," << v[2] << ")" << endl;
}

void handleEvent(SDL_Event event){
    if(event.type == SDL_KEYDOWN) {
        if(event.key.keysym.sym == SDLK_LEFT) {
            cameraPosition.x -=0.1;
        }
        else if(event.key.keysym.sym == SDLK_RIGHT) {
            cameraPosition.x += 0.1;

        }
        else if(event.key.keysym.sym == SDLK_UP) {
            cameraPosition.y += 0.1;

        }
        else if(event.key.keysym.sym == SDLK_DOWN) {
            cameraPosition.y -= 0.1;

        }
        else if(event.key.keysym.sym == SDLK_RIGHTBRACKET) {
            cameraPosition.z += 0.1;
        }
        else if(event.key.keysym.sym == SDLK_LEFTBRACKET) {
            cameraPosition.z -= 0.1;
        }
        else if(event.key.keysym.sym == SDLK_c) {
            cout << "C" << endl;
            window.clearPixels();
        }
        else if(event.key.keysym.sym == SDLK_w) {
            cout << "W" << endl;
            float a = 2*PI/180;
            rotateInX(a);

        }
        else if(event.key.keysym.sym == SDLK_s) {
            cout << "S" << endl;
            float a = -2*PI/180;
            rotateInX(a);
        }
        else if(event.key.keysym.sym == SDLK_a) {
            cout << "A" << endl;
            float a = 2*PI/180;
            rotateInY(a);
        }
        else if(event.key.keysym.sym == SDLK_d) {
            cout << "D" << endl;
            float a = -2*PI/180;
            rotateInY(a);

        }
        else if(event.key.keysym.sym == SDLK_q) {
            cout << "Q" << endl;
            float a = 2*PI/180;
            rotateInZ(a);
        }
        else if(event.key.keysym.sym == SDLK_e) {
            cout << "E" << endl;
            float a = -2*PI/180;
            rotateInZ(a);
        }
        else if(event.key.keysym.sym == SDLK_l) {
            cout << "L" << endl;
            window.clearPixels();
            lookAt(sceneCentre);
        }
        else if(event.key.keysym.sym == SDLK_u){
            lightPosition.z -=0.05;
        }
        else if(event.key.keysym.sym == SDLK_j){
            lightPosition.z +=0.05;
        }
        else if(event.key.keysym.sym == SDLK_k){
            lightPosition.x +=0.05;
        }
        else if(event.key.keysym.sym == SDLK_h){
            lightPosition.x -=0.05;
        }
        else if(event.key.keysym.sym == SDLK_y){
            lightPosition.y +=0.05;
        }
        else if(event.key.keysym.sym == SDLK_g){
            lightPosition.y -=0.05;
        }
        else if(event.key.keysym.sym == SDLK_r) {
            cout << "Reset" << endl;
            resetCamera();
            vector<ModelTriangle> cornell = loadObj("cornell-box/cornell-box.obj");
            vector<ModelTriangle> logo = loadObj("logo/logo.obj");
            vector<ModelTriangle> sphere = loadSphere("logo/sphere.obj", 0.25f);
            model = initialiseModels(cornell, sphere, logo);
        }
        else if(event.key.keysym.sym == SDLK_1) {
            mode = 1;
            cout << "Wireframe:" << endl;
        }
        else if(event.key.keysym.sym == SDLK_2) {
            mode = 2;
            cout << "Rasterize:" << endl;
        }
        else if(event.key.keysym.sym == SDLK_3) {
            mode = 3;
            cout << "Raytrace:" << endl;
        }
        else if(event.key.keysym.sym == SDLK_4) {
            mode = 4;
            cout << "Raytrace + AA:" << endl;
        }
        else if(event.key.keysym.sym == SDLK_5) {
            mode = 5;
            cout << "monteTracer:" << endl;
        }
        else if(event.key.keysym.sym == SDLK_p) {
            savePPM(42069);
        }
        else if(event.key.keysym.sym == SDLK_b) {
            cout << "BOUNCE" << endl;
            // animation = 0;
            spinAndBounce();
        }
        else if(event.key.keysym.sym == SDLK_f) {
            cout << "pan" << endl;
            panStop();
            panStop2();
        }
        else if(event.key.keysym.sym == SDLK_z) {
            cout << "ZOOM" << endl;
            zoomAndLight();
        }
    }
    else if(event.type == SDL_MOUSEBUTTONDOWN) cout << "MOUSE CLICKED" << endl;
}
