#include <iostream>
#include <vector>

using namespace std;

class Image
{
  public:
      int width;
      int height;
      vector<uint32_t> pixels;



    Image(int w, int h, vector<uint32_t> p)
    {
      pixels = p;
      width = w;
      height = h;
    }

    uint32_t getPixel(int x, int y){
        return (pixels.at((y*width)+x));
    }

};

std::ostream& operator<<(std::ostream& os, const Image& i)
{
    os << "Image: Width= " << i.width << ", Height= " << i.height << std::endl;
    return os;
}
