#include <iostream>
#include <string>
class Colour
{
  public:
    std::string name;
    //Ints 0 - 255
    int red;
    int green;
    int blue;

    Colour()
    {
    }

    Colour(int r, int g, int b)
    {
      name = "";
      red = r;
      green = g;
      blue = b;
    }

    Colour(std::string n, int r, int g, int b)
    {
      name = n;
      red = r;
      green = g;
      blue = b;
    }
    Colour(int r, int g, int b, float brightness)
    {
      red = r * brightness;
      green = g * brightness;
      blue = b * brightness;
    }

    Colour(std::string n, std::string r, std::string g, std::string b){
        name = n;
        red = stof(r) * 255;
        green = stof(g) * 255;
        blue = stof(b) * 255;
        // red = 255;
        // green = 255;
        // blue = 255;
    }

    uint32_t getPacked(){
        uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
        return colour;
    }


};

std::ostream& operator<<(std::ostream& os, const Colour& colour)
{
    os << colour.name << " [" << colour.red << ", " << colour.green << ", " << colour.blue << "]" << std::endl;
    return os;
}
