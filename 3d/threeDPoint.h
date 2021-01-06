#include <iostream>
#include <vector>

using namespace std;

class threeDPoint
{
  public:
    float x;
    float y;
    float z;

    threeDPoint(float xPos, float yPos, float zPos)
    {
      x = xPos;
      y = yPos;
      z = zPos;
    }

};

std::ostream& operator<<(std::ostream& os, const threeDPoint& point)
{
    os << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    return os;
}
