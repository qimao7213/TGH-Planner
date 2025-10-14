#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

#define INTPOINT IntPoint

/*! A light-weight integer point with fields x,y */
class IntPoint
{
public:
  IntPoint() : x(0), y(0)
  {
  }
  IntPoint(int _x, int _y) : x(_x), y(_y)
  {
  }
  int x, y;
  bool operator==(const IntPoint& other) const {
    return x == other.x && y == other.y;
  }
  //重载加法
  IntPoint operator+(const IntPoint& other) const {
    return IntPoint(x + other.x, y + other.y);
  }
  IntPoint operator-(const IntPoint& other) const {
    return IntPoint(x - other.x, y - other.y);
  } 
  bool operator<(const IntPoint& other) const {
        return (y < other.y) || (y == other.y && x < other.x);
  }
};

#endif
