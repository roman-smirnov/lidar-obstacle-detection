#ifndef BOX_H_
#define BOX_H_

#include "common.h"

struct Box final {

  const float xmin, xmax, ymin, ymax, zmin, zmax;

  constexpr explicit Box(const Point& min, const Point& max) :
      xmin(min.x), xmax(max.x), ymin(min.y), ymax(max.y), zmin(min.z), zmax(max.z) {}
};

#endif // BOX_H_