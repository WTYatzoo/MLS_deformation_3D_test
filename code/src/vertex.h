#ifndef _VERTEX_
#define _VERTEX_

#include <math.h>
#include "myvector.h"
class vertex
{
 public:
  myvector location;
  myvector location_deform;
  int index_tet[4];
  vertex();
  vertex(myvector location);
  ~vertex();
};

#endif
