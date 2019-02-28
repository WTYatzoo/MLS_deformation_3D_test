#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "vertex.h"
using namespace std;

vertex::vertex()
{
  int i;
  for(i=0;i<4;i++)
    {
      this->index_tet[i]=-1;
    }
}

vertex::vertex(myvector location)
{
  this->location=location;
  int i;
  for(i=0;i<4;i++)
    {
      this->index_tet[i]=-1;
    }
}

vertex::~vertex()
{
  ;
}

