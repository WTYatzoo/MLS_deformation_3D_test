#ifndef _OBJECT_
#define _OBJECT_

#include <vector>
#include "vertex.h"
#include "testface.h"
using namespace std;

class object
{
public:
	vector<vertex > myvertexs;
	vector<testface > mytestfaces;

	int num_vertex;
	int num_face;

	object();
	~object();
	void getObjData();
	void testdraw();
        void deform(vector< vertex >& myvertexs,double h, vector<pair< myvector, myvector> >& controlPointsPair);
};
#endif
