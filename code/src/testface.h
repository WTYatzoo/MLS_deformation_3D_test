#ifndef _TESTFACE_
#define _TESTFACE_

class testface
{
public:
	int index_vertex[3];

	testface()
	{
		;
	}
	
	testface(int index1,int index2,int index3)
	{
		index_vertex[0]=index1; index_vertex[1]=index2; index_vertex[2]=index3;
	}

	~testface()
	{

	}
};
#endif