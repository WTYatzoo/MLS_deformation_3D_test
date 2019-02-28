#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <map>
#include <GL/glut.h>
#include <float.h>
#include "object.h"
#include "grid.h"
#include "vertex_grid.h"

#include </usr/include/eigen3/Eigen/Eigen>
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/Cholesky>
#include </usr/include/eigen3/Eigen/LU>
#include </usr/include/eigen3/Eigen/Sparse>
#include </usr/include/eigen3/Eigen/SparseQR>
#include </usr/include/eigen3/Eigen/SparseLU>
#include </usr/include/eigen3/Eigen/IterativeLinearSolvers>

using namespace std;
using namespace Eigen;

#define min(a,b) (((a) < (b)) ? (a) : (b))

static int vertex_collection[2][5][4][3]={
  { 
    {
      {
	0,0,0
	  },
	{
	  1,1,0
	    },
	  {
	    0,1,1
	      },
	    { 
	      1,0,1
		}
    },
      {
	{
	  0,1,1
	    },
	  {
	    1,0,1
	      },
	    {
	      1,1,1
		},
	      {
		1,1,0
		  }
      },
	{
	  {
	    0,0,0
	      },
	    {
	      0,0,1
		},
	      {
		0,1,1
		  },
		{
		  1,0,1
		    }
	},
	  {
	    {
	      0,0,0
		},
	      {
		1,1,0
		  },
		{
		  0,1,0
		    },
		  {
		    0,1,1
		      }
	  },
	    {
	      {
		0,0,0
		  },
		{
		  1,0,0
		    },
		  {
		    1,1,0
		      },
		    {
		      1,0,1
			}
	    }
  },
    { 
      {
	{
	  0,0,1
	    },
	  {
	    1,1,1
	      },
	    {
	      0,1,0
		},
	      {
		1,0,0	
		  }
      },
	{
	  {
	    0,0,1
	      },
	    {
	      0,1,1
		},
	      {
		1,1,1
		  },
		{
		  0,1,0
		    }
	},
	  {
	    {
	      0,0,1
		},
	      {
		1,0,1
		  },
		{
		  1,1,1
		    },
		  {
		    1,0,0
		      }
	  },
	    {
	      {
		0,0,0
		  },
		{
		  0,1,0
		    },
		  {
		    1,0,0
		      },
		    {
		      0,0,1
			}
	    },
	      {
		{
		  1,0,0
		    },
		  {
		    0,1,0
		      },
		    {
		      1,1,0
			},
		      {
			1,1,1
			  }
	      }
    }
};

object::object()
{
  getObjData();
  // testdraw();
  
  double h;
  vector<pair<myvector,myvector> > controlPointsPair;

  {
    //torus.txt
    /*
      h=0.1;
      controlPointsPair.push_back(make_pair(myvector(-1.25,-0.25,-1.25),myvector(-1.25,-0.25,-4)));
      controlPointsPair.push_back(make_pair(myvector(0,0,1.25),myvector(0,0,1.6)));
      controlPointsPair.push_back(make_pair(myvector(0,0,-1.25),myvector(0,0,-1.6)));
      controlPointsPair.push_back(make_pair(myvector(-1.25,0,0),myvector(-1.25,0,0)));
    */
  }

  {
    //homer.txt
    h=0.1;

//   controlPointsPair.push_back(make_pair(myvector(0,0,0),myvector(1,1,1)));

    /*controlPointsPair.push_back(make_pair(myvector(-0.28,0.06,0.13),myvector(-0.28,0.06,0.13)));
    controlPointsPair.push_back(make_pair(myvector(0.28,0.06,0.13),myvector(0.28,0.06,0.13)));
    controlPointsPair.push_back(make_pair(myvector(0,0,0),myvector(0,0,0)));
    controlPointsPair.push_back(make_pair(myvector(0,0.5,0),myvector(0,0.5,0)));
     controlPointsPair.push_back(make_pair(myvector(0.08,-0.5,0),myvector(0.32,-0.22,0)));*/

    controlPointsPair.push_back(make_pair(myvector(-0.28,0.06,0.13),myvector(-0.28,0.06,0.13)));
    controlPointsPair.push_back(make_pair(myvector(0.28,0.06,0.13),myvector(0.28,0.06,0.13)));
    controlPointsPair.push_back(make_pair(myvector(0,0,0),myvector(0,0,0)));
    controlPointsPair.push_back(make_pair(myvector(0,0.5,0),myvector(0,0.5,0.3)));
    controlPointsPair.push_back(make_pair(myvector(0.28,-0.5,0),myvector(0.4,-0.22,0)));
    
    controlPointsPair.push_back(make_pair(myvector(0.28,-0.5,0.05),myvector(0.4,-0.22,0.05)));
    controlPointsPair.push_back(make_pair(myvector(0.28,-0.5,0.1),myvector(0.4,-0.22,0.1)));
    controlPointsPair.push_back(make_pair(myvector(0.28,-0.5,0.15),myvector(0.4,-0.22,0.15)));
    controlPointsPair.push_back(make_pair(myvector(0.28,-0.5,-0.05),myvector(0.4,-0.22,-0.05)));
    controlPointsPair.push_back(make_pair(myvector(0.28,-0.5,-0.1),myvector(0.4,-0.22,-0.1)));
  //  controlPointsPair.push_back(make_pair(myvector(0.28,-0.5,-0.15),myvector(0.4,-0.22,-0.15)));  
  }

  deform(myvertexs,h,controlPointsPair);
  
}

object::~object()
{
	
}

void  object::testdraw()
{
  int i;
  int siz=mytestfaces.size();
  testface now;
  glColor3d(0,1,1);
  for(i=0;i<siz;i++)
    {
      now=mytestfaces[i];
      glBegin(GL_POLYGON);
      {
	/*	glVertex3d(myvertexs[now.index_vertex[0]].location.x,myvertexs[now.index_vertex[0]].location.y,myvertexs[now.index_vertex[0]].location.z);
		glVertex3d(myvertexs[now.index_vertex[1]].location.x,myvertexs[now.index_vertex[1]].location.y,myvertexs[now.index_vertex[1]].location.z);
		glVertex3d(myvertexs[now.index_vertex[2]].location.x,myvertexs[now.index_vertex[2]].location.y,myvertexs[now.index_vertex[2]].location.z);
	*/
	glVertex3d(myvertexs[now.index_vertex[0]].location_deform.x,myvertexs[now.index_vertex[0]].location_deform.y,myvertexs[now.index_vertex[0]].location_deform.z);
	glVertex3d(myvertexs[now.index_vertex[1]].location_deform.x,myvertexs[now.index_vertex[1]].location_deform.y,myvertexs[now.index_vertex[1]].location_deform.z);
	glVertex3d(myvertexs[now.index_vertex[2]].location_deform.x,myvertexs[now.index_vertex[2]].location_deform.y,myvertexs[now.index_vertex[2]].location_deform.z);
        
      }
      glEnd();
    }
}

void object::getObjData()
{

  string name="/home/wtyatzoo/project/model/homer.txt";
  FILE* file=fopen(name.c_str(),"r");
  fscanf(file,"%d%d",&num_vertex,&num_face);
  printf("%d %d \n",num_vertex,num_face);
	
  int i,j,k;
  double x,y,z;
  vertex here;
  char  filter[5];
  for(i=0;i<num_vertex;i++)
    {
      fscanf(file,"%s%lf%lf%lf",filter,&x,&y,&z);
      myvertexs.push_back(vertex(myvector(x,y,z)));
    }
	
  int index[3];
  for(i=0;i<num_face;i++)
    {
      fscanf(file,"%s%d%d%d",filter,&index[0],&index[1],&index[2]);
      for(j=0;j<3;j++)
	{
	  index[j]--;
	}
      mytestfaces.push_back(testface(index[0],index[1],index[2]));
    }
}
void object::deform(vector< vertex >& myvertexs,double h, vector<pair< myvector, myvector> >& controlPointsPair)
{
  double max_min_xyz[3][2];
  double len[3];
  int div[3];
  max_min_xyz[0][0]=max_min_xyz[0][1]=myvertexs[0].location.x;
  max_min_xyz[1][0]=max_min_xyz[1][1]=myvertexs[0].location.y;
  max_min_xyz[2][0]=max_min_xyz[2][1]=myvertexs[0].location.z;
  
  int num_vertex=myvertexs.size();
  int i,j,k;
  for(i=0;i<num_vertex;i++)
    {
      vertex vertex_now=myvertexs[i];
      if(vertex_now.location.x>max_min_xyz[0][0])
	{
	  max_min_xyz[0][0]=vertex_now.location.x;
	}
      else if(vertex_now.location.x<max_min_xyz[0][1])
	{
	  max_min_xyz[0][1]=vertex_now.location.x;
	}
      if(vertex_now.location.y>max_min_xyz[1][0])
	{
	  max_min_xyz[1][0]=vertex_now.location.y;
	}
      else if(vertex_now.location.y<max_min_xyz[1][1])
	{
	  max_min_xyz[1][1]=vertex_now.location.y;
	}
      if(vertex_now.location.z>max_min_xyz[2][0])
	{
	  max_min_xyz[2][0]=vertex_now.location.z;
	}
      else if(vertex_now.location.z<max_min_xyz[2][1])
	{
	  max_min_xyz[2][1]=vertex_now.location.z;
	}
    }
  for(i=0;i<3;i++)
    {
      len[i]=max_min_xyz[i][0]-max_min_xyz[i][1];
      div[i]=floor(len[i]/h)+1;
      printf("max_min_xyz[%d] %lf %lf\n",i,max_min_xyz[i][0],max_min_xyz[i][1]);
    }

  printf("div %d %d %d \n",div[0],div[1],div[2]);

  int div_vertex[3];
  for(i=0;i<3;i++)
    {
      div_vertex[i]=div[i]+1;
    }

  int*** index_vertex_grid=(int***) new int**[div_vertex[0]];
  for(i=0;i<div_vertex[0];i++)
    {
      index_vertex_grid[i]=(int**) new int*[div_vertex[1]];
      for(j=0;j<div_vertex[1];j++)
	{
	  index_vertex_grid[i][j]=new int[div_vertex[2]];
	}
    }
  int index_now=0;
  vector< vertex_grid > myvertexsOfGrid;
  myvector location_now;
  for(i=0;i<div_vertex[0];i++)
    {
      for(j=0;j<div_vertex[1];j++)
	{
	  for(k=0;k<div_vertex[2];k++)
	    {
	      location_now=myvector(max_min_xyz[0][1]+i*h,max_min_xyz[1][1]+j*h,max_min_xyz[2][1]+k*h);
	      myvertexsOfGrid.push_back(vertex_grid(location_now));
	      index_vertex_grid[i][j][k]=index_now;
	      index_now++;
	    }
	}
    }
  int num_vertexOfGrid=myvertexsOfGrid.size();

  printf("%d num_vertexOfGrid\n",num_vertexOfGrid);
  grid*** mygrid=(grid***) new grid**[div[0]];
  
  for(i=0;i<div[0];i++)
    {
      mygrid[i]=(grid**) new grid*[div[1]];
      for(j=0;j<div[1];j++)
	{
	  mygrid[i][j]=new grid[div[2]];
	}
    }
  
  int a,b,c;
  int kind=0;
  int kind_out=0;
  int kind_out_out=0;
  for(i=0;i<div[0];i++)
    {
      kind_out=kind_out_out;
      for(j=0;j<div[1];j++)
	{
	  kind=kind_out;
	  for(k=0;k<div[2];k++)
	    {
	      mygrid[i][j][k].kind=kind;
	      for(a=0;a<2;a++)
		{
		  for(b=0;b<2;b++)
		    {
		      for(c=0;c<2;c++)
			{
			  mygrid[i][j][k].index_vertex[a][b][c]=index_vertex_grid[i+a][j+b][k+c];   
			}
		    }
		}
	      kind=!kind;
	    }
	  kind_out=!kind_out;
	}
      kind_out_out=!kind_out_out;
    }
  int xx,yy,zz;
  for(i=0;i<div[0];i++)
    {
      for(j=0;j<div[1];j++)
	{
	  for(k=0;k<div[2];k++)
	    {
	      int kind_now = mygrid[i][j][k].kind;
	      for(a=0;a<5;a++)
		{
		  for(b=0;b<4;b++)
		    {
		      xx=vertex_collection[kind_now][a][b][0];
		      yy=vertex_collection[kind_now][a][b][1];
		      zz=vertex_collection[kind_now][a][b][2];
		      mygrid[i][j][k].mytet[a].index_vertex[b]=mygrid[i][j][k].index_vertex[xx][yy][zz];
		      
		    }
		}
	    }
	}
    }


  {
    FILE* fp=fopen("./res/mytet_ori.vtk","w");
    fprintf(fp,"# vtk DataFile Version 2.0\n");
    fprintf(fp,"tetra\n");
    fprintf(fp,"ASCII\n");
    fprintf(fp,"DATASET UNSTRUCTURED_GRID\n");
    fprintf(fp,"POINTS %d double\n",num_vertexOfGrid);
    for(i=0;i<num_vertexOfGrid;i++)
      {
	fprintf(fp,"%lf %lf %lf\n",myvertexsOfGrid[i].location.x,myvertexsOfGrid[i].location.y,myvertexsOfGrid[i].location.z);
      }

    int num_grid=div[0]*div[1]*div[2];
    fprintf(fp,"CELLS %d %d\n",5*num_grid,25*num_grid);

    for(i=0;i<div[0];i++)
      {
	for(j=0;j<div[1];j++)
	  {
	    for(k=0;k<div[2];k++)
	      {
		for(a=0;a<5;a++)
		  {
		    fprintf(fp,"4");
		    for(b=0;b<4;b++)
		      {
			fprintf(fp," %d",mygrid[i][j][k].mytet[a].index_vertex[b]);
		      }
		    fprintf(fp,"\n");
		  }
	      }
	  }
      }

    fprintf(fp,"CELL_TYPES %d\n",5*num_grid);
    for(i=0;i<5*num_grid;i++)
      {
	fprintf(fp,"10\n");
      }
    fclose(fp);
  }

  int which_grid[3];
  for(i=0;i<num_vertex;i++)
    {
      location_now=myvertexs[i].location;
      which_grid[0]=floor((location_now.x-max_min_xyz[0][1])/h);
      which_grid[1]=floor((location_now.y-max_min_xyz[1][1])/h);
      which_grid[2]=floor((location_now.z-max_min_xyz[2][1])/h);

      tetrahedron tet_now;
      MatrixXd matrix_test=MatrixXd::Random(4,4);
      int mark=-2;
      int mark_now=-2;
      
      bool find;
      
      for(j=0;j<5;j++)
	{
	  find = true;
	  tet_now=mygrid[which_grid[0]][which_grid[1]][which_grid[2]].mytet[j];
	  
	  for(a=0;a<4;a++)
	    {
	      matrix_test(a,0)=myvertexsOfGrid[tet_now.index_vertex[a]].location.x;
	      matrix_test(a,1)=myvertexsOfGrid[tet_now.index_vertex[a]].location.y;
	      matrix_test(a,2)=myvertexsOfGrid[tet_now.index_vertex[a]].location.z;
	      matrix_test(a,3)=1;
	    }
	  mark=(matrix_test.determinant()>0?1:-1);

	  for(a=0;a<4;a++)
	    {
	      matrix_test(a,0)=location_now.x;
	      matrix_test(a,1)=location_now.y;
	      matrix_test(a,2)=location_now.z;

	      double det=matrix_test.determinant();

	      matrix_test(a,0)=myvertexsOfGrid[tet_now.index_vertex[a]].location.x;
	      matrix_test(a,1)=myvertexsOfGrid[tet_now.index_vertex[a]].location.y;
	      matrix_test(a,2)=myvertexsOfGrid[tet_now.index_vertex[a]].location.z;
	       
	      if(abs(det) <= 1e-15)
		{
		  continue;
		}
	      mark_now=(det>0?1:-1);
	      if(mark==mark_now)
		{
		  continue;
		}
	      else
		{
		  find=false;
		  break;
		}
	    }
	  if(find==true)
	    {
	      break;
	    }
	}
      myvertexs[i].index_tet[0]=which_grid[0];
      myvertexs[i].index_tet[1]=which_grid[1];
      myvertexs[i].index_tet[2]=which_grid[2];
      if(find==true)
	{
	  myvertexs[i].index_tet[3]=j;
	}
    }

  //deform vertexOfGrid
  vertex_grid vertexOfGrid_now;
  int siz_pair=controlPointsPair.size();

  printf("siz_pair %d \n",siz_pair);

  for(i=0;i<siz_pair;i++)
    {
      printf("%lf %lf %lf first\n",controlPointsPair[i].first.x,controlPointsPair[i].first.y,controlPointsPair[i].first.z);
      printf("%lf %lf %lf second\n",controlPointsPair[i].second.x,controlPointsPair[i].second.y,controlPointsPair[i].second.z);
    }
  
  double* w=new double[siz_pair];
  double alpha=1;

  for(i=0;i<num_vertexOfGrid;i++)
    {
      vertexOfGrid_now=myvertexsOfGrid[i];
      double sum_w=0;

      myvector vector_pc=myvector(0,0,0);
      myvector vector_qc=myvector(0,0,0);
      myvector vector_pi=myvector(0,0,0);
      myvector vector_qi=myvector(0,0,0);

      
      // 需要判断是否当前被移动的点是一个control point,如果当前点就是一个control point 那么直接映射成对应的qi
      int  is_control_point=0;
      for(j=0;j<siz_pair;j++)
	{
	  w[j]=myvector(vertexOfGrid_now.location-controlPointsPair[j].first).len_sq();
	  w[j]=pow(w[j],-1*alpha);

	  if(isinf((float)w[j])!=0)
	    {
	      is_control_point=1;
	      break;
	    }
	  vector_pc+=(w[j]*controlPointsPair[j].first);
	  vector_qc+=(w[j]*controlPointsPair[j].second);
	  sum_w+=w[j];
	}
      if(is_control_point==1)
	{
	  myvertexsOfGrid[i].location_deform=controlPointsPair[j].second;  
	  continue;
	}
      vector_pc/=sum_w;
      vector_qc/=sum_w;

      MatrixXd pc=MatrixXd::Random(3,1);
      MatrixXd qc=MatrixXd::Random(3,1);
      pc(0,0)=vector_pc.x; pc(1,0)=vector_pc.y; pc(2,0)=vector_pc.z;
      qc(0,0)=vector_qc.x; qc(1,0)=vector_qc.y; qc(2,0)=vector_qc.z;
      MatrixXd pi=MatrixXd::Random(3,1);
      MatrixXd qi=MatrixXd::Random(3,1);

      MatrixXd PQ=MatrixXd::Random(3,3);
      PQ.fill(0);
      for(j=0;j<siz_pair;j++)
	{
	  vector_pi=controlPointsPair[j].first;
	  vector_qi=controlPointsPair[j].second;
	  pi(0,0)=vector_pi.x; pi(1,0)=vector_pi.y; pi(2,0)=vector_pi.z;
	  qi(0,0)=vector_qi.x; qi(1,0)=vector_qi.y; qi(2,0)=vector_qi.z;
	  PQ+= w[j]*(pi-pc)*((qi-qc).transpose());
	  
	}
      JacobiSVD<MatrixXd> svd(PQ, ComputeThinU | ComputeThinV);
      MatrixXd M=MatrixXd::Random(3,3);
      M=svd.matrixV()*(svd.matrixU().transpose());

      MatrixXd loc=MatrixXd::Random(3,1);
      MatrixXd loc_deform=MatrixXd::Random(3,1);
      loc(0,0)=vertexOfGrid_now.location.x; loc(1,0)=vertexOfGrid_now.location.y; loc(2,0)=vertexOfGrid_now.location.z;
      loc_deform=M*(loc-pc)+qc;
      myvertexsOfGrid[i].location_deform=myvector(loc_deform(0,0),loc_deform(1,0),loc_deform(2,0));

    }
 
  {
    FILE* fp=fopen("./res/mytet_deform.vtk","w");
    fprintf(fp,"# vtk DataFile Version 2.0\n");
    fprintf(fp,"tetra\n");
    fprintf(fp,"ASCII\n");
    fprintf(fp,"DATASET UNSTRUCTURED_GRID\n");
    fprintf(fp,"POINTS %d double\n",num_vertexOfGrid);
    for(i=0;i<num_vertexOfGrid;i++)
      {
	fprintf(fp,"%lf %lf %lf\n",myvertexsOfGrid[i].location_deform.x,myvertexsOfGrid[i].location_deform.y,myvertexsOfGrid[i].location_deform.z);
      }

    int num_grid=div[0]*div[1]*div[2];
    fprintf(fp,"CELLS %d %d\n",5*num_grid,25*num_grid);

    for(i=0;i<div[0];i++)
      {
	for(j=0;j<div[1];j++)
	  {
	    for(k=0;k<div[2];k++)
	      {
		for(a=0;a<5;a++)
		  {
		    fprintf(fp,"4");
		    for(b=0;b<4;b++)
		      {
			fprintf(fp," %d",mygrid[i][j][k].mytet[a].index_vertex[b]);
		      }
		    fprintf(fp,"\n");
		  }
	      }
	  }
      }

    fprintf(fp,"CELL_TYPES %d\n",5*num_grid);
    for(i=0;i<5*num_grid;i++)
      {
	fprintf(fp,"10\n");
      }
    fclose(fp);
  }
  

  int index_tet[4];
  int index_vertex[4];
  myvector loc_need[3];
  myvector loc_deform_need[3];
  MatrixXd m_ori=MatrixXd::Random(3,3);
  MatrixXd m_def=MatrixXd::Random(3,3);

  MatrixXd loc_ori=MatrixXd::Random(3,1);
  MatrixXd loc_def=MatrixXd::Random(3,1);
  for(i=0;i<num_vertex;i++)
    {
      for(j=0;j<4;j++)
	{
	  index_tet[j]=myvertexs[i].index_tet[j];
	}

      //这层循环要写在这里，写在if里面则逻辑错误
      for(j=0;j<4;j++)
	{
	  index_vertex[j]=mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].index_vertex[j];
	}
      
      if(mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].get_P==0)
	{ 
	  for(j=1;j<4;j++)
	    { 
	      loc_need[j-1]=myvertexsOfGrid[index_vertex[j]].location-myvertexsOfGrid[index_vertex[0]].location;
	      loc_deform_need[j-1]=myvertexsOfGrid[index_vertex[j]].location_deform-myvertexsOfGrid[index_vertex[0]].location_deform;
	    }
	  for(j=0;j<3;j++)
	    {
	      m_ori(0,j)=loc_need[j].x; m_ori(1,j)=loc_need[j].y; m_ori(2,j)=loc_need[j].z;
	      m_def(0,j)=loc_deform_need[j].x; m_def(1,j)=loc_deform_need[j].y; m_def(2,j)=loc_deform_need[j].z;
	    }
	  mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].P=m_def*(m_ori.inverse());

	  mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].get_P=1;
	}
     

      //  printf("get_P %d\n",mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].get_P);
      
      loc_ori(0,0)=myvertexs[i].location.x-myvertexsOfGrid[index_vertex[0]].location.x;
      loc_ori(1,0)=myvertexs[i].location.y-myvertexsOfGrid[index_vertex[0]].location.y;
      loc_ori(2,0)=myvertexs[i].location.z-myvertexsOfGrid[index_vertex[0]].location.z;
      
      loc_def=mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].P*loc_ori;
      
      myvertexs[i].location_deform.x=loc_def(0,0)+myvertexsOfGrid[index_vertex[0]].location_deform.x;
      myvertexs[i].location_deform.y=loc_def(1,0)+myvertexsOfGrid[index_vertex[0]].location_deform.y;
      myvertexs[i].location_deform.z=loc_def(2,0)+myvertexsOfGrid[index_vertex[0]].location_deform.z;
    }
  
}



