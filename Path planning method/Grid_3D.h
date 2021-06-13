#pragma once
#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>
#include<vector>
#include <math.h>
#include <time.h>
#include <iomanip>
#include"m_TimeKeeping.h"
#include"AStar.h"
#include <iterator>
#include <fstream>
#pragma warning(disable:4996)
typedef unsigned __int64 uint64;
typedef unsigned int uint32;


const int path_N = 6;
using namespace std;


//【点的三维坐标】
struct Coor_XYZ
{
	Coor_XYZ(double _x = 0.0, double _y = 0.0, double _z = 0.0) :X(_x), Y(_y), Z(_z){}
	double X;
	double Y;
	double Z;
};


//【网格的IJK坐标】
struct Coor_IJK{
	Coor_IJK(unsigned int _i = 0, unsigned int _j = 0, unsigned int _k = 0) :I(_i), J(_j), K(_k){}
	unsigned int I : 21;
	unsigned int J : 21;
	unsigned int K : 21;
	unsigned int Mode : 1;  //0,1表示是否含有数据
};

//【三角形】
struct Triangle
{
	Coor_XYZ angle[3];
};

class C3D_GriddingCode
{
public:
	C3D_GriddingCode(void);
	~C3D_GriddingCode(void);

//==================【初始化网格剖分范围】====================
	void initArea(double minx, double miny, double minz, double maxx, double maxy, double maxz);

//==================【场景网格化】====================
	//【点要素】
	uint64 toCode_fromXYZ(Coor_XYZ p, int N);//单点
	//【面要素】
	void triangleVoxelization(vector<Triangle>input, int N, vector<Coor_IJK>&output);//三角网体素化
	void ScanLineSeedFill_3(vector<Coor_IJK>& outer);//三维扫描线种子填充算法（反向填充）


//==================【无人机路径规划】====================
	//用网格模型对地图进行填充
	void initFlyArea(vector<Coor_IJK>&Area, vector<Coor_IJK>data);
	//初始化【格点化】地图
	void init_GridPoint_Map(vector<Coor_IJK>Area, vector<vector<vector<bool>>>& maze);
	//单无人机规划(格点化地图)
	void sin_pathFinding_GridPoint(vector<vector<vector<bool>>> maze, vector<Point>UAV, vector<Point>&pathPoint, bool isFlyRestrict);



//==================【输出】====================
	//输出格点轨迹
	void out_vtk_pointpath(vector<Point>input, char filename[20]);
	void out_vtk_pointpath2(vector<Point>input, char filename[20]);//二维





private:
//==================【基础函数】====================
	//网格坐标和经纬度坐标的相互转换
	Coor_XYZ toXYZ_fromIJK(Coor_IJK p, int N);
	Coor_IJK toIJK_fromXYZ(Coor_XYZ p, int N);//单点
 
};