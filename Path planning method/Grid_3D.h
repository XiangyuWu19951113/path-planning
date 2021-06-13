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


//�������ά���꡿
struct Coor_XYZ
{
	Coor_XYZ(double _x = 0.0, double _y = 0.0, double _z = 0.0) :X(_x), Y(_y), Z(_z){}
	double X;
	double Y;
	double Z;
};


//�������IJK���꡿
struct Coor_IJK{
	Coor_IJK(unsigned int _i = 0, unsigned int _j = 0, unsigned int _k = 0) :I(_i), J(_j), K(_k){}
	unsigned int I : 21;
	unsigned int J : 21;
	unsigned int K : 21;
	unsigned int Mode : 1;  //0,1��ʾ�Ƿ�������
};

//�������Ρ�
struct Triangle
{
	Coor_XYZ angle[3];
};

class C3D_GriddingCode
{
public:
	C3D_GriddingCode(void);
	~C3D_GriddingCode(void);

//==================����ʼ�������ʷַ�Χ��====================
	void initArea(double minx, double miny, double minz, double maxx, double maxy, double maxz);

//==================���������񻯡�====================
	//����Ҫ�ء�
	uint64 toCode_fromXYZ(Coor_XYZ p, int N);//����
	//����Ҫ�ء�
	void triangleVoxelization(vector<Triangle>input, int N, vector<Coor_IJK>&output);//���������ػ�
	void ScanLineSeedFill_3(vector<Coor_IJK>& outer);//��άɨ������������㷨��������䣩


//==================�����˻�·���滮��====================
	//������ģ�ͶԵ�ͼ�������
	void initFlyArea(vector<Coor_IJK>&Area, vector<Coor_IJK>data);
	//��ʼ������㻯����ͼ
	void init_GridPoint_Map(vector<Coor_IJK>Area, vector<vector<vector<bool>>>& maze);
	//�����˻��滮(��㻯��ͼ)
	void sin_pathFinding_GridPoint(vector<vector<vector<bool>>> maze, vector<Point>UAV, vector<Point>&pathPoint, bool isFlyRestrict);



//==================�������====================
	//������켣
	void out_vtk_pointpath(vector<Point>input, char filename[20]);
	void out_vtk_pointpath2(vector<Point>input, char filename[20]);//��ά





private:
//==================������������====================
	//��������;�γ��������໥ת��
	Coor_XYZ toXYZ_fromIJK(Coor_IJK p, int N);
	Coor_IJK toIJK_fromXYZ(Coor_XYZ p, int N);//����
 
};