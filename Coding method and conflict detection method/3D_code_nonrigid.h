// 下列 ifdef 块是创建使从 DLL 导出更简单的
// 宏的标准方法。此 DLL 中的所有文件都是用命令行上定义的 GRIDDLL_EXPORTS
// 符号编译的。在使用此 DLL 的
// 任何其他项目上不应定义此符号。这样，源文件中包含此文件的任何其他项目都会将
// GRIDDLL_API 函数视为是从 DLL 导入的，而此 DLL 则将用此宏定义的
// 符号视为是被导出的。
#ifdef GRIDDLL_EXPORTS
#define GRIDDLL_API __declspec(dllexport)
#else
#define GRIDDLL_API __declspec(dllimport)
#endif

__declspec(dllexport) void nothing(void);

void nothing(void)
{
}



#pragma once
#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>
#include<vector>
#include <math.h>
#include <time.h>
#include <iomanip>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include"m_TimeKeeping.h"
#pragma warning(disable:4996)
typedef unsigned __int64 uint64;
typedef unsigned int uint32;

#define pi 3.141592653589793238462643383279 //圆周率
#define R_earth 6371004//地球平均半径
const int dad_n = 3;//查找父单元向上层级数
using namespace std;

#define random(a,b) double(rand()%(100*(b-a+1)))/100 + a//随机数
#define random2(a,b) double(rand()%((b-a+1))) + a//随机数

//预先定义数据经纬度和高程的范围
const double Bmin = 0;
const double Bmax = 10000;
const double Lmin = 0;
const double Lmax = 10000;
const double Hmin = 0;
const double Hmax = 10000;

const int path_N = 7;

//【点的三维坐标】
struct Coor_XYZ
{
	Coor_XYZ(double _x = 0.0, double _y = 0.0, double _z = 0.0) :X(_x), Y(_y), Z(_z){}
	double X;
	double Y;
	double Z;
	int T;
};

struct info
{
	uint64 ID;
	int T;//时间
};


//【网格的IJK坐标】
struct Coor_IJK{
	Coor_IJK(unsigned int _i = 0, unsigned int _j = 0, unsigned int _k = 0) :I(_i), J(_j), K(_k){}
	unsigned int I;    // 0-1023，秒以下
	unsigned int J;    // 0-59秒
	unsigned int K;    // 0-59分
	unsigned int Mode;  //0,1表示是否含有数据

	unsigned int A = 0;
	unsigned int B = 0;
	info info1;

	vector<info>infoSet_A;
	vector<info>infoSet_B;
};





//【子单元范围】
struct GBorder
{
	uint64 front;
	uint64 end;
};

struct line
{
	
	vector<Coor_XYZ>points;
};
struct pointline
{
	int ID;
	vector<Coor_IJK>points;
};

//【无人机属性】
struct UAV
{
	Coor_XYZ coor;//三维坐标
	double size;//无人机尺寸
	uint64 ID;//标识编码
};


class Nonrigid
{
public:
	Nonrigid();
	~Nonrigid();


//【1】编码代数基础
	//坐标 + 层级 = 编码
	uint64 toCodefromIJKandN(Coor_IJK P, int N);//由网格坐标计算
	uint64 toCodefromCodeIJKandN(Coor_IJK P, int N);//由编码坐标计算
	//编码转网格坐标
	Coor_IJK toIJKfromCode(uint64 Code);
	Coor_IJK toCodeIJKfromCode(uint64 Code);
	//计算编码层级
	int toNfromCode(uint64 Code);
	//父单元
	uint64 FindDad_RigidCode(uint64 Code, int N);//空间网格父单元
	uint64 FindDad_Code(uint64 Code, int N);//编码父单元
	uint64 FindDad_NonRigidCode(uint64 Code);//非刚性父单元
	//子单元
	void FindSon_Code(uint64 Code, int N, vector<uint64>&SonCodes);//指定层级子单元编码集合
	void FindSon_CodeFanwei(uint64 Code, GBorder& border);//基础层级子单元查询(范围)
	//网格邻域计算
	void Find_Neighbourhood_26(uint64 Code, vector<uint64> &codelist);//26邻域
	void Find_Neighbourhood_6(uint64 Code, vector<uint64> &codelist);//6邻域
	//经纬度坐标转编码
	uint64 toCode_fromXYZ(Coor_XYZ p, int N);
	//经纬度坐标转网格坐标
	Coor_IJK toIJK_fromXYZ(Coor_XYZ p, int N);
	//经纬度坐标转格点坐标
	Coor_IJK toCodeIJK_fromXYZ(Coor_XYZ p, int N);


//【2】直线的格点化
	void toGridpoint_fromLine(Coor_XYZ p1, Coor_XYZ p2, int N, vector<Coor_IJK>&pointlist);
	void toGridpoint_fromLine(Coor_XYZ p1, Coor_XYZ p2, int N, vector<uint64>&codelist);

//【3】冲突检测实验
	void test_conflict(int amount);


private:


	//======================================【应用】==================================================
	//【1】输出包围盒的唯一ID
	uint64 toIntID(Coor_XYZ leftdown, Coor_XYZ rightup);
	uint64 toRigid_IntID(Coor_XYZ leftdown, Coor_XYZ rightup);//刚性标识

	//【3】直线的网格化
	void toGrid_fromLine(Coor_XYZ p1, Coor_XYZ p2, int N, vector<uint64>&codelist);


	//======================================【大论文实验】==================================================
	//【1】模拟航迹
	void simulate_Paths(int amount, vector<Coor_XYZ>&paths);
	void simulate_Paths_2(int num, vector<vector<Coor_XYZ>>lines);

	//【2】直线格点化实验
	void test_gridding_line(vector<Coor_XYZ>paths);
	double toD_fromGrid(Coor_XYZ p1, Coor_XYZ p2, vector<uint64>codelist);
	double toD_fromGridPoint(Coor_XYZ p1, Coor_XYZ p2, vector<uint64>codelist);


	double getDistance_from_line_line(Coor_XYZ A, Coor_XYZ B, Coor_XYZ C, Coor_XYZ D);//计算两直线间距离
	double getDistance_from_point_line(Coor_XYZ C, Coor_XYZ A, Coor_XYZ B);
	double getDistance_from_point_point(Coor_XYZ A, Coor_XYZ B);
	double getDistance_from_xianduan_xianduan(Coor_XYZ A, Coor_XYZ B, Coor_XYZ C, Coor_XYZ D);



	void coor_of_code(uint64 code, Coor_XYZ &center);//计算网格格点中心坐标
	void rigid_coor_of_code(uint64 code, Coor_XYZ &center);

	//======================================【校验】==================================================
	// 判断网格坐标是否越界，越界后修改
	bool isOutofIJKBorder(Coor_IJK& newIJK, Coor_IJK& origin, int N);
	//经纬度高程坐标是否是真实地理范围
	bool isOutofXYZBorder(Coor_XYZ p);
	//编码去重
	void code_unique(vector<uint64>& vCode);

	//返回集合中的重复数量
	int repeat(vector<int>code);

	//======================================【原刚性】==================================================
	//判断空间点是否在刚性网格中
	bool isPointinGrid(Coor_XYZ P, uint64 code);

	//父单元计算
	uint64 rigid_FindDadCode(uint64 Code, int N);
	//子单元范围
	void rigid_FindSon_CodeFanwei(uint64 Code, GBorder& border);
	//计算编码层级
	int rigid_toNfromCode(uint64 Code);
	//经纬度坐标转编码
	uint64 rigid_toCode_fromXYZ(Coor_XYZ p, int N);
	//经纬度坐标转IJK
	Coor_IJK rigid_toIJK_fromXYZ(Coor_XYZ p, int N);
	//坐标 + 层级 = 编码
	uint64 rigid_toCodefromIJKandN(Coor_IJK P, int N);
	//编码转网格坐标
	Coor_IJK rigid_toIJKfromCode(uint64 Code);

	//根据多尺度整数编码，计算对应格网的八个角点坐标
	void CountCoord(uint64 Inputcode, vector<Coor_XYZ>&result);

};



