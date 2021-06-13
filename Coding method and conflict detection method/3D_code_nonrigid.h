// ���� ifdef ���Ǵ���ʹ�� DLL �������򵥵�
// ��ı�׼�������� DLL �е������ļ��������������϶���� GRIDDLL_EXPORTS
// ���ű���ġ���ʹ�ô� DLL ��
// �κ�������Ŀ�ϲ�Ӧ����˷��š�������Դ�ļ��а������ļ����κ�������Ŀ���Ὣ
// GRIDDLL_API ������Ϊ�Ǵ� DLL ����ģ����� DLL ���ô˺궨���
// ������Ϊ�Ǳ������ġ�
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

#define pi 3.141592653589793238462643383279 //Բ����
#define R_earth 6371004//����ƽ���뾶
const int dad_n = 3;//���Ҹ���Ԫ���ϲ㼶��
using namespace std;

#define random(a,b) double(rand()%(100*(b-a+1)))/100 + a//�����
#define random2(a,b) double(rand()%((b-a+1))) + a//�����

//Ԥ�ȶ������ݾ�γ�Ⱥ͸̵߳ķ�Χ
const double Bmin = 0;
const double Bmax = 10000;
const double Lmin = 0;
const double Lmax = 10000;
const double Hmin = 0;
const double Hmax = 10000;

const int path_N = 7;

//�������ά���꡿
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
	int T;//ʱ��
};


//�������IJK���꡿
struct Coor_IJK{
	Coor_IJK(unsigned int _i = 0, unsigned int _j = 0, unsigned int _k = 0) :I(_i), J(_j), K(_k){}
	unsigned int I;    // 0-1023��������
	unsigned int J;    // 0-59��
	unsigned int K;    // 0-59��
	unsigned int Mode;  //0,1��ʾ�Ƿ�������

	unsigned int A = 0;
	unsigned int B = 0;
	info info1;

	vector<info>infoSet_A;
	vector<info>infoSet_B;
};





//���ӵ�Ԫ��Χ��
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

//�����˻����ԡ�
struct UAV
{
	Coor_XYZ coor;//��ά����
	double size;//���˻��ߴ�
	uint64 ID;//��ʶ����
};


class Nonrigid
{
public:
	Nonrigid();
	~Nonrigid();


//��1�������������
	//���� + �㼶 = ����
	uint64 toCodefromIJKandN(Coor_IJK P, int N);//�������������
	uint64 toCodefromCodeIJKandN(Coor_IJK P, int N);//�ɱ����������
	//����ת��������
	Coor_IJK toIJKfromCode(uint64 Code);
	Coor_IJK toCodeIJKfromCode(uint64 Code);
	//�������㼶
	int toNfromCode(uint64 Code);
	//����Ԫ
	uint64 FindDad_RigidCode(uint64 Code, int N);//�ռ����񸸵�Ԫ
	uint64 FindDad_Code(uint64 Code, int N);//���븸��Ԫ
	uint64 FindDad_NonRigidCode(uint64 Code);//�Ǹ��Ը���Ԫ
	//�ӵ�Ԫ
	void FindSon_Code(uint64 Code, int N, vector<uint64>&SonCodes);//ָ���㼶�ӵ�Ԫ���뼯��
	void FindSon_CodeFanwei(uint64 Code, GBorder& border);//�����㼶�ӵ�Ԫ��ѯ(��Χ)
	//�����������
	void Find_Neighbourhood_26(uint64 Code, vector<uint64> &codelist);//26����
	void Find_Neighbourhood_6(uint64 Code, vector<uint64> &codelist);//6����
	//��γ������ת����
	uint64 toCode_fromXYZ(Coor_XYZ p, int N);
	//��γ������ת��������
	Coor_IJK toIJK_fromXYZ(Coor_XYZ p, int N);
	//��γ������ת�������
	Coor_IJK toCodeIJK_fromXYZ(Coor_XYZ p, int N);


//��2��ֱ�ߵĸ�㻯
	void toGridpoint_fromLine(Coor_XYZ p1, Coor_XYZ p2, int N, vector<Coor_IJK>&pointlist);
	void toGridpoint_fromLine(Coor_XYZ p1, Coor_XYZ p2, int N, vector<uint64>&codelist);

//��3����ͻ���ʵ��
	void test_conflict(int amount);


private:


	//======================================��Ӧ�á�==================================================
	//��1�������Χ�е�ΨһID
	uint64 toIntID(Coor_XYZ leftdown, Coor_XYZ rightup);
	uint64 toRigid_IntID(Coor_XYZ leftdown, Coor_XYZ rightup);//���Ա�ʶ

	//��3��ֱ�ߵ�����
	void toGrid_fromLine(Coor_XYZ p1, Coor_XYZ p2, int N, vector<uint64>&codelist);


	//======================================��������ʵ�顿==================================================
	//��1��ģ�⺽��
	void simulate_Paths(int amount, vector<Coor_XYZ>&paths);
	void simulate_Paths_2(int num, vector<vector<Coor_XYZ>>lines);

	//��2��ֱ�߸�㻯ʵ��
	void test_gridding_line(vector<Coor_XYZ>paths);
	double toD_fromGrid(Coor_XYZ p1, Coor_XYZ p2, vector<uint64>codelist);
	double toD_fromGridPoint(Coor_XYZ p1, Coor_XYZ p2, vector<uint64>codelist);


	double getDistance_from_line_line(Coor_XYZ A, Coor_XYZ B, Coor_XYZ C, Coor_XYZ D);//������ֱ�߼����
	double getDistance_from_point_line(Coor_XYZ C, Coor_XYZ A, Coor_XYZ B);
	double getDistance_from_point_point(Coor_XYZ A, Coor_XYZ B);
	double getDistance_from_xianduan_xianduan(Coor_XYZ A, Coor_XYZ B, Coor_XYZ C, Coor_XYZ D);



	void coor_of_code(uint64 code, Coor_XYZ &center);//������������������
	void rigid_coor_of_code(uint64 code, Coor_XYZ &center);

	//======================================��У�顿==================================================
	// �ж����������Ƿ�Խ�磬Խ����޸�
	bool isOutofIJKBorder(Coor_IJK& newIJK, Coor_IJK& origin, int N);
	//��γ�ȸ߳������Ƿ�����ʵ����Χ
	bool isOutofXYZBorder(Coor_XYZ p);
	//����ȥ��
	void code_unique(vector<uint64>& vCode);

	//���ؼ����е��ظ�����
	int repeat(vector<int>code);

	//======================================��ԭ���ԡ�==================================================
	//�жϿռ���Ƿ��ڸ���������
	bool isPointinGrid(Coor_XYZ P, uint64 code);

	//����Ԫ����
	uint64 rigid_FindDadCode(uint64 Code, int N);
	//�ӵ�Ԫ��Χ
	void rigid_FindSon_CodeFanwei(uint64 Code, GBorder& border);
	//�������㼶
	int rigid_toNfromCode(uint64 Code);
	//��γ������ת����
	uint64 rigid_toCode_fromXYZ(Coor_XYZ p, int N);
	//��γ������תIJK
	Coor_IJK rigid_toIJK_fromXYZ(Coor_XYZ p, int N);
	//���� + �㼶 = ����
	uint64 rigid_toCodefromIJKandN(Coor_IJK P, int N);
	//����ת��������
	Coor_IJK rigid_toIJKfromCode(uint64 Code);

	//���ݶ�߶��������룬�����Ӧ�����İ˸��ǵ�����
	void CountCoord(uint64 Inputcode, vector<Coor_XYZ>&result);

};



