#pragma once
/*
//A*�㷨������
*/
#include <vector>
#include <list>
#include <set>
#include <iostream>
#include<algorithm>


using namespace std;
const int kCost1 = 10; //ֱ��һ������
const int kCost2 = 15; //б��һ������
const int kCost3 = 18; //б��һ������

struct Point
{
	int T;//ʱ��㣬�����ʱ�̸�����ռ��
	int x, y, z; //������
	int F, G, H; //F=G+H
	Point *parent; //parent�����꣬����û����ָ�룬�Ӷ��򻯴���
	Point(int _x, int _y, int _z) :x(_x), y(_y),z(_z),T(0), F(0), G(0), H(0), parent(NULL)  //������ʼ��
	{
	}
};

class Astar
{
public:
	void InitAstar(std::vector<std::vector<std::vector<bool>>> &_maze);
	void update_Astar(Point &update);
	std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isFlyRestrict);
	std::list<Point *> GetPath_mulgrid(Point &startPoint, Point &endPoint);


	//���ʹ���-�����˻�ͬʱ�滮
	//std::vector<std::list<Point *>> GetPath_mul_UVAs(vector<Point>start, vector<Point>end);






private:
	Point *findPath(Point &startPoint, Point &endPoint, bool isFlyRestrict);
	Point *findPath_mulgrid(Point &startPoint, Point &endPoint);

	std::vector<Point *> getSurroundPoints(const Point *point, bool isFlyRestrict) const;
	std::vector<Point *> getSurroundPoints_mulgrid(const Point *point) const;

	bool isCanreach(const Point *point, const Point *target, bool isFlyRestrict) const; //�ж�ĳ���Ƿ����������һ���ж�
	bool isCanreach_mulgrid(const Point *point, const Point *target) const; //�ж�ĳ���Ƿ����������һ���ж�


	Point *isInList(const std::list<Point *> &list, const Point *point) const; //�жϿ���/�ر��б����Ƿ����ĳ��
	Point *getLeastFpoint(); //�ӿ����б��з���Fֵ��С�Ľڵ�
	//����FGHֵ
	int calcG(Point *temp_start, Point *point);
	int calcH(Point *point, Point *end);
	int calcF(Point *point);
	
private:
	std::vector<std::vector<std::vector<bool>>> maze;
	std::list<Point *> openList;  //�����б�
	std::list<Point *> closeList; //�ر��б�

	std::set<Point *> openSet;  //�����б�
	std::set<Point *> closeSet; //�ر��б�
};