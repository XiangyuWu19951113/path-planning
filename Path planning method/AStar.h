#pragma once
/*
//A*算法对象类
*/
#include <vector>
#include <list>
#include <set>
#include <iostream>
#include<algorithm>


using namespace std;
const int kCost1 = 10; //直移一格消耗
const int kCost2 = 15; //斜移一格消耗
const int kCost3 = 18; //斜移一格消耗

struct Point
{
	int T;//时间点，代表该时刻该网格被占用
	int x, y, z; //点坐标
	int F, G, H; //F=G+H
	Point *parent; //parent的坐标，这里没有用指针，从而简化代码
	Point(int _x, int _y, int _z) :x(_x), y(_y),z(_z),T(0), F(0), G(0), H(0), parent(NULL)  //变量初始化
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


	//创客大赛-多无人机同时规划
	//std::vector<std::list<Point *>> GetPath_mul_UVAs(vector<Point>start, vector<Point>end);






private:
	Point *findPath(Point &startPoint, Point &endPoint, bool isFlyRestrict);
	Point *findPath_mulgrid(Point &startPoint, Point &endPoint);

	std::vector<Point *> getSurroundPoints(const Point *point, bool isFlyRestrict) const;
	std::vector<Point *> getSurroundPoints_mulgrid(const Point *point) const;

	bool isCanreach(const Point *point, const Point *target, bool isFlyRestrict) const; //判断某点是否可以用于下一步判断
	bool isCanreach_mulgrid(const Point *point, const Point *target) const; //判断某点是否可以用于下一步判断


	Point *isInList(const std::list<Point *> &list, const Point *point) const; //判断开启/关闭列表中是否包含某点
	Point *getLeastFpoint(); //从开启列表中返回F值最小的节点
	//计算FGH值
	int calcG(Point *temp_start, Point *point);
	int calcH(Point *point, Point *end);
	int calcF(Point *point);
	
private:
	std::vector<std::vector<std::vector<bool>>> maze;
	std::list<Point *> openList;  //开启列表
	std::list<Point *> closeList; //关闭列表

	std::set<Point *> openSet;  //开启列表
	std::set<Point *> closeSet; //关闭列表
};