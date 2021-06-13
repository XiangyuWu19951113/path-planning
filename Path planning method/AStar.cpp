#include <math.h>
#include"AStar.h"

void Astar::InitAstar(std::vector<std::vector<std::vector<bool>>> &_maze)
{
	maze = _maze;
}

void Astar::update_Astar(Point &update)
{
	maze[update.x][update.y][update.z] = 1;
}

int Astar::calcG(Point *temp_start, Point *point)
{
	int extraG = 0;
	if ((abs(point->x - temp_start->x) + abs(point->y - temp_start->y) + abs(point->z - temp_start->z)) == 1)
	{
		extraG = kCost1;
	}
	else if ((abs(point->x - temp_start->x) + abs(point->y - temp_start->y) + abs(point->z - temp_start->z)) == 2)
	{
		extraG = kCost2;
	}
	else
	{
		extraG = kCost3;
	}
	int parentG = point->parent == NULL ? 0 : point->parent->G; //如果是初始节点，则其父节点是空
	return parentG + extraG;
}

int Astar::calcH(Point *point, Point *end)
{
	//用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法
	//return sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y) + (double)(end->z - point->z)*(double)(end->z - point->z))*kCost1;

	//用对角线距离

	int a = abs(point->x - end->x);
	int b = abs(point->y - end->y);
	int c = abs(point->z - end->z);
	int t = 0;
	if (a>b) { t = a; a = b; b = t; }
	if (b>c) { t = b; b = c; c = t; }
	if (a>b) { t = a; a = b; b = t; }//a最小,c最大

	return a*kCost3 + (b - a)*kCost2 + (c - b)*kCost1;


	//double h_diagonal = min(abs(point->x - end->x), abs(point->y - end->y));
	//double h_straight = (abs(point->x - end->x) + abs(point->y - end->y) + abs(point->z - end->z));
	//return (kCost2 * h_diagonal + kCost1 * (h_straight - 2 * h_diagonal));
	
}

int Astar::calcF(Point *point)
{
	return point->G + point->H;
}

Point *Astar::getLeastFpoint()
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();
		for (auto &point : openList)
		if (point->F<resPoint->F)
			resPoint = point;
		return resPoint;
	}
	return NULL;
}

Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	Point *newP = new Point(startPoint.x, startPoint.y, startPoint.z);
	newP->parent = startPoint.parent;
	openList.push_back(newP); //置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //找到F值最小的点
		openList.remove(curPoint); //从开启列表中删除
		closeList.push_back(curPoint); //放到关闭列表
		//1,找到当前周围可以通过的格子
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//cout << "拓展节点:(" << target->x << "," << target->y << "," << target->z << ")" << endl;
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);

				openList.push_back(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
			else
			{
				int tempG = calcG(curPoint, target);
				if (tempG<target->G)
				{
					target->parent = curPoint;

					target->G = tempG;
					target->F = calcF(target);
				}
			}
			Point *resPoint = isInList(openList, &endPoint);
			if (resPoint)
				return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
		}
	}

	return NULL;
}

Point *Astar::findPath_mulgrid(Point &startPoint, Point &endPoint)
{
	Point *newP = new Point(startPoint.x, startPoint.y, startPoint.z);
	newP->parent = startPoint.parent;
	openList.push_back(newP); //置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //找到F值最小的点
		openList.remove(curPoint); //从开启列表中删除
		closeList.push_back(curPoint); //放到关闭列表
		//1,找到当前周围可以通过的格子
		auto surroundPoints = getSurroundPoints_mulgrid(curPoint);
		for (auto &target : surroundPoints)
		{
			//cout << "拓展节点:(" << target->x << "," << target->y << "," << target->z << ")" << endl;
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);

				openList.push_back(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
			else
			{
				int tempG = calcG(curPoint, target);
				if (tempG<target->G)
				{
					target->parent = curPoint;

					target->G = tempG;
					target->F = calcF(target);
				}
			}
			Point *resPoint = isInList(openList, &endPoint);
			if (resPoint)
				return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
		}
	}

	return NULL;
}


std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	if (startPoint.x == endPoint.x&&startPoint.y == endPoint.y&&startPoint.z == endPoint.z)
	{
		std::list<Point *> path;
		path.push_front(&startPoint);
		return path;
	}



	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point *> path;
	//返回路径，如果没找到路径，返回空链表
	while (result)
	{
		if (result->x == startPoint.x&&result->y == startPoint.y&&result->z == startPoint.z)
		{
			break;
		}
		else
		{
			path.push_front(result);
			result = result->parent;
		}
	}

	// 清空临时开闭列表，防止重复执行GetPath导致结果异常
	openList.clear();
	closeList.clear();

	return path;
}

std::list<Point *> Astar::GetPath_mulgrid(Point &startPoint, Point &endPoint)
{
	Point *result = findPath_mulgrid(startPoint, endPoint);
	std::list<Point *> path;
	//返回路径，如果没找到路径，返回空链表
	while (result)
	{
		if (result->x == startPoint.x&&result->y == startPoint.y&&result->z == startPoint.z)
		{
			break;
		}
		else
		{
			path.push_front(result);
			result = result->parent;
		}
	}

	// 清空临时开闭列表，防止重复执行GetPath导致结果异常
	openList.clear();
	closeList.clear();

	return path;
}







Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
	for (auto p : list)
	if (p->x == point->x&&p->y == point->y&&p->z == point->z)
		return p;
	return NULL;
}

bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const
{

	if (target->x<0 || target->x>maze.size() - 1
		|| target->y<0 || target->y>maze[0].size() - 1
		|| target->z<0 || target->z>maze[0][0].size() - 1
		|| maze[target->x][target->y][target->z] == 1
		|| target->x == point->x&&target->y == point->y&&target->z == point->z
		|| isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
		return false;
	else
	{

		if (isIgnoreCorner == 1)
		{
			return true;
		}

		else
		{
			if (abs(point->x - target->x) + abs(point->y - target->y) == 1)
			{
				if (abs(point->z - target->z) == 0)
				{
					return true;
				}
				else
				{
					if (maze[point->x][point->y][target->z] == 0 && maze[target->x][target->y][point->z] == 0)
						return true;
					else
						return false;
				}
			}
			else
			{
				if (abs(point->z - target->z) == 0)
				{
					if (maze[point->x][target->y][point->z] == 0 && maze[target->x][point->y][point->z] == 0)
						return true;
					else
						return false;
				}
				else
				{
					if (maze[point->x][target->y][point->z] == 0 && maze[target->x][point->y][point->z] == 0 && maze[point->x][target->y][target->z] == 0 && maze[target->x][point->y][target->z] == 0 && maze[point->x][point->y][target->z] == 0 && maze[target->x][target->y][point->z] == 0)
						return true;
					else
						return false;
				}
			}
		}

	}
}

bool Astar::isCanreach_mulgrid(const Point *point, const Point *target) const
{
	if (target->x<1 || target->x>maze.size() - 2
		|| target->y<1 || target->y>maze[0].size() - 2
		|| target->z<1 || target->z>maze[0][0].size() - 2
		|| maze[target->x][target->y][target->z] == 1
		|| target->x == point->x&&target->y == point->y&&target->z == point->z
		|| isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
		return false;
	else
	{
		if (abs(point->x - target->x) == 1 && abs(point->y - target->y) == 0)
		{
			if (maze[2 * target->x - point->x][point->y][point->z] == 0 && maze[2 * target->x - point->x][point->y - 1][point->z] == 0 && maze[2 * target->x - point->x][point->y + 1][point->z] == 0)
				return true;
			else
				return false;
		}


		else if (abs(point->x - target->x) == 0 && abs(point->y - target->y) == 1)
		{
			if (maze[point->x][2 * target->y - point->y][point->z] == 0 && maze[point->x - 1][2 * target->y - point->y][point->z] == 0 && maze[point->x + 1][2 * target->y - point->y][point->z] == 0)
				return true;
			else
				return false;
		}
		else
		{
			if (maze[2 * target->x - point->x][2 * target->y - point->y][point->z] == 0 && maze[2 * target->x - point->x][point->y][point->z] == 0 && maze[point->x][2 * target->y - point->y][point->z] == 0 && maze[2 * target->x - point->x][target->y][point->z] == 0 && maze[target->x][2 * target->y - point->y][point->z] == 0)
				return true;
			else
				return false;
		}
	}
}

std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) const
{
	std::vector<Point *> surroundPoints;

	//无飞行限制，任意26邻域可选
	if (isIgnoreCorner == true)
	{
		for (int x = point->x - 1; x <= point->x + 1; x++)
		for (int y = point->y - 1; y <= point->y + 1; y++)
		for (int z = point->z - 1; z <= point->z + 1; z++)
		//for (int z = point->z; z <= point->z; z++)
		if (isCanreach(point, new Point(x, y, z), isIgnoreCorner))
			surroundPoints.push_back(new Point(x, y, z));

		return surroundPoints;
	}

	//有最大转弯半径和俯仰角限制
		//如果是起点
		if (point->parent == NULL)
		{
			for (int x = point->x - 1; x <= point->x + 1; x++)
			for (int y = point->y - 1; y <= point->y + 1; y++)
			for (int z = point->z - 1; z <= point->z + 1; z++)
			//for (int z = point->z; z <= point->z; z++)
			if (x == point->x&&y == point->y&&z == point->z + 1)
			{
				continue;
			}
			else if (x == point->x&&y == point->y&&z == point->z - 1)
			{
				continue;
			}
			else
			{
				if (isCanreach(point, new Point(x, y, z), isIgnoreCorner))
					surroundPoints.push_back(new Point(x, y, z));
			}
			return surroundPoints;
		}


		else
		{
			if (abs(point->x - point->parent->x) == 1 && abs(point->y - point->parent->y) == 0)
			{
				for (int z = point->z - 1; z <= point->z + 1; z++)
				//for (int z = point->z; z <= point->z; z++)
				{
					if (isCanreach(point, new Point(point->x + (point->x - point->parent->x), point->y + 1, z), isIgnoreCorner))
						surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y + 1, z));
					if (isCanreach(point, new Point(point->x + (point->x - point->parent->x), point->y - 1, z), isIgnoreCorner))
						surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y - 1, z));
					if (isCanreach(point, new Point(point->x + (point->x - point->parent->x), point->y, z), isIgnoreCorner))
						surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y, z));
				}
				return surroundPoints;
			}
			else if (abs(point->x - point->parent->x) == 0 && abs(point->y - point->parent->y) == 1)
			{
				//for (int z = point->z; z <= point->z; z++)
				for (int z = point->z - 1; z <= point->z + 1; z++)
				{
					if (isCanreach(point, new Point(point->x - 1, point->y + (point->y - point->parent->y), z), isIgnoreCorner))
						surroundPoints.push_back(new Point(point->x - 1, point->y + (point->y - point->parent->y), z));
					if (isCanreach(point, new Point(point->x + 1, point->y + (point->y - point->parent->y), z), isIgnoreCorner))
						surroundPoints.push_back(new Point(point->x + 1, point->y + (point->y - point->parent->y), z));
					if (isCanreach(point, new Point(point->x, point->y + (point->y - point->parent->y), z), isIgnoreCorner))
						surroundPoints.push_back(new Point(point->x, point->y + (point->y - point->parent->y), z));
				}
				return surroundPoints;
			}
			else if (abs(point->x - point->parent->x) == 1 && abs(point->y - point->parent->y) == 1)
			{
				//for (int z = point->z; z <= point->z; z++)
				for (int z = point->z - 1; z <= point->z + 1; z++)
				{
					if (isCanreach(point, new Point(point->x, point->y + (point->y - point->parent->y), z), isIgnoreCorner))
						surroundPoints.push_back(new Point(point->x, point->y + (point->y - point->parent->y), z));
					if (isCanreach(point, new Point(point->x + (point->x - point->parent->x), point->y, z), isIgnoreCorner))
						surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y, z));
					if (isCanreach(point, new Point(point->x + (point->x - point->parent->x), point->y + (point->y - point->parent->y), z), isIgnoreCorner))
						surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y + (point->y - point->parent->y), z));
				}
				return surroundPoints;
			}
		}		
}


std::vector<Point *> Astar::getSurroundPoints_mulgrid(const Point *point) const
{
	std::vector<Point *> surroundPoints;

	//有最大转弯半径和俯仰角限制
	//如果是起点
	if (point->parent == NULL)
	{
		for (int x = point->x - 1; x <= point->x + 1; x++)
		for (int y = point->y - 1; y <= point->y + 1; y++)
			//for (int z = point->z - 1; z <= point->z + 1; z++)
		for (int z = point->z; z <= point->z; z++)
		if (x == point->x&&y == point->y&&z == point->z + 1)
		{
			continue;
		}
		else if (x == point->x&&y == point->y&&z == point->z - 1)
		{
			continue;
		}
		else
		{
			if (isCanreach_mulgrid(point, new Point(x, y, z)))
				surroundPoints.push_back(new Point(x, y, z));
		}
		return surroundPoints;
	}


	else
	{
		if (abs(point->x - point->parent->x) == 1 && abs(point->y - point->parent->y) == 0)
		{
			//for (int z = point->z - 1; z <= point->z + 1; z++)
				for (int z = point->z; z <= point->z; z++)
			{
				if (isCanreach_mulgrid(point, new Point(point->x + (point->x - point->parent->x), point->y + 1, z)))
					surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y + 1, z));
				if (isCanreach_mulgrid(point, new Point(point->x + (point->x - point->parent->x), point->y - 1, z)))
					surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y - 1, z));
				if (isCanreach_mulgrid(point, new Point(point->x + (point->x - point->parent->x), point->y, z)))
					surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y, z));
			}
			return surroundPoints;
		}
		else if (abs(point->x - point->parent->x) == 0 && abs(point->y - point->parent->y) == 1)
		{
			for (int z = point->z; z <= point->z; z++)
			//for (int z = point->z - 1; z <= point->z + 1; z++)
			{
				if (isCanreach_mulgrid(point, new Point(point->x - 1, point->y + (point->y - point->parent->y), z)))
					surroundPoints.push_back(new Point(point->x - 1, point->y + (point->y - point->parent->y), z));
				if (isCanreach_mulgrid(point, new Point(point->x + 1, point->y + (point->y - point->parent->y), z)))
					surroundPoints.push_back(new Point(point->x + 1, point->y + (point->y - point->parent->y), z));
				if (isCanreach_mulgrid(point, new Point(point->x, point->y + (point->y - point->parent->y), z)))
					surroundPoints.push_back(new Point(point->x, point->y + (point->y - point->parent->y), z));
			}
			return surroundPoints;
		}
		else if (abs(point->x - point->parent->x) == 1 && abs(point->y - point->parent->y) == 1)
		{
			for (int z = point->z; z <= point->z; z++)
			//for (int z = point->z - 1; z <= point->z + 1; z++)
			{
				if (isCanreach_mulgrid(point, new Point(point->x, point->y + (point->y - point->parent->y), z)))
					surroundPoints.push_back(new Point(point->x, point->y + (point->y - point->parent->y), z));
				if (isCanreach_mulgrid(point, new Point(point->x + (point->x - point->parent->x), point->y, z)))
					surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y, z));
				if (isCanreach_mulgrid(point, new Point(point->x + (point->x - point->parent->x), point->y + (point->y - point->parent->y), z)))
					surroundPoints.push_back(new Point(point->x + (point->x - point->parent->x), point->y + (point->y - point->parent->y), z));
			}
			return surroundPoints;
		}
	}
}