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
	int parentG = point->parent == NULL ? 0 : point->parent->G; //����ǳ�ʼ�ڵ㣬���丸�ڵ��ǿ�
	return parentG + extraG;
}

int Astar::calcH(Point *point, Point *end)
{
	//�ü򵥵�ŷ����þ������H�����H�ļ����ǹؼ������кܶ��㷨
	//return sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y) + (double)(end->z - point->z)*(double)(end->z - point->z))*kCost1;

	//�öԽ��߾���

	int a = abs(point->x - end->x);
	int b = abs(point->y - end->y);
	int c = abs(point->z - end->z);
	int t = 0;
	if (a>b) { t = a; a = b; b = t; }
	if (b>c) { t = b; b = c; c = t; }
	if (a>b) { t = a; a = b; b = t; }//a��С,c���

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
	openList.push_back(newP); //�������,��������һ���ڵ㣬�������
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //�ҵ�Fֵ��С�ĵ�
		openList.remove(curPoint); //�ӿ����б���ɾ��
		closeList.push_back(curPoint); //�ŵ��ر��б�
		//1,�ҵ���ǰ��Χ����ͨ���ĸ���
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//cout << "��չ�ڵ�:(" << target->x << "," << target->y << "," << target->z << ")" << endl;
			//2,��ĳһ�����ӣ���������ڿ����б��У����뵽�����б����õ�ǰ��Ϊ�丸�ڵ㣬����F G H
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);

				openList.push_back(target);
			}
			//3����ĳһ�����ӣ����ڿ����б��У�����Gֵ, �����ԭ���Ĵ�, ��ʲô������, �����������ĸ��ڵ�Ϊ��ǰ��,������G��F
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
				return resPoint; //�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ���������
		}
	}

	return NULL;
}

Point *Astar::findPath_mulgrid(Point &startPoint, Point &endPoint)
{
	Point *newP = new Point(startPoint.x, startPoint.y, startPoint.z);
	newP->parent = startPoint.parent;
	openList.push_back(newP); //�������,��������һ���ڵ㣬�������
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //�ҵ�Fֵ��С�ĵ�
		openList.remove(curPoint); //�ӿ����б���ɾ��
		closeList.push_back(curPoint); //�ŵ��ر��б�
		//1,�ҵ���ǰ��Χ����ͨ���ĸ���
		auto surroundPoints = getSurroundPoints_mulgrid(curPoint);
		for (auto &target : surroundPoints)
		{
			//cout << "��չ�ڵ�:(" << target->x << "," << target->y << "," << target->z << ")" << endl;
			//2,��ĳһ�����ӣ���������ڿ����б��У����뵽�����б����õ�ǰ��Ϊ�丸�ڵ㣬����F G H
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);

				openList.push_back(target);
			}
			//3����ĳһ�����ӣ����ڿ����б��У�����Gֵ, �����ԭ���Ĵ�, ��ʲô������, �����������ĸ��ڵ�Ϊ��ǰ��,������G��F
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
				return resPoint; //�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ���������
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
	//����·�������û�ҵ�·�������ؿ�����
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

	// �����ʱ�����б���ֹ�ظ�ִ��GetPath���½���쳣
	openList.clear();
	closeList.clear();

	return path;
}

std::list<Point *> Astar::GetPath_mulgrid(Point &startPoint, Point &endPoint)
{
	Point *result = findPath_mulgrid(startPoint, endPoint);
	std::list<Point *> path;
	//����·�������û�ҵ�·�������ؿ�����
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

	// �����ʱ�����б���ֹ�ظ�ִ��GetPath���½���쳣
	openList.clear();
	closeList.clear();

	return path;
}







Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
	//�ж�ĳ���ڵ��Ƿ����б��У����ﲻ�ܱȽ�ָ�룬��Ϊÿ�μ����б����¿��ٵĽڵ㣬ֻ�ܱȽ�����
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
		|| isInList(closeList, target)) //������뵱ǰ�ڵ��غϡ�������ͼ�����ϰ�������ڹر��б��У�����false
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
		|| isInList(closeList, target)) //������뵱ǰ�ڵ��غϡ�������ͼ�����ϰ�������ڹر��б��У�����false
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

	//�޷������ƣ�����26�����ѡ
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

	//�����ת��뾶�͸���������
		//��������
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

	//�����ת��뾶�͸���������
	//��������
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