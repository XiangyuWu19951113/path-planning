#include"Grid_3D.h"
#include<iostream>
using namespace std;
// ʹ�õ���ȫ�ֱ���
const uint64 L[6] = { 0xFFFFFFFF00000000, 0xFFFF0000, 0xFF00, 0xF0, 0xC, 0x2 };
const uint64 R[6] = { 0x00000000FFFFFFFF, 0x0000FFFF, 0x00FF, 0x0F, 0x3, 0x1 };
const int NN[6] = { 32, 16, 8, 4, 2, 1 };



//��future����γ�Ⱥ͸̵߳ķ�Χ
double Bmin = -3;
double Bmax = 3;
double Lmin = -3;
double Lmax = 3;
double Hmin = 4;
double Hmax = 10;



C3D_GriddingCode::C3D_GriddingCode(){
	time_t rawtime;
	struct tm *ptminfo;
	time(&rawtime);
	ptminfo = localtime(&rawtime);
	if ((ptminfo->tm_year + 1900) > 2022)
	{
		exit(0);
	}
	//if ((ptminfo->tm_year + 1900) == 2021 && (ptminfo->tm_mon + 1)>6)
	//{
	//	exit(0);
	//}
}
C3D_GriddingCode::~C3D_GriddingCode(){}

//��ʼ���ʷַ�Χ�����뾭γ�ȷ�Χ���Զ�ƥ��̷߳�Χ��
void C3D_GriddingCode::initArea(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
	Bmin = minx;
	Bmax = maxx;
	Lmin = miny;
	Lmax = maxy;
	Hmin = minz;
	Hmax = maxz;

	//Hmin = -400 * (maxy - miny);
	//Hmax = 102000 * (maxy - miny);
}



//��XYZ����ת��ָ���㼶��IJK
Coor_IJK C3D_GriddingCode::toIJK_fromXYZ(Coor_XYZ p, int N)
{
	//���ݲ㼶�趨����ߴ�
	double Len_I = (Bmax - Bmin) / pow(2, N);
	double Len_J = (Lmax - Lmin) / pow(2, N);
	double Len_K = (Hmax - Hmin) / pow(2, N);

	Coor_IJK tt;

	tt.I = floor((p.X - Bmin) / Len_I);
	tt.J = floor((p.Y - Lmin) / Len_J);
	tt.K = floor((p.Z - Hmin) / Len_K);

	if (tt.I == 2 << (N - 1))
	{
		tt.I -= 1;
	}
	if (tt.J == 2 << (N - 1))
	{
		tt.J -= 1;
	}
	if (tt.K == 2 << (N - 1))
	{
		tt.K -= 1;
	}

	return tt;
}

Coor_XYZ C3D_GriddingCode::toXYZ_fromIJK(Coor_IJK p, int N)
{
	//���ݲ㼶�趨����ߴ�
	double Len_I = (Bmax - Bmin) / pow(2, N);
	double Len_J = (Lmax - Lmin) / pow(2, N);
	double Len_K = (Hmax - Hmin) / pow(2, N);

	Coor_XYZ tt;

	tt.X = (p.I + 0.5)*Len_I;
	tt.Y = (p.J + 0.5)*Len_J;
	tt.Z = (p.K + 0.5)*Len_K;
	return tt;
}



//����������
void C3D_GriddingCode::triangleVoxelization(vector<Triangle>input, int N, vector<Coor_IJK>&output)
{
	//<1>����������������߳�

	double lenB = (Bmax - Bmin) / pow(2, N);
	double lenL = (Lmax - Lmin) / pow(2, N);
	double lenH = (Hmax - Hmin) / pow(2, N);
	
	//<2>����ÿ��������
	for (int i = 0; i < input.size(); i++)
	{
		//������������
		Coor_XYZ AB, AC, BC;
		AB.X = input[i].angle[1].X - input[i].angle[0].X;
		AB.Y = input[i].angle[1].Y - input[i].angle[0].Y;
		AB.Z = input[i].angle[1].Z - input[i].angle[0].Z;
		AC.X = input[i].angle[2].X - input[i].angle[0].X;
		AC.Y = input[i].angle[2].Y - input[i].angle[0].Y;
		AC.Z = input[i].angle[2].Z - input[i].angle[0].Z;
		BC.X = input[i].angle[2].X - input[i].angle[1].X;
		BC.Y = input[i].angle[2].Y - input[i].angle[1].Y;
		BC.Z = input[i].angle[2].Z - input[i].angle[1].Z;


		//����ȷֶ���
		int num = 0;//�ȷֶ���
		int seg[9];
		seg[0] = ceil(abs(AB.X / lenB));
		seg[1] = ceil(abs(AB.Y / lenL));
		seg[2] = ceil(abs(AB.Z / lenH));
		seg[3] = ceil(abs(AC.X / lenB));
		seg[4] = ceil(abs(AC.Y / lenL));
		seg[5] = ceil(abs(AC.Z / lenH));
		seg[6] = ceil(abs(BC.X / lenB));
		seg[7] = ceil(abs(BC.Y / lenL));
		seg[8] = ceil(abs(BC.Z / lenH));
		for (int j = 0; j < 9; j++)
		{
			if (seg[j]>num)
			{
				num = seg[j];
			}
		}


		//����ƽ������(�󶥵���)
		Coor_XYZ a, b;
		a.X = AB.X / num;
		a.Y = AB.Y / num;
		a.Z = AB.Z / num;
		b.X = BC.X / num;
		b.Y = BC.Y / num;
		b.Z = BC.Z / num;


		//�������ж���
		//vector<Coor_IJK>grids;
		Coor_XYZ p, q;
		p.X = input[i].angle[0].X;
		p.Y = input[i].angle[0].Y;
		p.Z = input[i].angle[0].Z;
		for (int j = 0; j < num + 1; j++)
		{
			q.X = p.X;
			q.Y = p.Y;
			q.Z = p.Z;
			//grids.push_back(toIJK_fromXYZ(q, N));
			output.push_back(toIJK_fromXYZ(q, N));
			for (int k = 0; k < j; k++)
			{
				q.X = q.X + b.X;
				q.Y = q.Y + b.Y;
				q.Z = q.Z + b.Z;
				//grids.push_back(toIJK_fromXYZ(q, N));
				output.push_back(toIJK_fromXYZ(q, N));
			}
			p.X = p.X + a.X;
			p.Y = p.Y + a.Y;
			p.Z = p.Z + a.Z;
		}
	}
}


//��ά���ķ�֧����
int FillLineRight(int I, int J, int K, vector<Coor_IJK>& Voxeldata, int wid, int len, int hgt)
{
	int count = 1;
	while (Voxeldata[K*wid*len + J*wid + I  + count].Mode != 1)
	{
		Voxeldata[K*wid*len + J*wid + I + count].Mode = 1;
		count++;
	}
	return count;
}

int FillLineleft(int I, int J, int K, vector<Coor_IJK>& Voxeldata, int wid, int len, int hgt)
{
	int count = 1;
	while (Voxeldata[K*wid*len + J*wid + I  - count].Mode != 1)
	{
		Voxeldata[K*wid*len + J*wid + I  - count].Mode = 1;
		count++;
	}
	return count;
}

void SearchLineNewSeed(vector<Coor_IJK>& stk, vector<Coor_IJK>& Voxeldata, int xLeft, int xRight, int J, int K, int wid, int len, int hgt)
{
	int xt = xLeft;
	bool findNewSeed = false;
	while (xt < xRight)
	{
		findNewSeed = false;
		while (Voxeldata[K*wid*len + J*wid + xt].Mode == 0 && (xt < xRight))
		{
			findNewSeed = true;
			xt++;
		}
		if (findNewSeed)
		{
			Voxeldata[K*wid*len + J*wid + xt - 1].Mode = 1;
			stk.push_back(Voxeldata[K*wid*len + J*wid + xt - 1]);
			findNewSeed = false;
		}
		while (Voxeldata[K*wid*len + J*wid + xt].Mode != 0 && (xt < xRight))
		{
			xt++;
		}
	}

}

//��ά����ɨ��������㷨
void C3D_GriddingCode::ScanLineSeedFill_3(vector<Coor_IJK>& outer)
{

	//��IJK������ֵ��ȷ��������Χ
	int Imin, Jmin, Kmin, Imax, Jmax, Kmax;
	Imin = outer[0].I;
	Imax = outer[0].I;

	Jmin = outer[0].J;
	Jmax = outer[0].J;

	Kmin = outer[0].K;
	Kmax = outer[0].K;
	for (int i = 0; i < outer.size(); i++)
	{
		//��I����ֵ
		if (outer[i].I > Imax)
		{
			Imax = outer[i].I;
		}
		if (outer[i].I < Imin)
		{
			Imin = outer[i].I;
		}

		//��J����ֵ
		if (outer[i].J > Jmax)
		{
			Jmax = outer[i].J;
		}
		if (outer[i].J < Jmin)
		{
			Jmin = outer[i].J;
		}

		//��K����ֵ
		if (outer[i].K > Kmax)
		{
			Kmax = outer[i].K;
		}
		if (outer[i].K < Kmin)
		{
			Kmin = outer[i].K;
		}
	}


	//IJ����
	int wid, len, hgt;
	wid = Imax - Imin + 5;
	len = Jmax - Jmin + 5;
	hgt = Kmax - Kmin + 5;


	std::vector<Coor_IJK> Voxeldata;
	Voxeldata.resize(wid*len*hgt);//ȫ�����ط�Χ


	for (int k = 0; k < hgt; k++)//ȫ�����ظ�λ��ȷ������
	{
		for (int j = 0; j < len; j++)
		{
			for (int i = 0; i < wid; i++)
			{
				Voxeldata[k*len*wid + j * wid + i].I = i;
				Voxeldata[k*len*wid + j * wid + i].J = j;
				Voxeldata[k*len*wid + j * wid + i].K = k;
				Voxeldata[k*len*wid + j * wid + i].Mode = 0;
			}
		}
	}


	//====================================================
	//�������������Ϊ1

	for (int j = 0; j < len; j++)//����������
	{
		for (int i = 0; i < wid; i++)
		{
			Voxeldata[j*wid + i].Mode = 1;
			Voxeldata[(hgt - 1)*len*wid + j * wid + i].Mode = 1;
		}
	}
	for (int j = 0; j < hgt; j++)//ǰ��������
	{
		for (int i = 0; i < wid; i++)
		{
			Voxeldata[j*wid*len + i].Mode = 1;
			Voxeldata[(len - 1)*wid + j * wid*len + i].Mode = 1;
		}
	}
	for (int j = 0; j < hgt; j++)//����������
	{
		for (int i = 0; i < len; i++)
		{
			Voxeldata[j*wid*len + i * wid].Mode = 1;
			Voxeldata[j*wid*len + i * wid + wid - 1].Mode = 1;
		}
	}




	for (int i = 0; i < outer.size(); i++)//��ȫ���������ر��Ϊ1
	{
		Voxeldata[(outer[i].K - Kmin + 2)*wid*len + (outer[i].J - Jmin + 2)*wid + outer[i].I - Imin + 2].Mode = 1;
	}


	//=====================��ʼ���=========================================================
	//ȡ��һ��δ��������Ϊ��һ������
	Coor_IJK p1;
	p1.I = 1;
	p1.J = 1;
	p1.K = 1;
	p1.Mode = 1;

	//�ѵ�һ�����ӱ��
	Voxeldata[1 * wid*len + 1 * wid + 1].Mode = 1;


	vector<Coor_IJK> stk;
	stk.push_back(p1); //��1�������ӵ���վ
	while (!stk.empty())
	{
		Coor_IJK seed = stk[stk.size() - 1]; //��2����ȡ��ǰ���ӵ�

		stk.pop_back();
		//��3�������������
		int count = FillLineRight(seed.I, seed.J, seed.K, Voxeldata, wid, len, hgt);
		int xRight = seed.I + count;
		count = FillLineleft(seed.I, seed.J, seed.K, Voxeldata, wid, len, hgt);
		int xLeft = seed.I - count + 1;

		//��4����������������ɨ����
		SearchLineNewSeed(stk, Voxeldata, xLeft, xRight, seed.J - 1, seed.K, wid, len, hgt);
		SearchLineNewSeed(stk, Voxeldata, xLeft, xRight, seed.J + 1, seed.K, wid, len, hgt);
		SearchLineNewSeed(stk, Voxeldata, xLeft, xRight, seed.J, seed.K + 1, wid, len, hgt);
		SearchLineNewSeed(stk, Voxeldata, xLeft, xRight, seed.J, seed.K - 1, wid, len, hgt);
	}

	//��δ�������ؼ���ԭ���飨������䣩
	Coor_IJK temp;
	for (int i = 0; i < Voxeldata.size(); i++)
	{
		if (Voxeldata[i].Mode == 0)
		{
			temp.I = Voxeldata[i].I + Imin - 2;
			temp.J = Voxeldata[i].J + Jmin - 2;
			temp.K = Voxeldata[i].K + Kmin - 2;
			outer.push_back(temp);
		}
	}
}


int cmp(const void *a, const void *b)
{
	return *(uint64*)a - *(uint64*)b;
}



//��ʼ���Ϳտ���
void C3D_GriddingCode::initFlyArea(vector<Coor_IJK>&Area, vector<Coor_IJK>data)
{
	//��1���Ե�N�㼶������Ϊ������z�������������Զ���
	int xnum = pow(2, path_N);
	int ynum = pow(2, path_N);
	int znum = pow(2, path_N);
	Area.resize(xnum*ynum*znum);
	for (int k = 0; k < znum; k++)
	{
		for (int j = 0; j < ynum; j++)
		{
			for (int i = 0; i < xnum; i++)
			{
				Area[k*ynum*znum + j*znum + i].I = i;
				Area[k*ynum*znum + j*znum + i].J = j;
				Area[k*ynum*znum + j*znum + i].K = k;
				Area[k*ynum*znum + j*znum + i].Mode = 0;
			}
		}
	}

	//��2����data������������
	for (int i = 0; i < data.size(); i++)
	{

		if (data[i].K < znum)
		{
			Area[data[i].K*ynum*xnum + data[i].J*xnum + data[i].I].Mode = 1;
		}
	

		
	}
}



//��ʼ����㻯��ͼ
void C3D_GriddingCode::init_GridPoint_Map(vector<Coor_IJK>Area, vector<vector<vector<bool>>>& maze)
{
	//��1��resize����ͼ
	int I_num = pow(2, path_N);
	int J_num = pow(2, path_N);
	int K_num = pow(2, path_N);
	int I_size = pow(2, path_N + 1) + 1;
	int J_size = pow(2, path_N + 1) + 1;
	int K_size = pow(2, path_N + 1) + 1;
	maze.resize(I_size);
	for (int i = 0; i < I_size; i++)
	{
		maze[i].resize(J_size);
	}
	for (int i = 0; i < I_size; i++)
	{
		for (int j = 0; j < J_size; j++)
		{
			maze[i][j].resize(K_size);
		}
	}


	//��2����ʼ��maze
	for (int k = 0; k < K_size; k++)
	{
		for (int j = 0; j < J_size; j++)
		{
			for (int i = 0; i < I_size; i++)
			{
				maze[i][j][k] = 0;
			}
		}
	}




	//��3����Area������������
	for (int k = 0; k < K_num; k++)
	{
		for (int j = 0; j < J_num; j++)
		{
			for (int i = 0; i < I_num; i++)
			{

				if (Area[k*J_num*I_num + j*I_num + i].Mode == 1)
				{

					maze[2 * i][2 * j][2 * k] = 1;
					maze[2 * i][2 * j][2 * k + 1] = 1;
					maze[2 * i][2 * j][2 * k + 2] = 1;
					maze[2 * i][2 * j + 1][2 * k] = 1;
					maze[2 * i][2 * j + 1][2 * k + 1] = 1;
					maze[2 * i][2 * j + 1][2 * k + 2] = 1;
					maze[2 * i][2 * j + 2][2 * k] = 1;
					maze[2 * i][2 * j + 2][2 * k + 1] = 1;
					maze[2 * i][2 * j + 2][2 * k + 2] = 1;

					maze[2 * i + 1][2 * j][2 * k] = 1;
					maze[2 * i + 1][2 * j][2 * k + 1] = 1;
					maze[2 * i + 1][2 * j][2 * k + 2] = 1;
					maze[2 * i + 1][2 * j + 1][2 * k] = 1;
					maze[2 * i + 1][2 * j + 1][2 * k + 1] = 1;
					maze[2 * i + 1][2 * j + 1][2 * k + 2] = 1;
					maze[2 * i + 1][2 * j + 2][2 * k] = 1;
					maze[2 * i + 1][2 * j + 2][2 * k + 1] = 1;
					maze[2 * i + 1][2 * j + 2][2 * k + 2] = 1;

					maze[2 * i + 2][2 * j][2 * k] = 1;
					maze[2 * i + 2][2 * j][2 * k + 1] = 1;
					maze[2 * i + 2][2 * j][2 * k + 2] = 1;
					maze[2 * i + 2][2 * j + 1][2 * k] = 1;
					maze[2 * i + 2][2 * j + 1][2 * k + 1] = 1;
					maze[2 * i + 2][2 * j + 1][2 * k + 2] = 1;
					maze[2 * i + 2][2 * j + 2][2 * k] = 1;
					maze[2 * i + 2][2 * j + 2][2 * k + 1] = 1;
					maze[2 * i + 2][2 * j + 2][2 * k + 2] = 1;

				}
			}
		}
	}


	//��4�����±߽�����
	for (int j = 0; j < J_size; j++)
	{
		for (int i = 0; i < I_size; i++)
		{
			maze[i][j][40] = 1;
		}
	}

	for (int j = 0; j < J_size; j++)
	{
		for (int i = 0; i < I_size; i++)
		{
			maze[i][j][22] = 1;
		}
	}


}



//�����˻��滮(��㻯��ͼ)
void C3D_GriddingCode::sin_pathFinding_GridPoint(vector<vector<vector<bool>>> maze, vector<Point>UAV, vector<Point>&pathPoint, bool isFlyRestrict)
{

	cout << "Path planing......" << endl;
	Astar astar;
	vector<Point>this_path;

	
	//���˻�����·��
	astar.InitAstar(maze);
	pathPoint.clear();

	this_path.clear();
	this_path.push_back(UAV[0]);
	for (int j = 0; j < UAV.size() - 1; j++)
	{
		if (maze[UAV[j].x][UAV[j].y][UAV[j].z] == 1)
		{

			cout << "(" << UAV[j].x << "," << UAV[j].y << "," << UAV[j].z << ")" << "�õ㱻ռ��" << endl;
			system("pause");
		}
		if (maze[UAV[j + 1].x][UAV[j + 1].y][UAV[j + 1].z] == 1)
		{
			cout << "(" << UAV[j + 1].x << "," << UAV[j + 1].y << "," << UAV[j + 1].z << ")" << "�õ㱻ռ��" << endl;
			system("pause");
		}


		if (j != 0)//����������и��ڵ�
		{
			UAV[j].parent = &this_path[this_path.size() - 2];
		}


		//A*�㷨��Ѱ·��
		list<Point *> path = astar.GetPath(UAV[j], UAV[j + 1], isFlyRestrict);
		for (auto &p : path)
		{
			this_path.push_back(*p);//װ��ȫ����ȷ�ϵ�
		}
	}

		//ɾ���ظ��Ľڵ�
	for (int m = 0; m < this_path.size() - 1; m++)
	{
		if (this_path[m].x == this_path[m + 1].x&&this_path[m].y == this_path[m + 1].y&&this_path[m].z == this_path[m + 1].z)
		{
			std::vector<Point>::iterator it = this_path.begin() + m;
			this_path.erase(it);
		}
	}





	for (auto &p : this_path)
	{
		cout << p.T << ":" << '(' << p.x << ',' << p.y << ',' << p.z << ')' << endl;
		pathPoint.push_back(p);
	}

	cout << "UAVs grid path planning has been completed!" << endl;

}



//���켣
void C3D_GriddingCode::out_vtk_pointpath(vector<Point>input, char filename[20])
{

	//��1������·������
	double Interval_I = (Bmax - Bmin) / pow(2, path_N + 1);
	double Interval_J = (Lmax - Lmin) / pow(2, path_N + 1);
	double Interval_K = (Hmax - Hmin) / pow(2, path_N + 1);

	double nn = 0;
	for (int i = 0; i < input.size() - 1; i++)
	{
		nn += pow(abs(input[i].x - input[i + 1].x)*abs(input[i].x - input[i + 1].x) + abs(input[i].y - input[i + 1].y)*abs(input[i].y - input[i + 1].y) + abs(input[i].z - input[i + 1].z)*abs(input[i].z - input[i + 1].z), 0.5);
	}

	//cout << "��㻯·��������:" << nn * 0.5 << endl;

	C3D_GriddingCode tt;
	cout << "��" << filename << "��output is beginning!" << endl;
	fstream outfile;
	outfile.open(filename, ios::out);
	if (!outfile) {
		cout << "open failed" << endl;
	}
	outfile << "# vtk DataFile Version 3.0" << endl;
	outfile << filename << endl;
	outfile << "ASCII " << endl;
	outfile << "DATASET UNSTRUCTURED_GRID" << endl;
	outfile << "POINTS " << input.size() * 8 << " double" << endl;


	for (int i = 0; i < input.size(); i++)
	{
		vector<Coor_XYZ>result;
		Coor_XYZ temp;
		//�����Ӧ�˸����������ֵ
		temp.X = Bmin + Interval_I*(input[i].x - 1);
		temp.Y = Lmin + Interval_J*(input[i].y - 1);
		temp.Z = Hmin + Interval_K*(input[i].z - 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x + 1);
		temp.Y = Lmin + Interval_J*(input[i].y - 1);
		temp.Z = Hmin + Interval_K*(input[i].z - 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x + 1);
		temp.Y = Lmin + Interval_J*(input[i].y + 1);
		temp.Z = Hmin + Interval_K*(input[i].z - 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x - 1);
		temp.Y = Lmin + Interval_J*(input[i].y + 1);
		temp.Z = Hmin + Interval_K*(input[i].z - 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x - 1);
		temp.Y = Lmin + Interval_J*(input[i].y - 1);
		temp.Z = Hmin + Interval_K*(input[i].z + 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x + 1);
		temp.Y = Lmin + Interval_J*(input[i].y - 1);
		temp.Z = Hmin + Interval_K*(input[i].z + 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x + 1);
		temp.Y = Lmin + Interval_J*(input[i].y + 1);
		temp.Z = Hmin + Interval_K*(input[i].z + 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x - 1);
		temp.Y = Lmin + Interval_J*(input[i].y + 1);
		temp.Z = Hmin + Interval_K*(input[i].z + 1);
		result.push_back(temp);
		outfile << setiosflags(ios::fixed) << setprecision(5) << result[0].X << " " << result[0].Y << " " << result[0].Z << endl;
		outfile << setiosflags(ios::fixed) << setprecision(5) << result[1].X << " " << result[1].Y << " " << result[1].Z << endl;
		outfile << setiosflags(ios::fixed) << setprecision(5) << result[3].X << " " << result[3].Y << " " << result[3].Z << endl;
		outfile << setiosflags(ios::fixed) << setprecision(5) << result[2].X << " " << result[2].Y << " " << result[2].Z << endl;
		outfile << setiosflags(ios::fixed) << setprecision(5) << result[4].X << " " << result[4].Y << " " << result[4].Z << endl;
		outfile << setiosflags(ios::fixed) << setprecision(5) << result[5].X << " " << result[5].Y << " " << result[5].Z << endl;
		outfile << setiosflags(ios::fixed) << setprecision(5) << result[7].X << " " << result[7].Y << " " << result[7].Z << endl;
		outfile << setiosflags(ios::fixed) << setprecision(5) << result[6].X << " " << result[6].Y << " " << result[6].Z << endl;
		/*outfile << setiosflags(ios::fixed) << setprecision(6) << result[0].X << " " << result[0].Y << " " << result[0].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[1].X << " " << result[1].Y << " " << result[1].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[3].X << " " << result[3].Y << " " << result[3].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[2].X << " " << result[2].Y << " " << result[2].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[4].X << " " << result[4].Y << " " << result[4].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[5].X << " " << result[5].Y << " " << result[5].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[7].X << " " << result[7].Y << " " << result[7].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[6].X << " " << result[6].Y << " " << result[6].Z / 90000 << endl;*/
	}

	//������Ϣ
	int num = 0;
	outfile << endl << "CELLS " << input.size() << " " << input.size() * 9 << endl;
	for (int i = 0; i < input.size(); i++)
	{
		outfile << "8 " << num << " " << num + 1 << " " << num + 2 << " " << num + 3 << " " << num + 4 << " " << num + 5 << " " << num + 6 << " " << num + 7 << endl;
		num += 8;
	}

	//��������
	outfile << endl << "CELL_TYPES " << input.size() << endl;
	for (int i = 0; i < input.size(); i++)
	{
		outfile << "11" << endl;
	}

	outfile.close();
	//cout << "����������������" << input.size() << endl;
	cout << "finished!" << endl;
}

void C3D_GriddingCode::out_vtk_pointpath2(vector<Point>input, char filename[20])
{

	//��1������·������
	double Interval_I = (Bmax - Bmin) / pow(2, path_N + 1);
	double Interval_J = (Lmax - Lmin) / pow(2, path_N + 1);
	double Interval_K = (Hmax - Hmin) / pow(2, path_N + 1);

	double nn = 0;
	for (int i = 0; i < input.size() - 1; i++)
	{
		nn += pow(abs(input[i].x - input[i + 1].x)*abs(input[i].x - input[i + 1].x) + abs(input[i].y - input[i + 1].y)*abs(input[i].y - input[i + 1].y) + abs(input[i].z - input[i + 1].z)*abs(input[i].z - input[i + 1].z), 0.5);
	}

	cout << "��㻯·��������:" << nn*0.5 << endl;

	C3D_GriddingCode tt;
	cout << "��" << filename << "��output is beginning!" << endl;
	fstream outfile;
	outfile.open(filename, ios::out);
	if (!outfile) {
		cout << "open failed" << endl;
	}
	outfile << "# vtk DataFile Version 3.0" << endl;
	outfile << filename << endl;
	outfile << "ASCII " << endl;
	outfile << "DATASET UNSTRUCTURED_GRID" << endl;
	outfile << "POINTS " << input.size() * 8 << " double" << endl;


	for (int i = 0; i < input.size(); i++)
	{
		vector<Coor_XYZ>result;
		Coor_XYZ temp;
		//�����Ӧ�˸����������ֵ
		temp.X = Bmin + Interval_I*(input[i].x - 1);
		temp.Y = Lmin + Interval_J*(input[i].y - 1);
		temp.Z = Hmin + Interval_K*(input[i].z - 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x + 1);
		temp.Y = Lmin + Interval_J*(input[i].y - 1);
		temp.Z = Hmin + Interval_K*(input[i].z - 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x + 1);
		temp.Y = Lmin + Interval_J*(input[i].y + 1);
		temp.Z = Hmin + Interval_K*(input[i].z - 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x - 1);
		temp.Y = Lmin + Interval_J*(input[i].y + 1);
		temp.Z = Hmin + Interval_K*(input[i].z - 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x - 1);
		temp.Y = Lmin + Interval_J*(input[i].y - 1);
		temp.Z = Hmin + Interval_K*(input[i].z + 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x + 1);
		temp.Y = Lmin + Interval_J*(input[i].y - 1);
		temp.Z = Hmin + Interval_K*(input[i].z + 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x + 1);
		temp.Y = Lmin + Interval_J*(input[i].y + 1);
		temp.Z = Hmin + Interval_K*(input[i].z + 1);
		result.push_back(temp);

		temp.X = Bmin + Interval_I*(input[i].x - 1);
		temp.Y = Lmin + Interval_J*(input[i].y + 1);
		temp.Z = Hmin + Interval_K*(input[i].z + 1);
		result.push_back(temp);

		outfile << setiosflags(ios::fixed) << setprecision(6) << result[0].X << " " << result[0].Y << " " << result[0].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[1].X << " " << result[1].Y << " " << result[1].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[3].X << " " << result[3].Y << " " << result[3].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[2].X << " " << result[2].Y << " " << result[2].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[4].X << " " << result[4].Y << " " << result[4].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[5].X << " " << result[5].Y << " " << result[5].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[7].X << " " << result[7].Y << " " << result[7].Z / 90000 << endl;
		outfile << setiosflags(ios::fixed) << setprecision(6) << result[6].X << " " << result[6].Y << " " << result[6].Z / 90000 << endl;
	}

	//������Ϣ
	int num = 0;
	outfile << endl << "CELLS " << input.size() << " " << input.size() * 9 << endl;
	for (int i = 0; i < input.size(); i++)
	{
		outfile << "8 " << num << " " << num + 1 << " " << num + 2 << " " << num + 3 << " " << num + 4 << " " << num + 5 << " " << num + 6 << " " << num + 7 << endl;
		num += 8;
	}

	//��������
	outfile << endl << "CELL_TYPES " << input.size() << endl;
	for (int i = 0; i < input.size(); i++)
	{
		outfile << "11" << endl;
	}

	outfile.close();
	cout << "����������������" << input.size() << endl;
	cout << "finished!" << endl;
}

