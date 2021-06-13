#include"Grid_3D.h"
#include <fstream>
#include<omp.h>
struct codelist
{
	vector<uint64>vcode;
};


int main()
{
	C3D_GriddingCode tt;
	void load_obj(vector<Triangle>& data, char filename[20]);//load data

	
//城市三角网模型网格化
#if 1


	//【1】读取数据
	vector<Triangle> tri;
	char filename[20] = "future.obj";
	load_obj(tri, filename);

	//【2】网格化
	vector<Coor_IJK>bunny;
	tt.triangleVoxelization(tri, path_N, bunny);
	tt.ScanLineSeedFill_3(bunny);

	cout << "The gridding city has been established!" << endl << endl;

#endif




//无人机路径规划（三维）
#if 1

	//【1】初始化空域网格
	vector<Coor_IJK>Area;
	tt.initFlyArea(Area, bunny);
	vector<vector<vector<bool>>>maze2;
	tt.init_GridPoint_Map(Area, maze2);
	Astar astar;


	vector<Point>pathPoint2;
	vector<Point>path2;


	path2.push_back(Point(11, 117, 29));
	path2.push_back(Point(117, 11, 29));


	astar.InitAstar(maze2);
	tt.sin_pathFinding_GridPoint(maze2, path2, pathPoint2, true);//格点路径规划

	char filename2[20] = "pointpath.vtk";
	tt.out_vtk_pointpath(pathPoint2, filename2);


#endif

}




void load_obj(vector<Triangle>& data, char filename[20])
{

	std::vector <Coor_XYZ> ppp;
	Coor_XYZ temp;
	FILE* fp = fopen(filename, "r");
	char buf[2048];
	string str_head_1 = "v";
	string str_head_2 = "f";
	string str_head_read;
	while (!feof(fp))
	{
		fgets(buf, 2, fp);//读取第一个字符

		str_head_read = buf;
		if (str_head_read.find(str_head_1) != string::npos)
		{
			fscanf(fp, "%lf%lf%lf", &temp.Y, &temp.Z, &temp.X);
			ppp.push_back(temp);
		}



		else if (str_head_read.find(str_head_2) != string::npos)
		{

			Triangle tt;

			int pp1[3];
			fscanf(fp, "%d%d%d", &pp1[0], &pp1[1], &pp1[2]);

			tt.angle[0] = ppp[pp1[0] - 1];
			tt.angle[1] = ppp[pp1[1] - 1];
			tt.angle[2] = ppp[pp1[2] - 1];

			data.push_back(tt);

		}

	}
	fclose(fp);

	cout << "City data read successfully!" << endl << endl;
}
