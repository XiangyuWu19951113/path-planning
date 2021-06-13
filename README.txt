## Requirements
Visual Studio 2013, coded by C++

#Coding method and conflict detection method
The files include 3D_code_nonrigid.h , 3D_code_nonrigid.lib, 3D_code_nonrigid.dll.
This file contains all the coding algebraic algorithms.
Use test_conflict() to conduct a conflict detection experiment.



#Path planning method
The files include AStar.h, AStar.cpp, Grid_3D.h, Grid_3D.cpp, main.cpp.
The future.obj is a city's data for experiment.

This section include 3 steps:
First, use triangleVoxelization() to gridding the scene.
Second, use initFlyArea() and init_GridPoint_Map() to create a grid map.
Finally, use sin_pathFinding_GridPoint() To complete the path finding.
The output is a series of coordinate points, Users can use out_vtk_pointpath() to get files in VTK format and display them with ParaView.
All the functions that need to be called are in Grid_3D.h.
Please refer to main.cpp for details.


