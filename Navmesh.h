#ifndef NAVMESHCLASS_H
#define NAVMESHCLASS_H

#include <iostream>
#include <vector>
#include <map>
#include "SampleInterfaces.h"
#include "InputGeom.h"
#include "Sample_SoloMesh.h"
#include "Sample_TileMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_Debug.h"

#include "NavMeshTesterTool.h"

class Navmesh
{
public:
	Navmesh();
	~Navmesh();

	void init_by_obj(std::string file_path);
	void init_by_raw(std::vector<float> vertices, std::vector<int> faces);
	void build_navmesh();
	std::string get_log();  // clear ctx log after call this function
	std::vector<float> pathfind_straight(std::vector<float> start, std::vector<float> end, int vertex_mode = 0);  // return array of path point coordinates
	std::vector<float> pathfind_straight_batch(std::vector<float> coordinates, int vertex_mode = 0);
	float distance_to_wall(std::vector<float> point);
	std::vector<float> raycast(std::vector<float> start, std::vector<float> end);
	std::map<std::string, float> get_settings();
	void set_settings(std::map<std::string, float> settings);
	int get_partition_type();
	void set_partition_type(int type);
	std::vector<float> get_bounding_box();  // return 6-tuple of floats with geometry bounding box
	void save_navmesh(std::string file_path);
	void load_navmesh(std::string file_path);
	std::tuple<std::vector<float>, std::vector<int>> get_navmesh_trianglulation();  // return the pair ([vertices coordinates], [triangles point indexes])
	std::tuple<std::vector<float>, std::vector<int>> get_navmesh_trianglulation_sample();
	std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> get_navmesh_polygonization();  // return the tripple ([vertex coordinates], [polygon vertex indexes], [polygon sizes])
	std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> get_navmesh_polygonization_sample();
	std::vector<float> hit_mesh(std::vector<float> start, std::vector<float> end);

private:
	InputGeom* geom;
	Sample_SoloMesh* sample;
	BuildContext ctx;
	NavMeshTesterTool* tool;

	void clear();

	bool is_init;
	bool is_build;
};

#endif