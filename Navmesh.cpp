#include "Navmesh.h"
#include <algorithm>

Navmesh::Navmesh()
{
	//constructor
	geom = 0;
	sample = 0;
	tool = 0;
	is_build = false;
	is_init = false;
}

Navmesh::~Navmesh()
{
	//destructor
	clear();
}

void Navmesh::clear()
{
	delete geom;
	geom = 0;
	delete sample;
	sample = 0;
	ctx.resetLog();
	is_init = false;
	is_build = false;
}

std::string Navmesh::get_log()
{
	std::string to_return;
	for (size_t i = 0; i < ctx.getLogCount(); i++)
	{
		to_return += ctx.getLogText(i) + (i < ctx.getLogCount() - 1 ? std::string("\n") : "");
	}
	ctx.resetLog();

	return to_return;
}

std::map<std::string, float> Navmesh::get_settings()
{
	std::map<std::string, float> to_return;
	if (is_init)
	{
		BuildSettings settings;
		sample->collectSettings(settings);

		to_return["cellSize"] = settings.cellSize;
		to_return["cellHeight"] = settings.cellHeight;
		to_return["agentHeight"] = settings.agentHeight;
		to_return["agentRadius"] = settings.agentRadius;
		to_return["agentMaxClimb"] = settings.agentMaxClimb;
		to_return["agentMaxSlope"] = settings.agentMaxSlope;
		to_return["regionMinSize"] = settings.regionMinSize;
		to_return["regionMergeSize"] = settings.regionMergeSize;
		to_return["edgeMaxLen"] = settings.edgeMaxLen;
		to_return["edgeMaxError"] = settings.edgeMaxError;
		to_return["vertsPerPoly"] = settings.vertsPerPoly;  // this value should be <= 6 and >= 3
		to_return["detailSampleDist"] = settings.detailSampleDist;
		to_return["detailSampleMaxError"] = settings.detailSampleMaxError;
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Get settings: geometry is not initialized.");
	}
	return to_return;
}

void Navmesh::set_settings(std::map<std::string, float> settings)
{
	if (is_init)
	{
		std::map<std::string, float>::iterator it;

		for (it = settings.begin(); it != settings.end(); it++)
		{
			std::string k = it->first;
			float v = it->second;

			if (k == "cellSize") { sample->set_cell_size(std::max(v, 0.0001f)); }
			if (k == "cellHeight") { sample->set_cell_height(std::max(v, 0.0001f)); }
			if (k == "agentHeight") { sample->set_agent_height(std::max(v, 0.0f)); }
			if (k == "agentRadius") { sample->set_agent_radius(std::max(v, 0.0f)); }
			if (k == "agentMaxClimb") { sample->set_agent_max_climb(v); }
			if (k == "agentMaxSlope") { sample->set_agent_max_slope(v); }
			if (k == "regionMinSize") { sample->set_region_min_size(v); }
			if (k == "regionMergeSize") { sample->set_region_merge_size(v); }
			if (k == "edgeMaxLen") { sample->set_edge_max_len(v); }
			if (k == "edgeMaxError") { sample->set_edge_max_error(v); }
			if (k == "vertsPerPoly") { sample->set_verts_per_poly(std::max(3.0f, std::min(v, 6.0f))); }
			if (k == "detailSampleDist") { sample->set_detail_sample_dist(v); }
			if (k == "detailSampleMaxError") { sample->set_detail_sample_max_error(v); }
		}
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Set settings: geometry is not initialized.");
	}
}

int Navmesh::get_partition_type()
{
	if (is_init)
	{
		BuildSettings settings;
		sample->collectSettings(settings);
		return settings.partitionType;
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Get partition type: geometry is not initialized.");
	}
	return 0;
}

void Navmesh::set_partition_type(int type)
{
	if (is_init)
	{
		sample->set_partition_type(type);
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Set partition type: geometry is not initialized.");
	}
}

std::vector<float> Navmesh::get_bounding_box()
{
	std::vector<float> to_return(0);
	if (is_init)
	{
		to_return.resize(6);
		const float* min = geom->getMeshBoundsMin();
		const float* max = geom->getMeshBoundsMax();

		to_return[0] = min[0];
		to_return[1] = min[1];
		to_return[2] = min[2];

		to_return[3] = max[0];
		to_return[4] = max[1];
		to_return[5] = max[2];

		return to_return;
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Get navmesh bounding box: geometry is not initialized.");
		return to_return;
	}
}

void Navmesh::save_navmesh(std::string file_path)
{
	if (is_build)
	{
		//check is extension is *.bin
		size_t extensionPos = file_path.find_last_of('.');
		if (extensionPos == std::string::npos)
		{
			ctx.log(RC_LOG_ERROR, "Save navmesh: invalid file path.");
		}
		else
		{
			std::string extension = file_path.substr(extensionPos);
			std::transform(extension.begin(), extension.end(), extension.begin(), tolower);
			if (extension == ".bin")
			{
				sample->save_to_file(file_path.c_str());
			}
			else
			{
				ctx.log(RC_LOG_ERROR, "Save navmesh: invalid file extension (it should be *.bin).");
			}
		}
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Save navmesh: navmesh is not builded.");
	}
}

void Navmesh::load_navmesh(std::string file_path)
{
	if (is_init)
	{
		sample->load_from_file(file_path.c_str());
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Load navmesh: geometry is not initialized.");
	}
}

std::tuple<std::vector<float>, std::vector<int>> Navmesh::get_navmesh_trianglulation()
{
	if (is_build)
	{
		std::vector<float> vertices(0);
		std::vector<int> triangles(0);

		rcPolyMesh* pmesh = sample->get_m_pmesh();
		for (int v_index = 0; v_index < pmesh->nverts; v_index++) {
			vertices.push_back(pmesh->bmin[0] + pmesh->cs * pmesh->verts[3 * v_index]);
			vertices.push_back(pmesh->bmin[1] + pmesh->ch * pmesh->verts[3 * v_index + 1]);
			vertices.push_back(pmesh->bmin[2] + pmesh->cs * pmesh->verts[3 * v_index + 2]);
		}

		for (int p_index = 0; p_index < pmesh->npolys; p_index++) {
			int pv = p_index * 2 * pmesh->nvp;
			unsigned short v_start = pmesh->polys[pv];

			for (int j = 2; j < pmesh->nvp; j++) {
				unsigned short v = pmesh->polys[pv + j];
				if (v == 0xffff) {
					break;
				}
				unsigned short v_prev = pmesh->polys[pv + j - 1];
				triangles.push_back(v_start);
				triangles.push_back(v_prev);
				triangles.push_back(v);
			}
		}

		std::tuple<std::vector<float>, std::vector<int>> to_return = std::make_tuple(vertices, triangles);
		return to_return;
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Get navmesh trianglulation: navmesh is not builded.");
		std::vector<float> vertices(0);
		std::vector<int> triangles(0);
		std::tuple<std::vector<float>, std::vector<int>> to_return = std::make_tuple(vertices, triangles);
		return to_return;
	}
}

std::tuple<std::vector<float>, std::vector<int>> Navmesh::get_navmesh_trianglulation_sample()
{
	if (is_build)
	{
		std::vector<float> vertices(0);
		std::vector<int> triangles(0);

		dtNavMesh* navmesh = sample->getNavMesh();
		int max_tiles = navmesh->getMaxTiles();
		int start_tile_index = 0;
		for (size_t i = 0; i < max_tiles; i++)
		{
			const dtMeshTile* tile = navmesh->getTile(i);
			if (!tile->header) continue;

			for (size_t j = 0; j < tile->header->vertCount; j++)
			{
				vertices.push_back(tile->verts[3 * j]);
				vertices.push_back(tile->verts[3 * j + 1]);
				vertices.push_back(tile->verts[3 * j + 2]);
			}

			for (int j = 0; j < tile->header->polyCount; ++j)
			{
				const dtPoly* p = &tile->polys[j];
				if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// skip off-mesh links.
					continue;

				const dtPolyDetail* pd = &tile->detailMeshes[j];
				for (int k = 0; k < pd->triCount; ++k)
				{
					const unsigned char* t = &tile->detailTris[(pd->triBase + k) * 4];
					triangles.push_back(p->verts[t[0]] + start_tile_index);
					triangles.push_back(p->verts[t[1]] + start_tile_index);
					triangles.push_back(p->verts[t[2]] + start_tile_index);
				}
			}
			start_tile_index += tile->header->vertCount;
		}

		std::tuple<std::vector<float>, std::vector<int>> to_return = std::make_tuple(vertices, triangles);
		return to_return;
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Get navmesh trianglulation: navmesh is not builded.");
		std::vector<float> vertices(0);
		std::vector<int> triangles(0);
		std::tuple<std::vector<float>, std::vector<int>> to_return = std::make_tuple(vertices, triangles);
		return to_return;
	}
}

std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> Navmesh::get_navmesh_polygonization()
{
	if (is_build)
	{
		std::vector<float> vertices(0);
		std::vector<int> polygons(0);
		std::vector<int> sizes(0);

		rcPolyMesh* pmesh = sample->get_m_pmesh();
		for (int v_index = 0; v_index < pmesh->nverts; v_index++) {
			vertices.push_back(pmesh->bmin[0] + pmesh->cs * pmesh->verts[3 * v_index]);
			vertices.push_back(pmesh->bmin[1] + pmesh->ch * pmesh->verts[3 * v_index + 1]);
			vertices.push_back(pmesh->bmin[2] + pmesh->cs * pmesh->verts[3 * v_index + 2]);
		}

		for (int p_index = 0; p_index < pmesh->npolys; p_index++) {
			int pv = p_index * 2 * pmesh->nvp;
			unsigned short p_size = 0;
			for (int j = 0; j < pmesh->nvp; j++) {
				unsigned short v = pmesh->polys[pv + j];
				if (v == 0xffff) {
					break;
				}
				polygons.push_back(v);
				p_size++;
			}
			sizes.push_back(p_size);
		}

		std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> to_return = std::make_tuple(vertices, polygons, sizes);
		return to_return;
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Get navmesh polygonization: navmesh is not builded.");
		std::vector<float> vertices(0);
		std::vector<int> polygons(0);
		std::vector<int> sizes(0);
		std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> to_return = std::make_tuple(vertices, polygons, sizes);
		return to_return;
	}
}

std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> Navmesh::get_navmesh_polygonization_sample()
{
	if (is_build)
	{
		std::vector<float> vertices(0);
		std::vector<int> polygons(0);
		std::vector<int> sizes(0);

		dtNavMesh* navmesh = sample->getNavMesh();
		int max_tiles = navmesh->getMaxTiles();
		for (size_t i = 0; i < max_tiles; i++)
		{
			const dtMeshTile* tile = navmesh->getTile(i);
			if (!tile->header) continue;

			for (size_t j = 0; j < tile->header->vertCount; j++)
			{
				vertices.push_back(tile->verts[3 * j]);
				vertices.push_back(tile->verts[3 * j + 1]);
				vertices.push_back(tile->verts[3 * j + 2]);
			}

			for (int j = 0; j < tile->header->polyCount; ++j)
			{
				const dtPoly* p = &tile->polys[j];
				if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// skip off-mesh links.
					continue;

				for (int k = 0; k < p->vertCount; k++)
				{
					polygons.push_back(p->verts[k]);
				}
				sizes.push_back(p->vertCount);
			}
		}

		std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> to_return = std::make_tuple(vertices, polygons, sizes);
		return to_return;
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Get navmesh polygonization: navmesh is not builded.");
		std::vector<float> vertices(0);
		std::vector<int> polygons(0);
		std::vector<int> sizes(0);
		std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> to_return = std::make_tuple(vertices, polygons, sizes);
		return to_return;
	}
}

void Navmesh::init_by_raw(std::vector<float> vertices, std::vector<int> faces)
{
	if (is_init)
	{
		clear();
	}

	geom = new InputGeom;
	if (!geom->loadMesh_raw(&ctx, vertices, faces))
	{
		ctx.log(RC_LOG_ERROR, "Fail to load geometry from raw data.");
		clear();
	}
	else
	{
		sample = new Sample_SoloMesh();
		sample->setContext(&ctx);
		sample->handleMeshChanged(geom);
		sample->resetCommonSettings();
		tool = new NavMeshTesterTool();
		sample->setTool(tool);

		is_init = true;
	}
}

void Navmesh::init_by_obj(std::string file_path)
{
	if (is_init)
	{
		clear();
	}

	geom = new InputGeom;
	if (!geom->load(&ctx, file_path))
	{
		ctx.log(RC_LOG_ERROR, "Fail to load geometry.");
		clear();
	}
	else
	{
		sample = new Sample_SoloMesh();
		sample->setContext(&ctx);
		sample->handleMeshChanged(geom);
		sample->resetCommonSettings();
		tool = new NavMeshTesterTool();
		sample->setTool(tool);

		is_init = true;
	}
}

void Navmesh::build_navmesh()
{
	sample->handleSettings();
	if (!sample->handleBuild())
	{
		ctx.log(RC_LOG_ERROR, "Fail to build navmesh.");
		is_build = false;
	}
	else
	{
		is_build = true;
	}
}

std::vector<float> Navmesh::pathfind_straight(std::vector<float> start, std::vector<float> end, int vertex_mode)
{
	if (start.size() == 3 && end.size() == 3 && is_build)
	{
		tool->set_mode_streight(vertex_mode);

		tool->set_points(&start[0], &end[0]);

		int length = tool->get_path_points_count();
		float* coordinates = tool->get_path();
		std::vector<float> to_return(3 * length);
		for (size_t i = 0; i < length; i++)
		{
			to_return[3 * i] = coordinates[3 * i];
			to_return[3 * i + 1] = coordinates[3 * i + 1];
			to_return[3 * i + 2] = coordinates[3 * i + 2];
		}
		return to_return;
	}
	else
	{
		if (!is_build)
		{
			ctx.log(RC_LOG_ERROR, "Find straight path: navmesh is not builded.");
		}
		else
		{
			ctx.log(RC_LOG_ERROR, "Find straight path: invalid input vectors.");
		}
		
		std::vector<float> to_return(0);
		return to_return;
	}
}

std::vector<float> Navmesh::pathfind_straight_batch(std::vector<float> coordinates, int vertex_mode)
{
	if (coordinates.size() % 6 == 0 && is_build)
	{
		std::vector<float> to_return(0);
		// we assume that first 6 coordinates in the array define the first pair of start and end point, next 6 coordinates defines the second pair and so on
		for (size_t step = 0; step < coordinates.size() / 6; step++)
		{
			std::vector<float> start(3);
			std::vector<float> end(3);
			copy(coordinates.begin() + 6 * step, coordinates.begin() + 6 * step + 3, start.begin());
			copy(coordinates.begin() + 6 * step + 3, coordinates.begin() + 6 * step + 6, end.begin());

			// calculate the path
			std::vector<float> path = pathfind_straight(start, end, vertex_mode);

			// add calculated coordinates to the one output array
			// each result starts from the number of points (and then x3 floats for actual coordinates)
			to_return.push_back(path.size() / 3);
			to_return.insert(to_return.end(), path.begin(), path.end());
		}

		return to_return;
	}
	else
	{
		if (!is_build)
		{
			ctx.log(RC_LOG_ERROR, "Find straight path batch: navmesh is not builded.");
		}
		else
		{
			ctx.log(RC_LOG_ERROR, "Find straight path batch: invalid input vector with coordinates.");
		}

		std::vector<float> to_return(0);
		return to_return;
	}
}

float Navmesh::distance_to_wall(std::vector<float> point)
{
	if (is_build && point.size() == 3)
	{
		tool->set_mode_distance();
		tool->set_point(&point[0]);

		float to_return = tool->get_distance_to_wall();
		return to_return;
	}
	else
	{
		if (!is_build)
		{
			ctx.log(RC_LOG_ERROR, "Distance to wall: navmesh is not builded.");
		}
		else
		{
			ctx.log(RC_LOG_ERROR, "Distance to wall: invalid input vector.");
		}
	}
	return 0.0f;
}

std::vector<float> Navmesh::raycast(std::vector<float> start, std::vector<float> end)
{
	if (start.size() == 3 && end.size() == 3 && is_build)
	{
		tool->set_mode_raycast();

		tool->set_points(&start[0], &end[0]);

		int length = tool->get_path_points_count();
		float* coordinates = tool->get_path();
		std::vector<float> to_return(3 * length);
		for (size_t i = 0; i < length; i++)
		{
			to_return[3 * i] = coordinates[3 * i];
			to_return[3 * i + 1] = coordinates[3 * i + 1];
			to_return[3 * i + 2] = coordinates[3 * i + 2];
		}
		return to_return;
	}
	else
	{
		if (is_build)
		{
			ctx.log(RC_LOG_ERROR, "Raycast: navmesh is not builded.");
		}
		else
		{
			ctx.log(RC_LOG_ERROR, "Raycast: invalid input vectors.");
		}

		std::vector<float> to_return(0);
		return to_return;
	}
}

std::vector<float> Navmesh::hit_mesh(std::vector<float> start, std::vector<float> end)
{
	if (start.size() == 3 && end.size() == 3)
	{
		if (is_init)
		{
			float hit_time;
			bool hit = geom->raycastMesh(&start[0], &end[0], hit_time);
			if (hit)
			{
				std::vector<float> to_return(3);
				for (int i = 0; i < to_return.size(); i++)
				{
					to_return[i] = start[i] + (end[i] - start[i]) * hit_time;
				}
				return to_return;
			}
			else
			{
				return end;
			}
		}
		else
		{
			ctx.log(RC_LOG_ERROR, "Hit mesh: geometry is not initialized.");
		}
	}
	else
	{
		ctx.log(RC_LOG_ERROR, "Hit mesh: invalid input vectors.");
	}
}

#ifdef _MAIN_APP
void main() {
	Navmesh* navmesh = new Navmesh();
	navmesh->init_by_obj("disc.obj");
	std::map<std::string, float> settings = navmesh->get_settings();
	settings["vertsPerPoly"] = 12;
	settings["cellSize"] = 0.1;

	navmesh->set_settings(settings);
	navmesh->build_navmesh();

	std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> mesh = navmesh->get_navmesh_polygonization();

	std::string verts_str = "vertices: ";
	std::vector<float> vertices = std::get<0>(mesh);
	for (size_t i = 0; i < vertices.size(); i++) {
		verts_str += std::to_string(vertices[i]) + ", ";
	}
	//std::cout << verts_str << std::endl;

	std::string polys_str = "polygons: ";
	std::vector<int> polygons = std::get<1>(mesh);
	for (size_t i = 0; i < polygons.size(); i++) {
		polys_str += std::to_string(polygons[i]) + ", ";
	}
	//std::cout << polys_str << std::endl;

	std::string sizes_str = "sizes: ";
	std::vector<int> sizes = std::get<2>(mesh);
	for (size_t i = 0; i < sizes.size(); i++) {
		sizes_str += std::to_string(sizes[i]) + ", ";
	}
	std::cout << sizes_str << std::endl;
}
#endif // _MAIN_APP