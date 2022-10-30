#include "Navmesh.h"

#ifndef _MAIN_APP
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#ifdef _Python2
PYBIND11_MODULE(Py2RecastDetour, m)
#else
#ifdef _Python36
PYBIND11_MODULE(Py36RecastDetour, m)
#else
#ifdef _Python37
PYBIND11_MODULE(Py37RecastDetour, m)
#else
#ifdef _Python38
PYBIND11_MODULE(Py38RecastDetour, m)
#else
#ifdef _Python39
PYBIND11_MODULE(Py39RecastDetour, m)
#else
PYBIND11_MODULE(Py310RecastDetour, m)
#endif
#endif
#endif
#endif
#endif
{
	m.doc() = "Class allows to build navmesh from obj or raw vertices and polygons data. Also allows to find the shortest path between two points. Based on RecastNavigation c++ library https://github.com/recastnavigation/recastnavigation";
	py::class_<Navmesh>(m, "Navmesh")
		.def(py::init<>())
		.def("init_by_obj", &Navmesh::init_by_obj, py::arg("file_path"))
		.def("init_by_raw", &Navmesh::init_by_raw, py::arg("vertices"), py::arg("faces"))
		.def("build_navmesh", &Navmesh::build_navmesh)
		.def("get_log", &Navmesh::get_log)
		.def("pathfind_straight", &Navmesh::pathfind_straight, py::arg("start"), py::arg("end"), py::arg("vertex_mode") = 0)
		.def("pathfind_straight_batch", &Navmesh::pathfind_straight_batch, py::arg("coordinates"), py::arg("vertex_mode") = 0)
		.def("distance_to_wall", &Navmesh::distance_to_wall, py::arg("point"))
		.def("raycast", &Navmesh::raycast, py::arg("start"), py::arg("end"))
		.def("get_settings", &Navmesh::get_settings)
		.def("set_settings", &Navmesh::set_settings, py::arg("settings"))
		.def("get_partition_type", &Navmesh::get_partition_type)
		.def("set_partition_type", &Navmesh::set_partition_type, py::arg("type"))
		.def("get_bounding_box", &Navmesh::get_bounding_box)
		.def("save_navmesh", &Navmesh::save_navmesh, py::arg("file_path"))
		.def("load_navmesh", &Navmesh::load_navmesh, py::arg("file_path"))
		.def("get_navmesh_trianglulation", &Navmesh::get_navmesh_trianglulation_sample)
		.def("get_navmesh_polygonization", &Navmesh::get_navmesh_polygonization_sample)
		.def("hit_mesh", &Navmesh::hit_mesh, py::arg("start"), py::arg("end"));
}
#endif // !_MAIN_APP