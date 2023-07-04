//
// Created by jinhuang on 28-09-21.
//
#include "Symmetry_viewer.h"
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/resources.h>
#include <easy3d/util/logging.h>
#include <easy3d/fileio/surface_mesh_io.h>

int main(int argc, char** argv)
{

	std::string file = "butterfly";
	const std::string file_name =  "../../data/" + file + ".obj";
	SymmetryViewer viewer;
	auto model = viewer.Viewer::add_model(file_name, true);
	if (!model)
	{
		LOG(ERROR) << "Error: failed to load model. Please make sure the file exists and format is correct.";
		return EXIT_FAILURE;
	}
	viewer.set_background_color(easy3d::vec4(1, 1, 1, 0));
	return viewer.run();
}


