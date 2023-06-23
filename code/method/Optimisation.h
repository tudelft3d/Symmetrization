//
// Created by jinhuang on 02-09-21.
//

#ifndef EASY3D_SYMMETRY_OPTIMISATION_H_
#define EASY3D_SYMMETRY_OPTIMISATION_H_

#include <easy3d/core/types.h>
#include <easy3d/core/surface_mesh.h>
#include <string>
namespace easy3d {

	class SurfaceMeshSymmetry {
	 public:
		SurfaceMeshSymmetry(SurfaceMesh *mesh, bool extra_constraints);
		~SurfaceMeshSymmetry(void);
		double optimize_vertices();

	 private:
		SurfaceMesh *mesh_;
		//use mixed integer nonlinear programming method to select symmetric edge pairs
		std::vector<double> result_;
        bool vertical_horizontal_constraints;

	};

} // namespace easy3d


#endif //EASY3D_SYMMETRY_OPTIMISATION_H_
