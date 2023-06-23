//
// Created by Jin on 31/05/2022.
//

#ifndef SYMMETRIZATION_NOISE_H
#define SYMMETRIZATION_NOISE_H


#include <vector>
#include <utility>
#include <algorithm>
#include <easy3d/core/types.h>
#include <easy3d/core/surface_mesh.h>
namespace easy3d {
    class Noise
    {

    public:
        void addNoise(SurfaceMesh *mesh, double noise_level);

    private:

        double generateRandomGaussian(double mean, double StandardDerivation);
        vec2 generateRandomDirection();

        void randomGaussianNumbers(double mean, double StandardDerivation, int number, std::vector<double> &RandomNumbers);

        void randomDirections(int number, std::vector<vec2> &RandomDirections);

    };
}



#endif //SYMMETRIZATION_NOISE_H
