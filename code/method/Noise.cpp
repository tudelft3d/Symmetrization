//
// Created by Jin on 31/05/2022.
//
#include "Noise.h"
namespace easy3d{
    vec2  pro_3d(vec3 a)
    {
        return  vec2 (a.x,a.y);
    };
    vec3  repro_2d(vec2 a)
    {
        return  vec3 (a.x,a.y,0);
    };

    void Noise::addNoise(SurfaceMesh *mesh, double noise_level)
    {
        // compute average length of mesh
        double average_length = 0.0;
        for (auto e:mesh->edges())
        {
            SurfaceMesh::Vertex s=mesh->vertex(e, 0);
            SurfaceMesh::Vertex t=mesh->vertex(e, 1);
            auto vs=mesh->position(s), vt=mesh->position(t);
            average_length+=(vs-vt).length();
        }
        average_length/=mesh->n_edges();
        // add noise
        double standard_derivation = average_length * noise_level;
        auto pts = mesh->get_vertex_property<vec3>("v:point");
        std::vector<double> GaussianNumbers;
        std::vector<vec2> RandomDirections;
        randomGaussianNumbers(0, standard_derivation, (int) mesh->n_vertices(), GaussianNumbers);
        randomDirections((int) mesh->n_vertices(), RandomDirections);
        for (auto v_t : mesh->vertices())
        {
            int index = v_t.idx();
            auto p = pro_3d(mesh->position(v_t)) + RandomDirections[index] * GaussianNumbers[index];
            pts[v_t]= repro_2d(p);
        }
    }

    double Noise::generateRandomGaussian(double mean, double StandardDerivation)
    {
        static double v1, v2, s;
        static int phase = 0;
        double x;
        if (phase == 0)
        {
            do
            {
                v1 = -1 + 2 * (double) rand() / (double) RAND_MAX;
                v2 = -1 + 2 * (double) rand() / (double) RAND_MAX;
                s = v1 * v1 + v2 * v2;
            } while (s >= 1 || s == 0);

            x = v1 * sqrt(-2 * log(s) / s);
        } else
            x = v2 * sqrt(-2 * log(s) / s);

        phase = 1 - phase;
        return x * StandardDerivation + mean;
    }

    vec2 Noise::generateRandomDirection()
    {
        double x, y,  length;
        do
        {
            x = -1 + 2 * (double) rand() / (double) RAND_MAX;
            y = -1 + 2 * (double) rand() / (double) RAND_MAX;
            length = x * x + y * y;
        } while (length > 1);

        const double r = 2 * std::sqrt(1 - length);

        x *= r;
        y *= r;
        return vec2(x, y);
    }

    void Noise::randomGaussianNumbers(double mean, double StandardDerivation, int number, std::vector<double> &RandomNumbers)
    {
        RandomNumbers.resize(number, 0.0);

        srand((unsigned int) time(NULL));
        for (int i = 0; i < number; i++)
        {
            RandomNumbers[i] = generateRandomGaussian(mean, StandardDerivation);
        }
    }


    void Noise::randomDirections(int number, std::vector<vec2> &RandomDirections)
    {
        RandomDirections.resize(number, vec2(0.0, 0.0));

        srand((unsigned int) time(NULL));
        for (int i = 0; i < number; i++)
        {
            RandomDirections[i] = generateRandomDirection();
        }
    }
}

