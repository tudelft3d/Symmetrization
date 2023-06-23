//
// Created by Jin on 07/04/2023.
//

#ifndef SYMMETRIZATION_BOUNDING_SPHERE_H
#define SYMMETRIZATION_BOUNDING_SPHERE_H
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_2.h>
#include <CGAL/Random.h>
#include <iostream>
#include "easy3d/core/types.h"

typedef  CGAL::Simple_cartesian<double>                   K;
typedef  CGAL::Min_sphere_of_points_d_traits_2<K,double>  Traits;
typedef  CGAL::Min_sphere_of_spheres_d<Traits>            Min_circle;
typedef  K::Point_2                                       Point;
namespace easy3d{
    class BoundingSphere{
    public:
        BoundingSphere(){};
        ~BoundingSphere(){};
        vec3 compute_center(std::vector<vec3> vts)
        {
            std::vector<Point> pts;
            for ( int i = 0; i < vts.size(); ++i){
                pts.push_back(Point(vts[i].x, vts[i].y)) ;
            }
            Min_circle  mc( pts.begin(), pts.end());
            Min_circle::Cartesian_const_iterator ccib = mc.center_cartesian_begin();

            return vec3(*ccib, *(ccib+1), 0);
        }

    };
}

#endif //SYMMETRIZATION_BOUNDING_SPHERE_H
