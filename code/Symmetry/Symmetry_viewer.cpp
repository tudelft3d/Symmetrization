/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Symmetry_viewer.h"
#include "../method/Optimisation.h"
#include "easy3d/algo/tessellator.h"
#include "easy3d/core/surface_mesh_builder.h"
#include "../method/Noise.h"
#include "method/bounding_sphere.h"
#include "easy3d/core/graph.h"
#include "easy3d/fileio/graph_io.h"
#include <easy3d/core/surface_mesh.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/texture_manager.h>
#include <easy3d/renderer/setting.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/state.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <3rd_party/glfw/include/GLFW/glfw3.h>    // for the KEYs
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <unistd.h>

using namespace easy3d;
typedef std::vector<vec3> Hole;
std::vector<std::pair<int, int>> Paired_Edges;
std::vector<Hole> hole_set;
vec3 center(0, 0, 0);

SymmetryViewer::SymmetryViewer(const std::string &title) : Viewer(title)
{
    camera()->setUpVector(vec3(0, 1, 0));
    camera()->setViewDirection(vec3(0, 0, -1));
    camera_->showEntireScene();

}

// convert the mesh into triangular mesh using the tessellator
void triangulate(SurfaceMesh *mesh)
{
    if (!mesh)
        return;

    mesh->update_face_normals();
    auto normals = mesh->face_property<vec3>("f:normal");

    Tessellator tessellator;
    for (auto f: mesh->faces())
    {
        tessellator.begin_polygon(normals[f]);

        tessellator.set_winding_rule(Tessellator::WINDING_NONZERO);
        tessellator.begin_contour();
        for (auto h: mesh->halfedges(f))
        {
            SurfaceMesh::Vertex v = mesh->target(h);
            tessellator.add_vertex(mesh->position(v), v.idx());
        }
        tessellator.end_contour();
        for (auto hole: hole_set)
        {
            tessellator.set_winding_rule(Tessellator::WINDING_ODD);
            tessellator.begin_contour();
            for (auto p: hole)
                tessellator.add_vertex(p);
            tessellator.end_contour();
        }
        tessellator.end_polygon();
    }

    // now the tessellation is done. We can clear the old mesh and
    // fill it will the new set of triangles

    mesh->clear();

    const auto &triangles = tessellator.elements();
    if (!triangles.empty())
    { // in degenerate cases num can be zero
        const std::vector<Tessellator::Vertex *> &vts = tessellator.vertices();
        for (auto v: vts)
        {
            mesh->add_vertex(vec3(v->data()));
        }

        for (const auto &t: triangles)
        {
            mesh->add_triangle(SurfaceMesh::Vertex(t[0]), SurfaceMesh::Vertex(t[1]), SurfaceMesh::Vertex(t[2]));
        }
    }
}

Model *SymmetryViewer::add_model(Model *model, bool create)
{
    if (Viewer::add_model(model, create))
    {
        auto mesh = dynamic_cast<SurfaceMesh *>(current_model());
        if (mesh->n_faces())
        {
            auto edges = model->renderer()->get_lines_drawable("edges");

            if (edges)
            {
                edges->set_visible(true);
                edges->set_line_width(5);
                edges->set_impostor_type(LinesDrawable::CYLINDER);
                edges->set_uniform_coloring(vec4(1, 0, 0, 0));
            }
        } else
        {
            //we need do tesselation, just show borders
            std::cout << "tesselation" << std::endl;
            auto edges = model->renderer()->get_lines_drawable("borders");
            if (edges)
            {
                edges->set_visible(true);
                edges->set_line_width(5);
                edges->set_impostor_type(LinesDrawable::CYLINDER);
            }
        }

        auto vertices = model->renderer()->get_points_drawable("vertices");
        if (vertices)
        {
            vertices->set_visible(true);
            vertices->set_point_size(16);
            vertices->set_uniform_coloring(vec4(0, 1, 0, 0));
        }
        auto faces = model->renderer()->get_triangles_drawable("faces");
        faces->set_visible(true);
        if (faces)
        {
            faces->set_distinct_back_color(false);
        }
        update();
    }
    return model;
}

std::string SymmetryViewer::usage() const
{

    return ("----------- Symmetrization usage ------------ \n"
            "'<': show input model\n"
            "'>': show current model\n"
            "'R': the symmetry axis of shape is aligned with y-axis \n"
            "'Q': the symmetry axis of shape is arbitrary\n"
            "'H': enforce horizontal and vertical constraints \n"
            "------------------------------------------------ \n");
}
vec2 computeIntersection(const vec2& point1, const vec2& direction1, const vec2& point2, const vec2& direction2) {
    double cross = direction1.x * direction2.y - direction1.y * direction2.x;

    if (cross == 0) {
        // Lines are parallel or collinear, no unique intersection
        return { 0.0, 0.0 }; // You can return a default value or throw an exception here
    } else {
        double factor = ((point2.x - point1.x) * direction2.y - (point2.y - point1.y) * direction2.x) / cross;
        double intersectionX = point1.x + factor * direction1.x;
        double intersectionY = point1.y + factor * direction1.y;
        return vec2(intersectionX, intersectionY );
    }
}
bool SymmetryViewer::key_press_event(int key, int modifiers)
{
    auto mesh = dynamic_cast<SurfaceMesh *>(current_model());
    if (key == GLFW_KEY_R)
    {
        if (!mesh)
        {
            LOG(ERROR) << "model does not exist" << std::endl;
            return false;
        }

        auto orig_points = mesh->get_vertex_property<vec3>("v:input");
        if (!orig_points)
        {
            orig_points = mesh->add_vertex_property<vec3>("v:input");
            orig_points.vector() = mesh->points();
        }
        mesh->points() = orig_points.vector();
        //transfer the mesh points
        SurfaceMesh mesh_copy = *mesh;
        vec3 poly_center;
        BoundingSphere bsphere;
        poly_center = bsphere.compute_center(mesh->points());

        auto points = mesh_copy.get_vertex_property<vec3>("v:point");
        for (auto v: mesh_copy.vertices())
        {
            auto p_temp = points[v];
            p_temp -= poly_center;
            points[v].x = p_temp.x;
            points[v].y = p_temp.y;
        }
        // use the current geometry for further processing (if the original model is being visualized).
        auto output = mesh->get_vertex_property<vec3>("v:output");
        SurfaceMesh *mesh_iter = &mesh_copy;
        SurfaceMeshIO::save("./result_before.obj", mesh_iter);
        SurfaceMeshSymmetry v0(mesh_iter, false);
        v0.optimize_vertices();
        auto edge_paired = mesh_iter->get_face_property<std::vector<std::pair<int, int>>>("e:edge_paired");
        Paired_Edges = *edge_paired.data();
        auto pts = mesh_iter->points();
        if (Paired_Edges.size())
        {
            auto p1 = Paired_Edges[0];
            auto s = mesh->vertex(SurfaceMesh::Edge(p1.first), 0).idx();
            auto t = mesh->vertex(SurfaceMesh::Edge(p1.first), 1).idx();
            auto s1 = pts[s], t1 = pts[t];

            auto se = mesh->vertex(SurfaceMesh::Edge(p1.second), 0).idx();
            auto te = mesh->vertex(SurfaceMesh::Edge(p1.second), 1).idx();
            auto se1 = pts[se], te1 = pts[te];
            center = ((s1 + t1 + se1 + te1) / 4);
            center += poly_center;
        }
        auto unused = mesh_iter->get_edge_property<bool>("e:unpaired");
        for (auto e: mesh_iter->edges())
        {
            if (unused[e])
                unpaired.push_back(e.idx());
        }

        // record the new geometry
        if (!output)
            output = mesh->vertex_property<vec3>("v:output");
        SurfaceMeshIO::save("./result_transfer.obj", mesh_iter);
        auto new_pts = mesh_iter->get_vertex_property<vec3>("v:point");
        for (auto v: mesh_iter->vertices())
        {
            auto p_temp = new_pts[v];
            new_pts[v].x = p_temp.x;
            new_pts[v].y = p_temp.y;
            new_pts[v] += poly_center;
        }
        output.vector() = mesh_iter->points();
        mesh->points() = output.vector();
        SurfaceMeshIO::save("./result.obj", mesh);
        mesh->renderer()->update();
        return false;
    } else if (key == GLFW_KEY_H)
    {
        if (!mesh)
        {
            LOG(ERROR) << "model does not exist" << std::endl;
            return false;
        }

        auto orig_points = mesh->get_vertex_property<vec3>("v:input");
        if (!orig_points)
        {
            orig_points = mesh->add_vertex_property<vec3>("v:input");
            orig_points.vector() = mesh->points();
        }
        mesh->points() = orig_points.vector();
        //transfer the mesh points
        SurfaceMesh mesh_copy = *mesh;
        vec3 poly_center;
        BoundingSphere bsphere;
        poly_center = bsphere.compute_center(mesh->points());
        auto points = mesh_copy.get_vertex_property<vec3>("v:point");
        for (auto v: mesh_copy.vertices())
        {
            auto p_temp = points[v];
            p_temp -= poly_center;
            points[v].x = p_temp.x;
            points[v].y = p_temp.y;
        }
        // use the current geometry for further processing (if the original model is being visualized).
        auto output = mesh->get_vertex_property<vec3>("v:output");
        SurfaceMesh *mesh_iter = &mesh_copy;
        SurfaceMeshSymmetry v0(mesh_iter, true);
        auto obj_val = v0.optimize_vertices();
        auto edge_paired = mesh_iter->get_face_property<std::vector<std::pair<int, int>>>("e:edge_paired");
        Paired_Edges = *edge_paired.data();
        auto pts = mesh_iter->points();
        if (Paired_Edges.size())
        {
            auto p1 = Paired_Edges[0];
            auto s = mesh->vertex(SurfaceMesh::Edge(p1.first), 0).idx();
            auto t = mesh->vertex(SurfaceMesh::Edge(p1.first), 1).idx();
            auto s1 = pts[s], t1 = pts[t];

            auto se = mesh->vertex(SurfaceMesh::Edge(p1.second), 0).idx();
            auto te = mesh->vertex(SurfaceMesh::Edge(p1.second), 1).idx();
            auto se1 = pts[se], te1 = pts[te];
            center = ((s1 + t1 + se1 + te1) / 4);
            center += poly_center;
        }
        auto unused = mesh_iter->get_edge_property<bool>("e:unpaired");
        for (auto e: mesh_iter->edges())
        {
            if (unused[e])
                unpaired.push_back(e.idx());
        }

        // record the new geometry
        if (!output)
            output = mesh->vertex_property<vec3>("v:output");
        auto new_pts = mesh_iter->get_vertex_property<vec3>("v:point");
        for (auto v: mesh_iter->vertices())
        {
            auto p_temp = new_pts[v];
            new_pts[v].x = p_temp.x;
            new_pts[v].y = p_temp.y;
            new_pts[v] += poly_center;
        }
        output.vector() = mesh_iter->points();
        mesh->points() = output.vector();
        mesh->renderer()->update();
        return false;
    } else if (key == GLFW_KEY_Q)
    {
        if (!mesh)
        {
            LOG(ERROR) << "model does not exist" << std::endl;
            return false;
        }
        double obj_min = std::numeric_limits<double>::max();
        auto orig_points = mesh->get_vertex_property<vec3>("v:input");
        if (!orig_points)
        {
            orig_points = mesh->add_vertex_property<vec3>("v:input");
            orig_points.vector() = mesh->points();
        }
        mesh->points() = orig_points.vector();
        //compute the bounding center
        vec3 poly_center;
        BoundingSphere bsphere;
        poly_center = bsphere.compute_center(mesh->points());

        for (int i = 0; i < 18; ++i)
        {
            theta = i * M_PI / 18;
            SurfaceMesh mesh_copy = *mesh;
            auto points = mesh_copy.get_vertex_property<vec3>("v:point");
            for (auto v: mesh_copy.vertices())
            {
                auto p_temp = orig_points[v];
                p_temp -= poly_center;
                points[v].x = std::cos(theta) * p_temp.x + std::sin(theta) * p_temp.y;
                points[v].y = std::cos(theta) * p_temp.y - std::sin(theta) * p_temp.x;
            }
            // use the current geometry for further processing (if the original model is being visualized).
            auto output = mesh->get_vertex_property<vec3>("v:output");
            SurfaceMesh *mesh_iter = &mesh_copy;
            SurfaceMeshIO::save("./"+std::to_string(i) +"result_before.obj", mesh_iter);
            SurfaceMeshSymmetry v0(mesh_iter, false);

            auto obj_val = v0.optimize_vertices();
            auto edge_paired = mesh_iter->get_face_property<std::vector<std::pair<int, int>>>("e:edge_paired");
            Paired_Edges = *edge_paired.data();

            auto unused = mesh_iter->get_edge_property<bool>("e:unpaired");
            for (auto e: mesh_iter->edges())
            {
                if (unused[e])
                    unpaired.push_back(e.idx());
            }

            if (obj_val < obj_min)
            {
                obj_min = obj_val;
                optimal_theta = theta;
                auto vts = mesh_iter->get_vertex_property<vec3>("v:point");
                for (auto v: mesh_iter->vertices())
                {
                    auto p_temp = vts[v];
                    vts[v].x = std::cos(theta) * p_temp.x - std::sin(theta) * p_temp.y;
                    vts[v].y = std::cos(theta) * p_temp.y + std::sin(theta) * p_temp.x;
                    vts[v] += poly_center;
                }

                // record the new geometry
                if (!output)
                    output = mesh->vertex_property<vec3>("v:output");
                SurfaceMeshIO::save("./"+std::to_string(i) +"result_after.obj", mesh_iter);
                output.vector() = mesh_iter->points();
                mesh->points() = output.vector();
                mesh->renderer()->update();
                if (Paired_Edges.size())
                {
                    auto pts= mesh->points();
                    auto p1 = Paired_Edges[0];
                    auto s = mesh->vertex(SurfaceMesh::Edge(p1.first), 0).idx();
                    auto t = mesh->vertex(SurfaceMesh::Edge(p1.first), 1).idx();
                    auto s1 = pts[s], t1 = pts[t];

                    auto se = mesh->vertex(SurfaceMesh::Edge(p1.second), 0).idx();
                    auto te = mesh->vertex(SurfaceMesh::Edge(p1.second), 1).idx();
                    auto se1 = pts[se], te1 = pts[te];
                    center = ((s1 + t1 + se1 + te1) / 4);
                }
            }
            std::cout << i << " optimal theta: " << optimal_theta * 57.3 << "  min energy: " << obj_min << std::endl;
        }
        std::cout<<"completed"<<std::endl;
        return false;
    } else if (key == GLFW_KEY_COMMA)
    {
        auto input = mesh->vertex_property<vec3>("v:input");
        mesh->points() = input.vector();
        mesh->renderer()->update();
        return false;
    } else if (key == GLFW_KEY_PERIOD)
    {
        auto output = mesh->vertex_property<vec3>("v:output");
        mesh->points() = output.vector();
        mesh->renderer()->update();
        return false;
    } else if (key == GLFW_KEY_O)
    {
        // Get the bounding box of the model. Then we defined the length of the
        // normal vectors to be 5% of the bounding box diagonal.
        const Box3 &box = mesh->bounding_box();
        float length = norm(box.max_point() - box.min_point()) * 0.75;
        // Compute the face normals.
        mesh->update_face_normals();
        // Every consecutive two points represent a normal vector.
        std::vector<vec3> points;
        {
            auto pts = mesh->points();
            vec2 p(-std::sin(optimal_theta), std::cos(optimal_theta));
            vec2 s = center;
            double y_max = -1e10, y_min = 1e10;
            for (auto pt: pts)
            {
                if (pt.y > y_max)
                    y_max = pt.y;
                if (pt.y < y_min)
                    y_min = pt.y;
            }
            vec2 t = s + p * length;
            vec2 t1 = s - p * length;

            points.push_back(vec3 (t1,0.5));
            points.push_back(vec3(t,0.5));

        }
        // Create a drawable for rendering the normal vectors.
        auto drawable = mesh->renderer()->add_lines_drawable("normals");
        // Upload the data to the GPU.
        drawable->update_vertex_buffer(points);
        // We will draw the normal vectors in a uniform green color
        drawable->set_uniform_coloring(vec4(0.0f, 0.0f, 1.0f, 1.0f));
        // Set the line width
        drawable->set_line_width(5.0f);
        // Also show the standard "edges"
        mesh->renderer()->get_lines_drawable("edges")->set_visible(true);
        return false;
    } else if (key == GLFW_KEY_M)
    {
        auto pts = mesh->get_vertex_property<vec3>("v:point");
        Noise ns;
        ns.addNoise(mesh, 0.05);
        SurfaceMeshIO::save("./" + std::to_string(1) + ".obj", mesh);
        mesh->renderer()->update();
        std::cout << "add noise" << std::endl;

    } else if (key == GLFW_KEY_T)
    {
        SurfaceMesh *new_mesh = new SurfaceMesh;;
        auto points = mesh->points();
        if (mesh->n_faces() > 1)
        {
            auto f_begin = mesh->faces().begin();
            std::vector<SurfaceMesh::Vertex> vts;
            for (auto v: mesh->vertices(*f_begin))
            {
                SurfaceMesh::Vertex s1 = new_mesh->add_vertex(points[v.idx()]);
                vts.push_back(s1);
            }
            new_mesh->add_face(vts);

            for (auto f: mesh->faces())
            {
                std::vector<vec3> holes_points;
                if (f != *mesh->faces().begin())
                {
                    for (auto v: mesh->vertices(f))
                    {
                        holes_points.push_back(points[v.idx()]);
                    }
                }
                hole_set.push_back(holes_points);
            }

        }
        triangulate(new_mesh);
        add_model(new_mesh, true);
        mesh->clear();
        mesh->renderer()->update();
    } else
        return Viewer::key_press_event(key, modifiers);
}