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
#include "easy3d/renderer/shapes.h"
#include <3rd_party/imgui/misc/fonts/imgui_fonts_droid_sans.h>
#include <3rd_party/imgui/imgui.h>
#include <3rd_party/imgui/impl/imgui_impl_glfw.h>
#include <3rd_party/imgui/impl/imgui_impl_opengl3.h>
#include <3rd_party/glfw/include/GLFW/glfw3.h>
#include <easy3d/renderer/text_renderer.h>

using namespace easy3d;
typedef std::vector<vec3> Hole;
std::vector<std::pair<int, int>> Paired_Edges;
std::vector<Hole> hole_set;
vec3 center(0, 0, 0);

ImGuiContext *SymmetryViewer::context_ = nullptr;

SymmetryViewer::SymmetryViewer(
        const std::string &title /* = "Easy3D ImGui Viewer" */,
        int samples /* = 4 */,
        int gl_major /* = 3 */,
        int gl_minor /* = 2 */,
        bool full_screen /* = false */,
        bool resizable /* = true */,
        int depth_bits /* = 24 */,
        int stencil_bits /* = 8 */
)
        : Viewer(title, samples, gl_major, gl_minor, full_screen, resizable, depth_bits, stencil_bits),
           texter_(nullptr), font_size_delta_(0.0f), line_spacing_(0.0f),
          alignment_(TextRenderer::ALIGN_LEFT), upper_left_(true)
{
    camera()->setUpVector(vec3(0, 1, 0));
    camera()->setViewDirection(vec3(0, 0, -1));
    camera_->showEntireScene();
}

void SymmetryViewer::draw_menu_file()
{
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Open", "Ctrl+O"))
            open();
        if (ImGui::MenuItem("Save As...", "Ctrl+S"))
            save();

        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Alt+F4"))
            glfwSetWindowShouldClose(window_, GLFW_TRUE);

        ImGui::EndMenu();
    }
}


void SymmetryViewer::draw_menu_view()
{
    if (ImGui::BeginMenu("View"))
    {
        if (ImGui::MenuItem("Snapshot", nullptr))
            snapshot();

        ImGui::Separator();

        ImGui::EndMenu();
    }
}

void SymmetryViewer::post_draw()
{
    //add text

    const float font_size = 28.0f + font_size_delta_;
    float x = 20.0f;
    float y = 40.0f;

    const auto num_fonts = texter_->num_fonts();
    const float font_height = texter_->font_height(font_size);

    texter_->draw(
            "Drop a file to the program to open a model, or\n"
            "manually create a model by Ctrl + clicking.  \n"
            "----------------------------------------------------- \n"
            "Press 'R' to symmetrize the model.",
            x * dpi_scaling(), y * dpi_scaling(), 20, TextRenderer::Align(alignment_), 0, vec3(0, 0, 0),
            line_spacing_, upper_left_);

    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 8));
    if (ImGui::BeginMainMenuBar())
    {
        draw_menu_file();

        draw_menu_view();

        ImGui::EndMainMenuBar();
    }
    ImGui::PopStyleVar();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (polygon_.size() >= 2)
    {
        // draw the boundary of the rect/lasso

        shapes::draw_polygon_wire(polygon_, vec4(1.0f, 0.0f, 0.0f, 1.0f), width(), height(), -1.0f);
        if (polygon_.size() >= 3)
        {
            // draw its transparent face
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            shapes::draw_polygon_filled(polygon_, vec4(1.0f, 0.0f, 0.0f, 0.2f), width(), height(), -0.9f);
            glDisable(GL_BLEND);
        }
    }
    if (real_time_polygon_.size()>=2)
        shapes::draw_polygon_wire(real_time_polygon_, vec4(0.0f, 1.0f, 0.0f, 1.0f), width(), height(), -1.0f);

}

void SymmetryViewer::init()
{
    Viewer::init();
    texter_ = new TextRenderer(dpi_scaling());
    texter_->add_font("../../code/resources/fonts/en_Roboto-Medium.ttf");

    if (!context_)
    {
        // Setup ImGui binding
        IMGUI_CHECKVERSION();

        context_ = ImGui::CreateContext();

        const char *glsl_version = "#version 150";
        ImGui_ImplGlfw_InitForOpenGL(window_, true);
        ImGui_ImplOpenGL3_Init(glsl_version);
        ImGuiIO &io = ImGui::GetIO();
        io.WantCaptureKeyboard = true;
        io.WantTextInput = true;
        io.IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGuiStyle &style = ImGui::GetStyle();
        style.FrameRounding = 5.0f;

        // load font
        reload_font();
    }
}


double SymmetryViewer::pixel_ratio()
{
    // Computes pixel ratio for hidpi devices
    int fbo_size[2], win_size[2];
    glfwGetFramebufferSize(window_, &fbo_size[0], &fbo_size[1]);
    glfwGetWindowSize(window_, &win_size[0], &win_size[1]);
    return static_cast<double>(fbo_size[0]) / static_cast<double>(win_size[0]);
}


void SymmetryViewer::reload_font(int font_size)
{
    ImGuiIO &io = ImGui::GetIO();
    io.Fonts->Clear();
    io.Fonts->AddFontFromMemoryCompressedTTF(droid_sans_compressed_data, droid_sans_compressed_size,
                                             font_size * dpi_scaling());
    io.FontGlobalScale = 1.0f / pixel_ratio();
    ImGui_ImplOpenGL3_DestroyDeviceObjects();
}


void SymmetryViewer::post_resize(int w, int h)
{
    Viewer::post_resize(w, h);
    if (context_)
    {
        ImGui::GetIO().DisplaySize.x = float(w);
        ImGui::GetIO().DisplaySize.y = float(h);
    }
}


void SymmetryViewer::cleanup()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImGui::DestroyContext(context_);

    Viewer::cleanup();
}


void SymmetryViewer::pre_draw()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    Viewer::pre_draw();
}


//    ImGui::Text("Drag the existing model to the program or  \n"
//    "Manually create the 2D polygonal model");
//    ImGui::Separator();
//    ImGui::Text("Press 'R' to symmetrize the shape or \n"
//    "Press 'H' to add horizontal and vertical constants to the shape");

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
            "'>': show output model\n"
            "'D': delete current model\n"
            "'R': the symmetry axis of shape is aligned with y-axis \n"
            "'Q': the symmetry axis of shape is arbitrary\n"
            "'M': manually create 2D polygonal shape\n"
            "'H': enforce horizontal and vertical constraints \n"
            "------------------------------------------------ \n");
}

bool SymmetryViewer::mouse_release_event(int x, int y, int button, int modifiers)
{
    if ((pickable_) && button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (real_time_polygon_.size() >= 2)
        {
            polygon_ = real_time_polygon_;
        }

    }

}

bool SymmetryViewer::mouse_press_event(int x, int y, int button, int modifiers)
{

    if ((pickable_) && button == GLFW_MOUSE_BUTTON_LEFT)
    {
        float x = ImGui::GetIO().MousePos.x;
        float y = ImGui::GetIO().MousePos.y;
        polygon_.push_back(vec2(x, y));
        return true;
    } else if ( button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        SurfaceMesh *new_mesh = new SurfaceMesh();
        SurfaceMeshBuilder builder(new_mesh);
        std::vector<SurfaceMesh::Vertex> vts;
        const int win_width = camera()->screenWidth();
        const int win_height = camera()->screenHeight();
        //add vertices
        builder.begin_surface();
        for (auto p: polygon_)
        {
            auto x0 = p.x;
            auto y0 = win_height - p.y - 1.0;
            auto v = new_mesh->add_vertex(vec3( x0, y0, 0));
            vts.push_back(v);
        }
        //add faces
        builder.add_face(vts);
        builder.end_surface();
        polygon_.clear();
        real_time_polygon_.clear();
        add_model(new_mesh, true);
        fit_screen();

    }
    return Viewer::mouse_press_event(x, y, button, modifiers);

}

bool SymmetryViewer::mouse_free_move_event(int x, int y, int dx, int dy, int modifiers)
{
    if (pickable_)
    {
        real_time_polygon_ = polygon_;
        real_time_polygon_.push_back(vec2(x, y));
        return false;
    } else
        return Viewer::mouse_free_move_event(x, y, dx, dy, modifiers);
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
                output.vector() = mesh_iter->points();
                mesh->points() = output.vector();
                mesh->renderer()->update();
                if (Paired_Edges.size())
                {
                    auto pts = mesh->points();
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
            std::cout << i << " optimal theta: " << optimal_theta * 57.3 << "  min energy: " << obj_min
                      << std::endl;
        }
        std::cout << "completed" << std::endl;
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

            points.push_back(vec3(t1, 0.5));
            points.push_back(vec3(t, 0.5));

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
    } else if (key == GLFW_KEY_LEFT_CONTROL)
    {
        pickable_ = true;
        std::cout << "you can start to left-pick on the paint canvas to select the points,\n"
                     "right click the mouse when it is finished" << std::endl;
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
bool SymmetryViewer::key_release_event(int key, int modifiers)
{
    if (key==GLFW_KEY_LEFT_CONTROL)
    {
        pickable_ = false;
    }
    else
        return Viewer::key_release_event(key, modifiers);
}


bool SymmetryViewer::mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers) {
    if (button == GLFW_MOUSE_BUTTON_LEFT)
        return false;
    else
        return Viewer::mouse_drag_event(x, y, dx, dy, button, modifiers);
}
