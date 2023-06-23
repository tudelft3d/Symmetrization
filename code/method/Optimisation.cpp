//
// Created by jinhuang on 12-10-21.
//
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/resources.h>
#include <easy3d/util/logging.h>
#include <easy3d/fileio/point_cloud_io.h>
#include "Optimisation.h"
#include <easy3d/core/surface_mesh_builder.h>
#include <fstream>
#include <chrono>
#include <gurobi_c++.h> //use gurobi as the solver

namespace easy3d {
    namespace details {
        namespace vertices {

            struct EdgeVector {
                SurfaceMesh::Vertex s; // the source of an edge
                SurfaceMesh::Vertex t; // the target of an edge
                SurfaceMesh::Edge e;
                bool y_side = true;
            };

            bool operator<(EdgeVector const &lhs, EdgeVector const &rhs)
            {
                if (lhs.s.idx() < rhs.s.idx())
                    return true;
            }

            void sort(std::vector<double> &a, std::vector<double> &b)
            {
                int length = a.size();
                for (int j = 0; j < length; j++)
                    for (int i = j + 1; i < length; i++)
                        if (a[i] < a[j])
                        {
                            double t1, t;
                            t = a[i];
                            a[i] = a[j];
                            a[j] = t;
                            t1 = b[i];
                            b[i] = b[j];
                            b[j] = t1;
                        }
            }

            struct VerticesData {
                int check_side(vec2 va)
                {
                    double x = va.x;
                    if (x > 0)
                        return 1;
                    else
                        return -1;
                }

                bool check_xside(SurfaceMesh *mesh, SurfaceMesh::Edge e)
                {
                    SurfaceMesh::Vertex s=mesh->vertex(e, 0);
                    SurfaceMesh::Vertex t=mesh->vertex(e, 1);
                    auto vs = mesh->position(s), vt = mesh->position(t);
                    auto st = vs - vt;
                    if (std::abs(st.y) < std::abs(st.x))
                    {
                        return false;
                    }
                    else
                        return  true;
                }

                vec2 vec3to2d(vec3 a)
                {
                    return vec2(a.x, a.y);
                }

                bool y_dist(SurfaceMesh *mesh, SurfaceMesh::Edge e0, SurfaceMesh::Edge e1)
                {
                    auto pts = mesh->points();
                    auto v0s = vec3to2d(pts[mesh->vertex(e0, 0).idx()]),
                            v0t = vec3to2d(pts[mesh->vertex(e0, 1).idx()]),
                            v1s = vec3to2d(pts[mesh->vertex(e1, 0).idx()]),
                            v1t = vec3to2d(pts[mesh->vertex(e1, 1).idx()]);
                    auto side_v0s = check_side(v0s),
                            side_v0t = check_side(v0t),
                            side_v1s = check_side(v1s),
                            side_v1t = check_side(v1t);
                    if ((side_v0s == side_v0t) && (side_v1s == side_v1t))
                    {
                        if (side_v0s * side_v1s > 0) //ei and ej are in the same side
                        {
                            return false;
                        }
                    }
                    auto v0sy = pts[mesh->vertex(e0, 0).idx()].y,
                            v0ty = pts[mesh->vertex(e0, 1).idx()].y,
                            v1sy = pts[mesh->vertex(e1, 0).idx()].y,
                            v1ty = pts[mesh->vertex(e1, 1).idx()].y;
                    auto min1 = std::min(v0ty, v0sy), max1 = std::max(v0ty, v0sy),
                            min2 = std::min(v1ty, v1sy), max2 = std::max(v1ty, v1sy);
                    auto off = std::min(max1, max2) - std::max(min1, min2);
                    auto le0=(pts[mesh->vertex(e0, 0).idx()] -pts[mesh->vertex(e0, 1).idx()]).length(),
                    le1=(pts[mesh->vertex(e1, 0).idx()] -pts[mesh->vertex(e1, 1).idx()]).length();
                    auto threshold = 0.5* std::max(le0,le1);
                    if (off >= -threshold)
                    {
                        return true;
                    } else
                    {
                        return false;
                    }

                }

                VerticesData(SurfaceMesh *mesh) : lambda(300)
                {
                    std::size_t num = mesh->n_vertices();

                    auto pts = mesh->points();
                    std::vector<double> angels;
                    for (auto v: mesh->vertices())
                    {
                        double edge_min = 0;
                        std::vector<vec2> edge_vector;
                        for (auto vv: mesh->vertices(v))
                            edge_vector.push_back(pts[v.idx()] - pts[vv.idx()]);
                        edge_min = (dot(edge_vector[0], edge_vector[1]) /
                                    (edge_vector[0].length() * edge_vector[1].length()));
                        auto theta = std::acos(edge_min);
                        angels.push_back(theta);
                        double final_coef = std::exp(pow(M_PI - theta, 1));
                        coefficient.push_back(final_coef);
                    }
                    double threshold= 10;
                    for (auto e:mesh->edges())
                    {
                        auto v0=mesh->vertex(e, 0), v1=mesh->vertex(e, 1);
                        auto dir = pts[v0.idx()] - pts[v1.idx()];
                        auto theta1= std::acos(std::abs(dot(dir, vec3(1, 0,0))) / dir.length())*180/M_PI;
                        auto theta2= std::acos(std::abs(dot(dir, vec3(0, 1,0))) / dir.length())*180/M_PI;
                        if (theta1<threshold)
                        {
                            dir_cons[e]=vec2(1,0);
                        }
                        if (theta2<threshold)
                        {
                            dir_cons[e]=vec2(0,1);
                        }
                    }

                    x0.resize(num * 2);
                    for (auto v: mesh->vertices())
                    {
                        const int id = v.idx();
                        const vec3 &p = pts[id];
                        x0[id * 2] = p.x;
                        x0[id * 2 + 1] = p.y;
                    }
                    auto max = *std::max_element(std::begin(x0), std::end(x0));
                    auto min = *std::min_element(std::begin(x0), std::end(x0));
                    //set the lower bound and upper bound
                    lb = min - 1.2 * (max - min);
                    ub = max + 1.2 * (max - min);
                    // initialize combination constraints
                    int idx = x0.size();

                    auto num_half = mesh->n_edges();
                    for (int ind = 0; ind < num_half; ind++)
                    {
                        auto e = SurfaceMesh::Edge(ind);
                        auto iter = SurfaceMesh::Edge((ind + 1));
                        for (; iter.idx() < num_half;)
                        {
                            EdgeVector e1, e2;
                            e1.s = mesh->vertex(e, 0);
                            e1.t = mesh->vertex(e, 1);
                            e1.e = e;
                            e1.y_side = check_xside(mesh, e);

                            e2.s = mesh->vertex(iter, 0);
                            e2.t = mesh->vertex(iter, 1);
                            e2.e = iter;
                            e2.y_side = check_xside(mesh, iter);
                            auto overlap = y_dist(mesh, e, iter);
                            if (overlap)
                            {
                                std::pair<EdgeVector, EdgeVector> ep(e1, e2);
                                edge_pairs_consts.insert(ep);
                                std::pair<int, int> b(e.idx(), iter.idx());
                                bin_cons_idx[b] = idx;
                                idx++;
                            }
                            iter = SurfaceMesh::Edge(iter.idx() + 1);
                        }
                    }
                }

                std::vector<double> x0;//store the initial position of vertices
                std::map<SurfaceMesh::Edge,vec2> dir_cons; //store the orthogonal or horizontal cons
                std::set<std::pair<EdgeVector, EdgeVector>> edge_pairs_consts;//store the edge pairs
                std::map<std::pair<int, int>, int> bin_cons_idx;
                std::vector<double> coefficient;
                double lambda;
                double lb, ub;
            };

        }
    }


    SurfaceMeshSymmetry::SurfaceMeshSymmetry(SurfaceMesh *mesh, bool extra_constraints)
            : mesh_(mesh)
    {
        vertical_horizontal_constraints=extra_constraints;
    }
    SurfaceMeshSymmetry::~SurfaceMeshSymmetry(void)
    {
    }
    double SurfaceMeshSymmetry::optimize_vertices()
    {
        details::vertices::VerticesData data(mesh_);
        const auto &edge_pairs_constrs = data.edge_pairs_consts;
        const auto &bin_cons_idx = data.bin_cons_idx;
        const auto &lambda = data.lambda;
        const auto &coefficient = data.coefficient;
        const auto &dir_cons = data.dir_cons;

        int n = mesh_->n_vertices() * 2;
        //the initial position of points
        std::vector<double> p(n);
        for (int i = 0; i < n; ++i)
        {
            p[i] = data.x0[i];
        }
        // assign new vertices
        try
        {
            auto t_start = std::chrono::high_resolution_clock::now();
            static GRBEnv env = GRBEnv();
            env.set(GRB_IntParam_LogToConsole, 0);

            GRBModel model = GRBModel(env);

            // create variables, the num of variables equals to 2*num_edge+num_edge_pair
            std::vector<GRBVar> X(p.size() + edge_pairs_constrs.size());

            //add continuous variable
            for (std::size_t i = 0; i < p.size(); ++i)
            {
                X[i] = model.addVar(data.lb, data.ub, 0.0, GRB_CONTINUOUS);
            }
            //add binary variable
            for (std::size_t i = 0; i < edge_pairs_constrs.size(); ++i)
            {
                X[i + p.size()] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
            }
            // Integrate new variables
            model.update();
            model.set(GRB_IntParam_NonConvex, 2);


            //Add orthogonal  constraints, use slope
            if (vertical_horizontal_constraints)
            {             auto it = dir_cons.begin();
                for (; it != dir_cons.end(); it++)
                {
                    auto e=it->first;auto dir=it->second;
                    auto v0 = mesh_->vertex(e, 0), v1 = mesh_->vertex(e, 1);
                    auto vsx1 = X[v0.idx() * 2], vsy1 = X[v0.idx() * 2 + 1];
                    auto vtx1 = X[v1.idx() * 2], vty1 = X[v1.idx() * 2 + 1];
                    if (dir[0]==1)
                    {
                        model.addConstr(vsy1 - vty1 == 0);
                    }
                    else{
                        model.addConstr(vsx1 - vtx1 == 0);
                    }

                }
            }

            //Add nonlinear constraints
            auto iter = edge_pairs_constrs.begin();
            for (; iter != edge_pairs_constrs.end(); iter++)
            {

                auto e1 = iter->first, e2 = iter->second;
                unsigned int vs1 = e1.s.idx();
                unsigned int vt1 = e1.t.idx();
                unsigned int vs2 = e2.s.idx();
                unsigned int vt2 = e2.t.idx();
                auto key = *iter;
                auto m = std::make_pair<int, int>(key.first.e.idx(), key.second.e.idx());
                auto num_idx = bin_cons_idx.at(m);
                auto vsx1 = X[vs1 * 2], vsy1 = X[vs1 * 2 + 1];
                auto vtx1 = X[vt1 * 2], vty1 = X[vt1 * 2 + 1];
                auto vsx2 = X[vs2 * 2], vsy2 = X[vs2 * 2 + 1];
                auto vtx2 = X[vt2 * 2], vty2 = X[vt2 * 2 + 1];

                auto psx1 = p[vs1 * 2], psy1 = p[vs1 * 2 + 1];
                auto ptx1 = p[vt1 * 2], pty1 = p[vt1 * 2 + 1];
                auto psx2 = p[vs2 * 2], psy2 = p[vs2 * 2 + 1];
                auto ptx2 = p[vt2 * 2], pty2 = p[vt2 * 2 + 1];

                auto X_ij = X[num_idx];
#define x_side
#ifdef x_side
                if (e1.y_side == false and e2.y_side == false)
                {
                    //add another term
                    if (psx1 < ptx1 && psx2 < ptx2)
                    {
                        model.addQConstr(X_ij * (vty1 - vsy2) == 0);
                        model.addQConstr(X_ij * (vsy1 - vty2) == 0);
                        model.addQConstr(X_ij * (vtx1 + vsx2) == 0);
                        model.addQConstr(X_ij * (vsx1 + vtx2) == 0);
                    } else if (psx1 < ptx1 && psx2 >= ptx2)
                    {
                        model.addQConstr(X_ij * (vty1 - vty2) == 0);
                        model.addQConstr(X_ij * (vsy1 - vsy2) == 0);
                        model.addQConstr(X_ij * (vtx1 + vtx2) == 0);
                        model.addQConstr(X_ij * (vsx1 + vsx2) == 0);
                    } else if (psx1 >= ptx1 && psx2 < ptx2)
                    {
                        model.addQConstr(X_ij * (vty1 - vty2) == 0);
                        model.addQConstr(X_ij * (vsy1 - vsy2) == 0);
                        model.addQConstr(X_ij * (vsx1 + vsx2) == 0);
                        model.addQConstr(X_ij * (vtx1 + vtx2) == 0);
                    } else
                    {
                        model.addQConstr(X_ij * (vsy1 - vty2) == 0);
                        model.addQConstr(X_ij * (vty1 - vsy2) == 0);
                        model.addQConstr(X_ij * (vsx1 + vtx2) == 0);
                        model.addQConstr(X_ij * (vtx1 + vsx2) == 0);
                    }
                } else
#endif
                {
                    //add another term
                    if (psy1 < pty1 && psy2 < pty2)
                    {
                        model.addQConstr(X_ij * (vty1 - vty2) == 0);
                        model.addQConstr(X_ij * (vsy1 - vsy2) == 0);
                        model.addQConstr(X_ij * (vtx1 + vtx2) == 0);
                        model.addQConstr(X_ij * (vsx1 + vsx2) == 0);
                    } else if (psy1 < pty1 && psy2 >= pty2)
                    {
                        model.addQConstr(X_ij * (vty1 - vsy2) == 0);
                        model.addQConstr(X_ij * (vsy1 - vty2) == 0);
                        model.addQConstr(X_ij * (vtx1 + vsx2) == 0);
                        model.addQConstr(X_ij * (vsx1 + vtx2) == 0);
                    } else if (psy1 >= pty1 && psy2 < pty2)
                    {
                        model.addQConstr(X_ij * (vty1 - vsy2) == 0);
                        model.addQConstr(X_ij * (vsy1 - vty2) == 0);
                        model.addQConstr(X_ij * (vsx1 + vtx2) == 0);
                        model.addQConstr(X_ij * (vtx1 + vsx2) == 0);
                    } else
                    {
                        model.addQConstr(X_ij * (vty1 - vty2) == 0);
                        model.addQConstr(X_ij * (vsy1 - vsy2) == 0);
                        model.addQConstr(X_ij * (vsx1 + vsx2) == 0);
                        model.addQConstr(X_ij * (vtx1 + vtx2) == 0);
                    }
                }

            }

            for (int k = 0; k < mesh_->n_edges(); ++k)
            {
                GRBLinExpr expr;
                for (auto it = edge_pairs_constrs.begin(); it != edge_pairs_constrs.end(); ++it)
                {
                    auto ep = *it;
                    auto ei = ep.first.e.idx(), ej = ep.second.e.idx();
                    if (ei == k || ej == k)//accumulate the term for the same i
                    {
                        auto m = std::make_pair<int, int>(ep.first.e.idx(), ep.second.e.idx());
                        auto num_idx = bin_cons_idx.at(m);
                        expr += X[num_idx];
                    }
                }
                model.addConstr(expr <= 1);
            }

            // Set objective
            GRBQuadExpr obj;

            for (int j = 0; j < p.size(); ++j)
            {
                double coe = coefficient[j / 2];
                obj += coe * (X[j] - p[j]) * (X[j] - p[j]);
            }

            for (int i = p.size(); i < p.size() + edge_pairs_constrs.size(); ++i)
            {
                obj += lambda * (1-X[i]);
            }

            // Set objective function sense
            model.setObjective(obj, GRB_MINIMIZE);

            // Optimize model result_transfer
            model.optimize();
            int status = model.get(GRB_IntAttr_Status);
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            std::cout << "optimization time:  " << elapsed_time_ms / 1000 << "s" << std::endl;

            switch (status)
            {
                case GRB_OPTIMAL:
                {
                    result_.resize(p.size() + edge_pairs_constrs.size());
                    auto points = mesh_->get_vertex_property<vec3>("v:point");
                    for (auto v: mesh_->vertices())
                    {
                        const int id = v.idx();
                        vec2 p_new(X[2 * id].get(GRB_DoubleAttr_X), X[2 * id + 1].get(GRB_DoubleAttr_X));
                        vec2 p_old(p[2 * id], p[2 * id + 1]);
                        points[v] = vec3(p_new, 0);
                    }
                    std::vector<bool> used_edge(mesh_->n_edges());
                    std::vector<std::pair<int, int>> paired_edges;
                    for (std::size_t i = p.size(); i < p.size() + edge_pairs_constrs.size(); ++i)
                    {
                        result_[i] = X[i].get(GRB_DoubleAttr_X);
                        if (result_[i])
                        {
                            int value = i;
                            for (auto m: bin_cons_idx)
                            {
                                if (m.second == value)
                                {
                                    used_edge[m.first.first] = true;
                                    used_edge[m.first.second] = true;
                                    paired_edges.push_back(std::make_pair(m.first.first, m.first.second));
                                }
                            }
                        }
                    }
                    mesh_->add_face_property<std::vector<std::pair<int, int>>>("e:edge_paired", paired_edges);
                    auto unused = mesh_->add_edge_property<bool>("e:unpaired");
                    for (auto e: mesh_->edges())
                        unused[e] = false;
                    return model.get(GRB_DoubleAttr_ObjVal);
                }

                case GRB_INF_OR_UNBD:
                    std::cerr << "model is infeasible or unbounded" << std::endl;
                    break;

                case GRB_INFEASIBLE:
                    std::cerr << "model is infeasible" << std::endl;
                    break;

                case GRB_UNBOUNDED:
                    std::cerr << "model is unbounded" << std::endl;
                    break;

                default:
                    std::cerr << "optimization was stopped with status = " << status << std::endl;
                    break;
            }

            return (status == GRB_OPTIMAL);
        }
        catch (GRBException e)
        {
            std::cout << e.getMessage() << " (error code: " << e.getErrorCode() << ")." << std::endl;
            if (e.getErrorCode() == GRB_ERROR_NO_LICENSE)
            {
                std::cout
                        << "Gurobi installed but license is missing or expired. Please choose another solver, e.g., SCIP."
                        << std::endl;
            }
        }
        catch (...)
        {
            std::cerr << "Exception during optimization" << std::endl;
        }
        return false;
    }
}




