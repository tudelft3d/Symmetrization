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

#ifndef EASY3D_SYMMETRIZATION_H
#define EASY3D_SYMMETRIZATION_H

#include <easy3d/viewer/viewer.h>

struct ImGuiContext;
namespace easy3d {
    class SymmetryViewer : public Viewer {
    public:
        SymmetryViewer(
                const std::string &title = "SymmetryViewer",
                int samples = 4,
                int gl_major = 3,
                int gl_minor = 2,
                bool full_screen = false,
                bool resizable = true,
                int depth_bits = 24,
                int stencil_bits = 8
        );

        double theta, optimal_theta;

        easy3d::Model *add_model(easy3d::Model *model, bool create) override;

        std::vector<int> unpaired;

    protected:

        // imgui plugins
        void init() override;

        // draw the widgets
        void pre_draw() override;

        //  the widgets
        void post_draw() override;

        void cleanup() override;

        void post_resize(int w, int h) override;

        bool mouse_press_event(int x, int y, int button, int modifiers) override;

        bool mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers) override;
        bool mouse_release_event(int x, int y, int button, int modifiers) override;

        void draw_menu_file();

        void draw_menu_view();

    protected:
        // Ratio between the framebuffer size and the window size.
        // May be different from the DPI scaling!
        double pixel_ratio();

        double widget_scaling()
        { return dpi_scaling() / pixel_ratio(); }

        // We don't need a per-window font. So this function is static
        void reload_font(int font_size = 16);


    protected:
        bool key_press_event(int key, int modifiers) override;

        std::string usage() const override;

    protected:
        // Single global context by default, but can be overridden by the user
        static ImGuiContext *context_;

        // Global variables for all the windows
        float alpha_;
        bool movable_;
        bool pickable_= false;

        float menu_height_;
        Polygon2 polygon_, real_time_polygon_;

    };
}


#endif // EASY3D_SYMMETRIZATION_H
