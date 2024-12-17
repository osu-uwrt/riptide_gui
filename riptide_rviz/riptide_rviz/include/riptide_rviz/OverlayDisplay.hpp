// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Team Spatzenhirn
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once

#ifndef Q_MOC_RUN
    #include <OgreColourValue.h>
    #include <OgreMaterial.h>
    #include <rviz_common/properties/enum_property.hpp>
    #include <rviz_common/properties/int_property.hpp>
    #include <rviz_common/display.hpp>
    #include <std_msgs/msg/color_rgba.h>

    #include "riptide_rviz/overlay_utils.hpp"

    #include <vector>

#endif

namespace riptide_rviz {
    struct PaintedTextConfig {
        int x_, y_, height_, width_;
        std::string text_, font_name_;
        bool invert_shadow_;
        int line_width_, font_size_;
        QColor text_color_;
    };

    struct PaintedCircleConfig {
        int x_, y_, height_, width_;
        int inner_radius_, outer_radius_;
        QColor inner_color_, outer_color_;
    };

    struct PaintedArcConfig {
        float x_, y_;
        float radius_;
        float start_angle_, end_angle_;
        float line_width_;
        QColor line_color_;
    };

    class OverlayDisplay : public rviz_common::Display {
        Q_OBJECT
      public:
        OverlayDisplay();
        virtual ~OverlayDisplay();

        virtual void onInitialize() override;
        virtual void onEnable() override;
        virtual void onDisable() override;
        virtual void update(float wall_dt, float ros_dt) override;
        virtual void reset() override;

        // methods to add text and shapes to the renderer
        int addText(const PaintedTextConfig & config);
        int addCircle(const PaintedCircleConfig & config);
        int addArc(const PaintedArcConfig& config);
        
        // methods to update text and shape overlays
        void updateText(int index, const PaintedTextConfig & config);
        void updateCircle(int index, const PaintedCircleConfig & config);
        void updateArc(int id, const PaintedArcConfig& config);

        void clearElements();

      protected:
        // helper functions for drawing text and other things on the base figure
        void paintText(const PaintedTextConfig & config, QPainter & painter);
        void paintCircle(const PaintedCircleConfig & config, QPainter & painter);
        void paintArc(const PaintedArcConfig & config, QPainter & painter);

        OverlayObject::SharedPtr overlay_;

        int texture_width_;
        int texture_height_;
        int horizontal_dist_;
        int vertical_dist_;

        HorizontalAlignment horizontal_alignment_;
        VerticalAlignment vertical_alignment_;

        QColor bg_color_;

        bool require_update_texture_;
        // properties are raw pointers since they are owned by Qt
        rviz_common::properties::IntProperty *hor_dist_property_;
        rviz_common::properties::IntProperty *ver_dist_property_;
        rviz_common::properties::EnumProperty *hor_alignment_property_;
        rviz_common::properties::EnumProperty *ver_alignment_property_;
        rviz_common::properties::IntProperty *width_property_;
        rviz_common::properties::IntProperty *height_property_;

        // vectors for holding each of the object types
        std::vector<PaintedTextConfig> text_vector_;
        std::vector<PaintedCircleConfig> circle_vector_;
        std::vector<PaintedArcConfig> arc_vector_;

      protected Q_SLOTS:
        void updateHorizontalDistance();
        void updateVerticalDistance();
        void updateHorizontalAlignment();
        void updateVerticalAlignment();
        void updateWidth();
        void updateHeight();
    };
} // namespace riptide_rviz
