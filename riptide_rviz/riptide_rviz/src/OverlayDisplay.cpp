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

#include "riptide_rviz/OverlayDisplay.hpp"

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTexture.h>
#include <QFontDatabase>
#include <QPainter>
#include <QStaticText>
#include <QTextDocument>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <regex>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/render_system.hpp>
#include <sstream>

namespace riptide_rviz {
    OverlayDisplay::OverlayDisplay() :
        texture_width_(0),
        texture_height_(0),
        bg_color_(255,255,255,0),
        require_update_texture_(false) {
        hor_dist_property_ = new rviz_common::properties::IntProperty("hor_dist", 0, "horizontal distance to anchor",
                                                                      this, SLOT(updateHorizontalDistance()));
        ver_dist_property_ = new rviz_common::properties::IntProperty("ver_dist", 0, "vertical distance to anchor",
                                                                      this, SLOT(updateVerticalDistance()));

        hor_alignment_property_ = new rviz_common::properties::EnumProperty("hor_alignment", "left",
            "horizontal alignment of the overlay", this, SLOT(updateHorizontalAlignment()));
        hor_alignment_property_->addOption("left", (int)HorizontalAlignment::LEFT);
        hor_alignment_property_->addOption("center", (int)HorizontalAlignment::CENTER);
        hor_alignment_property_->addOption("right", (int)HorizontalAlignment::RIGHT);

        ver_alignment_property_ = new rviz_common::properties::EnumProperty("ver_alignment", "top",
            "vertical alignment of the overlay", this, SLOT(updateVerticalAlignment()));
        ver_alignment_property_->addOption("top", (int)VerticalAlignment::TOP);
        ver_alignment_property_->addOption("center", (int)VerticalAlignment::CENTER);
        ver_alignment_property_->addOption("bottom", (int)VerticalAlignment::BOTTOM);

        width_property_ = new rviz_common::properties::IntProperty("width", 400, "width position", this, SLOT(updateWidth()));
        width_property_->setMin(0);
        height_property_ = new rviz_common::properties::IntProperty("height", 200, "height position", this, SLOT(updateHeight()));
        height_property_->setMin(0);
    }

    OverlayDisplay::~OverlayDisplay() {
        onDisable();
    }

    void OverlayDisplay::onEnable() {
        if (overlay_) {
            overlay_->show();
        }
    }

    void OverlayDisplay::onDisable() {
        if (overlay_) {
            overlay_->hide();
        }
    }

    // only the first time
    void OverlayDisplay::onInitialize() {
        rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

        // create the overlay
        if (!overlay_) {
            static int count = 0;
            std::stringstream ss;
            ss << "OverlayTextDisplayObject" << count++;
            overlay_.reset(new riptide_rviz::OverlayObject(ss.str()));
            overlay_->show();
        }

        onEnable();
        updateHorizontalDistance();
        updateVerticalDistance();
        updateHorizontalAlignment();
        updateVerticalAlignment();
        updateWidth();
        updateHeight();
        require_update_texture_ = true;
    }

    void OverlayDisplay::update(float /*wall_dt*/, float /*ros_dt*/) {
        if (!require_update_texture_) {
            return;
        }
        if (!isEnabled()) {
            return;
        }
        if (!overlay_) {
            return;
        }

        // std::cerr << "re-rendering overlay" << std::endl;

        overlay_->updateTextureSize(texture_width_, texture_height_);
        {
            ScopedPixelBuffer buffer = overlay_->getBuffer();
            QImage Hud = buffer.getQImage(*overlay_, bg_color_);
            QPainter painter(&Hud);
            painter.setRenderHint(QPainter::Antialiasing, true);
            uint16_t w = overlay_->getTextureWidth();
            uint16_t h = overlay_->getTextureHeight();

            // paint what we want to have on the display
            // the call that forced the render update needs to have
            // kept all previous info
            for(auto config : text_vector_){
                // std::cerr << "painting text: " << config.text_ << std::endl;
                config.width_ = w;
                config.height_ = h;
                paintText(config, painter);
            }

            // do the same for all circles
            for(auto config : circle_vector_){
                // std::cerr << "Painting circle: " << config.x_ << " " << config.y_ << std::endl;
                config.width_ = w;
                config.height_ = h;
                paintCircle(config, painter);
            }

            for(auto config : arc_vector_) {
                paintArc(config, painter);
            }
            
            painter.end();
        }

        // update render parameters for rviz
        overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
        overlay_->setPosition(horizontal_dist_, vertical_dist_, horizontal_alignment_, vertical_alignment_);

        require_update_texture_ = false;
    }

    /**
     * Helper method for rendering text onto a QImage with a QPainter
     * @param config a painted text config containing the location to paint
     * the font and size to use, the text to put in and the overall width and height of the panel
     * @param painter the painter to use while building the text overlay. Passed by refrence to
     * keep the changes pointed at the correct image
    */
    void OverlayDisplay::paintText(const PaintedTextConfig & config, QPainter & painter){
        // clear the brush just in case
        painter.setBrush(Qt::NoBrush);

        // configure the pen of the painter for the color we want
        painter.setPen(QPen(config.text_color_, std::max(config.line_width_, 1), Qt::SolidLine));

        // font
        if (config.font_size_ != 0) {
            // QFont font = painter.font();
            QFont font(config.font_name_.length() > 0 ? config.font_name_.c_str() : "Liberation Sans");
            font.setPointSize(config.font_size_);
            font.setBold(true);
            painter.setFont(font);
        }
        if (config.text_.length() > 0) {

            QColor shadow_color;
            if (config.invert_shadow_)
                shadow_color = Qt::white; // fg_color_.lighter();
            else
                shadow_color = Qt::black; // fg_color_.darker();
            shadow_color.setAlpha(config.text_color_.alpha());

            std::string color_wrapped_text =
                (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>") % config.text_ %
                config.text_color_.red() % config.text_color_.green() % config.text_color_.blue() % 
                config.text_color_.alpha()).str();

            // find a remove "color: XXX;" regex match to generate a proper shadow
            std::regex color_tag_re("color:.+?;");
            std::string null_char("");
            std::string formatted_text_ = std::regex_replace(config.text_, color_tag_re, null_char);
            std::string color_wrapped_shadow =
                    (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>") % formatted_text_ %
                        shadow_color.red() % shadow_color.green() % shadow_color.blue() % shadow_color.alpha())
                            .str();

            QStaticText static_text(boost::algorithm::replace_all_copy(color_wrapped_text, "\n", "<br >").c_str());
            static_text.setTextWidth(config.width_);

            painter.setPen(QPen(shadow_color, std::max(config.line_width_, 1), Qt::SolidLine));
            QStaticText static_shadow(
                    boost::algorithm::replace_all_copy(color_wrapped_shadow, "\n", "<br >").c_str());
            static_shadow.setTextWidth(config.width_);

            QStaticText only_wrapped_text(color_wrapped_text.c_str());
            QFontMetrics fm(painter.fontMetrics());
            QRect text_rect = fm.boundingRect(0, 0, config.width_, config.height_, Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                                                only_wrapped_text.text().remove(QRegExp("<[^>]*>")));
            painter.drawStaticText(config.x_ + 1, config.y_ + 1, static_shadow);
            painter.drawStaticText(config.x_, config.y_, static_text);
        }
    }

    void OverlayDisplay::paintCircle(const PaintedCircleConfig & config, QPainter & painter){
        // clear the pen and switch to a brush
        painter.setPen(Qt::NoPen);

        // configure the brush of the painter and draw outer circle
        Qt::BrushStyle style = Qt::SolidPattern;
        QBrush brush(config.outer_color_, style);
        painter.setBrush(brush);
        painter.drawEllipse(QPoint(config.x_, config.y_), config.outer_radius_, config.outer_radius_);

        // draw the inner circle now
        brush.setColor(config.inner_color_);
        painter.setBrush(brush);
        painter.drawEllipse(QPoint(config.x_, config.y_), config.inner_radius_, config.inner_radius_);
    }

    void OverlayDisplay::paintArc(const PaintedArcConfig & config, QPainter & painter) {
        painter.setPen(QPen(config.line_color_, config.line_width_));
        QRectF rect(config.x_ - config.radius_,
                    config.y_ - config.radius_,
                    config.radius_ * 2,
                    config.radius_ * 2);
        painter.drawArc(rect, 
                        config.start_angle_ * 16,  // Qt uses 1/16th of a degree
                        (config.end_angle_ - config.start_angle_) * 16);
    }

    // methods to add text and shapes to the renderer
    int OverlayDisplay::addText(const PaintedTextConfig & config){
        text_vector_.push_back(config);
        require_update_texture_ = true;
        return text_vector_.size() - 1;
    }

    int OverlayDisplay::addCircle(const PaintedCircleConfig & config){
        circle_vector_.push_back(config);
        require_update_texture_ = true;
        return circle_vector_.size() - 1;
    }

    int OverlayDisplay::addArc(const PaintedArcConfig& config) {
        arc_vector_.push_back(config);
        require_update_texture_ = true;
        return arc_vector_.size() - 1;
    }

    void OverlayDisplay::updateArc(int index, const PaintedArcConfig& config) {
        arc_vector_.at(index) = config;
        require_update_texture_ = true;
    }

    // methods to update text and shape overlays
    void OverlayDisplay::updateText(int index, const PaintedTextConfig & config){
        text_vector_.at(index) = config;
        require_update_texture_ = true;
    }

    void OverlayDisplay::updateCircle(int index, const PaintedCircleConfig & config){
        circle_vector_.at(index) = config;
        require_update_texture_ = true;
    }

    void OverlayDisplay::clearElements(){
        text_vector_.clear();
        circle_vector_.clear();
        arc_vector_.clear();
        require_update_texture_ = true;
    }

    void OverlayDisplay::reset() {
        if (overlay_) {
            overlay_->hide();
        }
    }

    void OverlayDisplay::updateVerticalDistance() {
        vertical_dist_ = ver_dist_property_->getInt();
        require_update_texture_ = true;
    }

    void OverlayDisplay::updateHorizontalDistance() {
        horizontal_dist_ = hor_dist_property_->getInt();
        require_update_texture_ = true;
    }

    void OverlayDisplay::updateVerticalAlignment() {
        vertical_alignment_ = static_cast<VerticalAlignment>(ver_alignment_property_->getOptionInt());
        require_update_texture_ = true;
    }

    void OverlayDisplay::updateHorizontalAlignment() {
        horizontal_alignment_ = static_cast<HorizontalAlignment>(hor_alignment_property_->getOptionInt());
        require_update_texture_ = true;
    }

    void OverlayDisplay::updateWidth() {
        texture_width_ = width_property_->getInt();
        require_update_texture_ = true;
    }

    void OverlayDisplay::updateHeight() {
        texture_height_ = height_property_->getInt();
        require_update_texture_ = true;
    }

} // namespace rviz_2d_overlay_plugins

// Since this is a parent class this has been removed
// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(riptide_rviz::OverlayTextDisplay, rviz_common::Display)