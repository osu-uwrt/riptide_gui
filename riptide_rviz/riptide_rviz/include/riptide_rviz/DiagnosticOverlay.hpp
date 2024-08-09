#pragma once

#include "riptide_rviz/OverlayDisplay.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/float_property.hpp>

namespace riptide_rviz
{
    class DiagnosticOverlay : public OverlayDisplay {
        Q_OBJECT
        public:
        DiagnosticOverlay();

        ~DiagnosticOverlay();

        protected:

        virtual void onInitialize() override;
        virtual void onEnable() override;
        virtual void onDisable() override;
        virtual void update(float wall_dt, float ros_dt) override;
        virtual void reset() override;

        void diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg);
        void killCallback(const std_msgs::msg::Bool & msg);
        void zedCallback(const sensor_msgs::msg::Temperature& msg);
        void leakCallback(const std_msgs::msg::Bool& msg);

        void checkTimeout();

        protected Q_SLOTS:
        void updateFont();
        void updateNS();

        private:

        // timer for determining timeouts
        rclcpp::TimerBase::SharedPtr checkTimer;

        // times for stamping
        rclcpp::Time lastDiag, lastKill, lastZed, lastLeak;
        bool diagsTimedOut, killTimedOut, zedTimedOut, leakTimedOut;
        bool startedLeaking = false;


        // subscription for diagnostics
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killSub;
        rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr zedSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leakSub;

        // ids for rendering items so that we can edit them
        int voltageTextId = -1;
        int diagLedConfigId = -1;
        int killLedConfigId = -1;
        int zedLedConfigId = -1;
        int leakLedConfigId = -1;

        // font configuration info
        QStringList fontFamilies;
        std::string fontName;

        // Addtional RVIZ settings
        rviz_common::properties::EnumProperty *fontProperty;
        rviz_common::properties::StringProperty * robotNsProperty;
        rviz_common::properties::FloatProperty * timeoutProperty;

        // configurations for display items
        PaintedCircleConfig diagLedConfig = {
            20, 50, 0, 0, 7, 9,
            QColor(255, 0, 255, 255),
            QColor(0, 0, 0, 255)
        };
        PaintedCircleConfig killLedConfig = {
            60, 50, 0, 0, 7, 9,
            QColor(255, 0, 255, 255),
            QColor(0, 0, 0, 255)
        };
        PaintedCircleConfig zedLedConfig = {
            100, 50, 0, 0, 7, 9,
            QColor(255, 0, 255, 255),
            QColor(0, 0, 0, 255)
        };
        PaintedCircleConfig leakLedConfig = {
            20, 100, 0, 0, 7, 9,
            QColor(255, 0, 255, 255),
            QColor(0, 0, 0, 255)
        };
        PaintedTextConfig voltageConfig = {
            12, 0, 0, 0, "00.00 V",
            fontName, false, 2, 12,
            QColor(255, 0, 0, 255)
        };

    };
} // namespace riptide_rviz
