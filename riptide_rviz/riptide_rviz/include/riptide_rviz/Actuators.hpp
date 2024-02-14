#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <QTimer>
#include "ui_Actuators.h"

namespace riptide_rviz
{
    class Actuators : public rviz_common::Panel
    {
        Q_OBJECT public : Actuators(QWidget *parent = 0);
        ~Actuators();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

    private slots:
        void handleArm();
        void handleDisarm();
        void handleOpenClaw();
        void handleCloseClaw();
        void handleFireTorpedo();
        void handleDropMarker();
        void handleClawGoHome();
        void handleClawSetHome();
        void handleTorpGoHome();
        void handleTorpSetHome();

    private:
        // UI Panel instance
        Ui_Actuators *uiPanel;

        //process vars
        std::string robot_ns;
    };

} // namespace riptide_rviz