#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include "ui_SimulatorPanel.h"

namespace riptide_rviz {
    class SimulatorPanel : public rviz_common::Panel {
        Q_OBJECT public : SimulatorPanel(QWidget *parent = 0);
        ~SimulatorPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

    protected:
        bool event(QEvent *event);

    private: 
        Ui_SimulatorPanel *uiPanel;
    };
}