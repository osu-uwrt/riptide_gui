#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <riptide_msgs2/srv/toggle_simulator.hpp>

#include "ui_SimulatorPanel.h"

namespace riptide_rviz {
    enum SimulatorState {
        SIM_STOPPED,
        SIM_RUNNING
    };

    
    enum BtnStartStopState {
        BTN_DISPLAY_START,
        BTN_DISPLAY_WAIT,
        BTN_DISPLAY_STOP,
        NUM_BTN_STATES
    };

    class SimulatorPanel : public rviz_common::Panel {
        Q_OBJECT public : SimulatorPanel(QWidget *parent = 0);
        ~SimulatorPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

    protected Q_SLOTS:
        void handleStartStopClick(void);

    private:
        void setBtnStartStopState(BtnStartStopState state);
        void toggleUiElements(bool enabled);
        Ui_SimulatorPanel *uiPanel;
        SimulatorState simState;
        rclcpp::Client<riptide_msgs2::srv::ToggleSimulator>::SharedPtr client;
        std::string robotNs;
    };
}