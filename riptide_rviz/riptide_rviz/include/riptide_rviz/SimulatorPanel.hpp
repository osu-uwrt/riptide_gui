#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <riptide_msgs2/srv/toggle_simulator.hpp>
#include <robot_localization/srv/set_pose.hpp>

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

    enum BtnSyncState {
        BTN_SYNC_IDLE,
        BTN_SYNC_BUSY,
        NUM_SYNC_STATES
    };

    class SimulatorPanel : public rviz_common::Panel {
        Q_OBJECT public : SimulatorPanel(QWidget *parent = 0);
        ~SimulatorPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

    protected Q_SLOTS:
        void handleStartStopClick(void);
        void handleSyncClick(void);

    private:
        void setBtnStartStopState(BtnStartStopState state);
        void setBtnSyncState(BtnSyncState state);
        void toggleUiElements(bool enabled);
        Ui_SimulatorPanel *uiPanel;
        SimulatorState simState;
        rclcpp::Client<riptide_msgs2::srv::ToggleSimulator>::SharedPtr startStopClient;
        rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr syncClient;
        std::string robotNs;
    };
}