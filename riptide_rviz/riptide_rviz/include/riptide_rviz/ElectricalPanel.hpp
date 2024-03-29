#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rviz_common/panel.hpp>

#include <riptide_msgs2/msg/electrical_command.hpp>
#include <riptide_msgs2/msg/imu_config.hpp>
#include <riptide_msgs2/action/mag_cal.hpp>

#include "ui_ElectricalPanel.h"

namespace riptide_rviz
{
    const static std::string CALIB_ACTION_NAME = "/vectornav/mag_cal";

    class ElectricalPanel : public rviz_common::Panel
    {
        using MagCal = riptide_msgs2::action::MagCal;
        using MagSendGoalOptions = rclcpp_action::Client<MagCal>::SendGoalOptions;
        using MagGoalHandle = rclcpp_action::Client<MagCal>::GoalHandle;
        

        Q_OBJECT
        public:
        ElectricalPanel(QWidget *parent = 0);
        ~ElectricalPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        private Q_SLOTS:
        void sendCommand();
        void sendMagCal();
        void handleMagCalMode();
        void handleMagOutputMode();
        void handleConvergenceRate();

        private:
        void goalResponseCb(const MagGoalHandle::SharedPtr & goal_handle);
        void feedbackCb(
            MagGoalHandle::SharedPtr,
            const std::shared_ptr<const MagCal::Feedback> feedback);
        void resultCb(const MagGoalHandle::WrappedResult & result);
        Qt::CheckState processCheckState(bool state);
        void imuConfigCb(const riptide_msgs2::msg::ImuConfig config);
        void publishImuConfig();
        void requestCurrentImuConfig();

        // electrical command vars
        bool loaded = false;
        Ui_ElectricalPanel *ui;
        QString robotNs;

        // mag cal vars
        bool calInProgress = false;
        double maxVar = 0.0;

        // Continuous mag cal vars
        bool imuHsiEnable = false;
        bool imuHsiOutput = false;
        int imuConvergenceRate = 1; 

        rclcpp::Publisher<riptide_msgs2::msg::ElectricalCommand>::SharedPtr pub;
        rclcpp_action::Client<riptide_msgs2::action::MagCal>::SharedPtr imuCalClient;
        
        rclcpp::Publisher<riptide_msgs2::msg::ImuConfig>::SharedPtr writeImuConfig;
        rclcpp::Subscription<riptide_msgs2::msg::ImuConfig>::SharedPtr readImuConfig;
    };
}