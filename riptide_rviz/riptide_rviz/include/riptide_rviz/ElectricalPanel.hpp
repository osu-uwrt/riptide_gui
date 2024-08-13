#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rviz_common/panel.hpp>

#include <riptide_msgs2/msg/electrical_command.hpp>
#include <riptide_msgs2/msg/imu_config.hpp>
#include <riptide_msgs2/action/mag_cal.hpp>
#include <riptide_msgs2/action/tare_gyro.hpp>

#include "ui_ElectricalPanel.h"

namespace riptide_rviz
{
    const static std::string 
        MAG_CAL_ACTION_NAME = "/vectornav/mag_cal",
        TARE_GYRO_ACTION_NAME = "/gyro/tare";

    class ElectricalPanel : public rviz_common::Panel
    {
        using MagCal = riptide_msgs2::action::MagCal;
        using MagSendGoalOptions = rclcpp_action::Client<MagCal>::SendGoalOptions;
        using MagGoalHandle = rclcpp_action::Client<MagCal>::GoalHandle;

        using TareGyro = riptide_msgs2::action::TareGyro;
        using TareGyroSendGoalOptions = rclcpp_action::Client<TareGyro>::SendGoalOptions;
        using TareGyroGoalHandle = rclcpp_action::Client<TareGyro>::GoalHandle;

        Q_OBJECT
        public:
        ElectricalPanel(QWidget *parent = 0);
        ~ElectricalPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        private Q_SLOTS:
        void sendElectricalCommand();
        void sendMagCal();
        void sendTareGyro();

        private:
        void setStatus(const QString& status, bool error);
        void magCalGoalResponseCb(const MagGoalHandle::SharedPtr & goal_handle);
        void magCalFeedbackCb(
            MagGoalHandle::SharedPtr,
            const std::shared_ptr<const MagCal::Feedback> feedback);
        void magCalResultCb(const MagGoalHandle::WrappedResult & result);
        void tareGyroGoalResponseCb(const TareGyroGoalHandle::SharedPtr & goal_handle);
        void tareGyroResultCb(const TareGyroGoalHandle::WrappedResult & result);
        Qt::CheckState processCheckState(bool state);
        void imuConfigCb(const riptide_msgs2::msg::ImuConfig config);
        void publishImuConfig();
        void requestCurrentImuConfig();

        // electrical command vars
        bool loaded = false;
        Ui_ElectricalPanel *ui;
        QString robotNs;

        // mag cal vars
        bool 
            imuCalInProgress = false,
            gyroTareInProgress = false;
            
        double maxVar = 0.0;

        // Continuous mag cal vars
        bool imuHsiEnable = false;
        bool imuHsiOutput = false;
        int imuConvergenceRate = 1; 

        rclcpp::Publisher<riptide_msgs2::msg::ElectricalCommand>::SharedPtr pub;
        rclcpp_action::Client<MagCal>::SharedPtr imuCalClient;
        rclcpp_action::Client<TareGyro>::SharedPtr tareGyroClient;
        
        rclcpp::Publisher<riptide_msgs2::msg::ImuConfig>::SharedPtr writeImuConfig;
        rclcpp::Subscription<riptide_msgs2::msg::ImuConfig>::SharedPtr readImuConfig;
    };
}