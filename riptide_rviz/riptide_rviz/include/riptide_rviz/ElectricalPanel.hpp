#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rviz_common/panel.hpp>
#include <QTimer>

#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <riptide_msgs2/msg/electrical_command.hpp>
#include <riptide_msgs2/msg/imu_config.hpp>
#include <riptide_msgs2/action/mag_cal.hpp>
#include <riptide_msgs2/action/tare_gyro.hpp>
#include <riptide_msgs2/srv/query_imu_serial.hpp>
#include <riptide_msgs2/msg/u_int8_stamped.hpp>

#include "ui_ElectricalPanel.h"

namespace riptide_rviz
{
    const static std::string 
        MAG_CAL_ACTION_NAME = "/vectornav/mag_cal",
        TARE_GYRO_ACTION_NAME = "/gyro/tare",
        CONFIG_SERVICE_NAME = "/vectornav/config";

    const static int NUM_PINGER_FREQUENCIES = 4;
    const static int PINGER_FREQUENCIES[NUM_PINGER_FREQUENCIES] = { 25, 30, 35, 40 };

    class ElectricalPanel : public rviz_common::Panel
    {
        using MagCal = riptide_msgs2::action::MagCal;
        using MagSendGoalOptions = rclcpp_action::Client<MagCal>::SendGoalOptions;
        using MagGoalHandle = rclcpp_action::Client<MagCal>::GoalHandle;

        using TareGyro = riptide_msgs2::action::TareGyro;
        using TareGyroSendGoalOptions = rclcpp_action::Client<TareGyro>::SendGoalOptions;
        using TareGyroGoalHandle = rclcpp_action::Client<TareGyro>::GoalHandle;

        using ImuConfig = riptide_msgs2::srv::QueryImuSerial;
        

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

        void sendIvcMsg();

        void writeIMU();
        void readIMU();
        void saveImuSettings();

        void setPingerFreq(int freq_khz);
        void pingerEnabledChanged(QCheckBox* box);
        
        private:
        void setStatus(const QString& status, bool error);
        void appendIvcConsole(const QString& prefix, const uint8_t& msg);
        void magCalGoalResponseCb(const MagGoalHandle::SharedPtr & goal_handle);
        void magCalFeedbackCb(
            MagGoalHandle::SharedPtr,
            const std::shared_ptr<const MagCal::Feedback> feedback);
        void magCalResultCb(const MagGoalHandle::WrappedResult & result);
        void ivcTxCb(const std_msgs::msg::UInt8::SharedPtr msg);
        void ivcRxCb(const std_msgs::msg::UInt8::SharedPtr msg);
        void ivcTxSuccessCb(const riptide_msgs2::msg::UInt8Stamped::SharedPtr msg);
        void tareGyroGoalResponseCb(const TareGyroGoalHandle::SharedPtr & goal_handle);
        void tareGyroResultCb(const TareGyroGoalHandle::WrappedResult & result);

        void pingerSelectedFreqCb(const std_msgs::msg::Int32::SharedPtr msg);
        void pingerAmplitudeCb(const std_msgs::msg::Float32::SharedPtr msg);

        void sendIMUConfigRequest(const std::string& request, bool extResponseTime = false);
        void waitForConfig(bool extResponseTime = false);

        void uncheckAllPingerButtons();

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

        // Pinger vars
        std::array<QPushButton *, NUM_PINGER_FREQUENCIES> pingerButtons;

        rclcpp::Publisher<riptide_msgs2::msg::ElectricalCommand>::SharedPtr elecPub;

        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ivcTxPub;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr 
            ivcTxSub,
            ivcRxSub;

        rclcpp::Subscription<riptide_msgs2::msg::UInt8Stamped>::SharedPtr ivcSuccessSub;

        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pingerSetFreqKHz;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pingerEnable;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pingerFreqKHzFeedback;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pingerFreqAmplitude;

        rclcpp_action::Client<MagCal>::SharedPtr imuCalClient;
        rclcpp_action::Client<TareGyro>::SharedPtr tareGyroClient;
        
        rclcpp::Client<ImuConfig>::SharedPtr imuConfigClient;
        std::shared_future<std::shared_ptr<riptide_msgs2::srv::QueryImuSerial_Response>> imuConfigFuture;
        int timerTick {};
        int imuConfigFutureId {};
    };
}