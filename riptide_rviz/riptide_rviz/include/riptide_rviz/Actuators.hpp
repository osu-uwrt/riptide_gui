#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <riptide_msgs2/action/arm_torpedo_dropper.hpp>
#include <riptide_msgs2/action/change_claw_state.hpp>
#include <riptide_msgs2/action/actuate_torpedos.hpp>
#include <riptide_msgs2/action/actuate_droppers.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include "ui_Actuators.h"
#include <QTimer>

namespace riptide_rviz
{
    using ArmTorpedoDropper = riptide_msgs2::action::ArmTorpedoDropper;
    using GHArmTorpedoDropper = rclcpp_action::ClientGoalHandle<ArmTorpedoDropper>;
    using ChangeClawState = riptide_msgs2::action::ChangeClawState;
    using GHChangeClawState = rclcpp_action::ClientGoalHandle<ChangeClawState>;
    using ActuateTorpedos = riptide_msgs2::action::ActuateTorpedos;
    using GHActuateTorpedos = rclcpp_action::ClientGoalHandle<ActuateTorpedos>;
    using ActuateDroppers = riptide_msgs2::action::ActuateDroppers;
    using GHActuateDropper = rclcpp_action::ClientGoalHandle<ActuateDroppers>;

    class Actuators : public rviz_common::Panel
    {
        Q_OBJECT public : Actuators(QWidget *parent = 0);
        ~Actuators();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

    protected Q_SLOTS:
        // QT slots (function callbacks)
        

    protected:
        bool event(QEvent *event);
        
        void armTaskStartCb(const GHArmTorpedoDropper::SharedPtr &goalHandle);
        void armTaskCompleteCb(const GHArmTorpedoDropper::WrappedResult &result);
        void armTaskFeedbackCb(GHArmTorpedoDropper::SharedPtr goalHandle, 
                               ArmTorpedoDropper::Feedback::ConstSharedPtr feedback);
        void clawTaskStartCb(const GHChangeClawState::SharedPtr &goalHandle);
        void clawTaskCompleteCb(const GHChangeClawState::WrappedResult &result);
        
        void dropperTaskStartCb(const GHActuateDropper::SharedPtr &goalHandle);
        void dropperTaskCompleteCb(const GHActuateDropper::WrappedResult &result);
        
        void torpedoTaskStartCb(const GHActuateTorpedos::SharedPtr &goalHandle);
        void torpedoTaskCompleteCb(const GHActuateTorpedos::WrappedResult &result);
        
        void handleArming(bool arm_torpedos, bool arm_droppers);
        void handleDroppers(int dropper_id);
        void handleTorpedos(int torpedo_id);
        void handleClaw(bool claw_open);

    private:
        // UI Panel instance
        Ui_Actuators *uiPanel;

        //ros node and timer
        rclcpp::Node::SharedPtr clientNode;
        QTimer * spinTimer;

        //action clients
        rclcpp_action::Client<ArmTorpedoDropper>::SharedPtr armTorpedoDropper;
        rclcpp_action::Client<ChangeClawState>::SharedPtr changeClawState;
        rclcpp_action::Client<ActuateTorpedos>::SharedPtr actuateTorpedos;
        rclcpp_action::Client<ActuateDroppers>::SharedPtr actuateDroppers;

        //process vars
        bool armed_flag;
        std::string robot_ns;

    };

} // namespace riptide_rviz