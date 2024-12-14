#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <QTimer>
#include <QMessageBox>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <riptide_msgs2/msg/actuator_status.hpp>
#include "ui_Actuators.h"

#include "riptide_rviz/GuiSrvClient.hpp"

using namespace std::chrono_literals;

namespace riptide_rviz
{
    using Trigger = std_srvs::srv::Trigger;
    using SetBool = std_srvs::srv::SetBool;
    using ActuatorStatus = riptide_msgs2::msg::ActuatorStatus;

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
        void handleReload();
        void handleClawGoHome();
        void handleClawSetHome();
        void handleTorpGoHome();
        void handleTorpSetHome();

    private:
        void statusCallback(const riptide_msgs2::msg::ActuatorStatus::SharedPtr msg);
        void updateStatus(const QString& status, const QString& color);
        void callTriggerService(GuiSrvClient<Trigger>::SharedPtr client);
        void callSetBoolService(GuiSrvClient<SetBool>::SharedPtr client, bool value);

        template<typename T>
        void serviceResponseCb(const std::string& srvName, typename rclcpp::Client<T>::SharedResponse response)
        {
            std::string successStr = (response->success ? "Succeeded" : "Failed");

            updateStatus(QString::fromStdString("Call to %1 %2; %3").arg(
                QString::fromStdString(srvName), QString::fromStdString(successStr), QString::fromStdString(response->message)),
                (response->success ? "000000" : "FF0000"));
        }

        // UI Panel instance
        Ui_Actuators *uiPanel;

        //process vars
        std::string robotNs;

        //status subscriber
        rclcpp::Subscription<ActuatorStatus>::SharedPtr statusSub;

        //service clients
        GuiSrvClient<Trigger>::SharedPtr
            dropperClient,
            torpedoClient,
            reloadClient,
            torpMarkerGoHomeClient,
            torpMarkerSetHomeClient,
            clawSetClosedPosClient;

        GuiSrvClient<SetBool>::SharedPtr
            armClient,
            clawClient;
    };

} // namespace riptide_rviz
