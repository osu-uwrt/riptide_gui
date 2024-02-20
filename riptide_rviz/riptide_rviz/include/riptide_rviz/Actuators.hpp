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
        void updateStatus(const std::string& status);
        void callTriggerService(rclcpp::Client<Trigger>::SharedPtr client);
        void callSetBoolService(rclcpp::Client<SetBool>::SharedPtr client, bool value);

        template<typename SrvType>
        void callService(typename rclcpp::Client<SrvType>::SharedPtr client, typename SrvType::Request::SharedPtr request, typename std::shared_future<typename SrvType::Response::SharedPtr>& shared_future)
        {
            if(srvReqId >= 0)
            {
                QMessageBox::warning(uiPanel->mainWidget, "Service call in progress!", "A service call is already in progess. Please try again later.");
                return;
            }

            srvReqId = 0;

            std::string srvName = client->get_service_name();
            updateStatus("Waiting for service " + srvName);
            if(!client->wait_for_service(1s))
            {
                updateStatus("Service unavailable!");
                srvReqId = -1; //we allowed to call services again
                return;
            }

            updateStatus("Making call to service " + srvName);
            auto future = client->async_send_request(request);
            srvReqId = future.request_id;
            shared_future = future.share();
            //this is how we wait for a service result
            QTimer::singleShot(250,
                [this, client, &shared_future] () { this->waitForService<SrvType>(client, shared_future); });
            auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
            clientSendTime = node->get_clock()->now();
        }


        template<typename SrvType>
        void waitForService(typename rclcpp::Client<SrvType>::SharedPtr client, typename std::shared_future<typename SrvType::Response::SharedPtr>& future)
        {
            if(!future.valid())
            {
                updateStatus("Service result has become invalid!");
                srvReqId = -1;
                return;
            }

            auto futureStatus = future.wait_for(10ms);
            if(futureStatus != std::future_status::timeout)
            {
                //success
                typename rclcpp::Client<SrvType>::SharedResponse response = future.get();
                std::string 
                    srvName = client->get_service_name(),
                    successStr = (response->success ? "Succeeded" : "Failed");

                updateStatus("Call to " + srvName + " " + successStr + "; " + response->message);
                srvReqId = -1;
                return;
            }

            //not ready yet
            auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
            rclcpp::Time currentTime = node->get_clock()->now();
            if(currentTime - clientSendTime > 5s)
            {
                updateStatus("Service timed out.");
                client->remove_pending_request(srvReqId);
                srvReqId = -1;
                return;
            }

            //schedule next check
            QTimer::singleShot(250,
                [this, client, &future] () { this->waitForService<SrvType>(client, future); });
        }

        // UI Panel instance
        Ui_Actuators *uiPanel;

        //process vars
        std::string robotNs;

        //status subscriber
        rclcpp::Subscription<ActuatorStatus>::SharedPtr statusSub;

        //service clients
        rclcpp::Client<Trigger>::SharedPtr
            dropperClient,
            torpedoClient,
            reloadClient,
            torpMarkerGoHomeClient,
            torpMarkerSetHomeClient;

        rclcpp::Client<SetBool>::SharedPtr
            armClient,
            clawClient;
        
        //service tracking
        std::shared_future<Trigger::Response::SharedPtr> activeTriggerClientFuture;
        std::shared_future<SetBool::Response::SharedPtr> activeSetBoolClientFuture;
        int64_t srvReqId;
        rclcpp::Time clientSendTime;
    };

} // namespace riptide_rviz
