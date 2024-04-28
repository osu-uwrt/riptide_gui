#pragma once

#include <string>
#include <functional>
#include <future>

#include <QTimer>

#include <rclcpp/rclcpp.hpp>

//
// riptide_rviz srv client utility. Handles calls to services without blocking the ui with feedback to the user.
//

typedef std::function<void(const QString&, const QString&)> StatusUpdateFunc;

template<typename SrvType>
class GuiSrvClient {
    public:
    typedef std::shared_ptr<GuiSrvClient> SharedPtr;
    typedef std::function<void(const std::string&, typename rclcpp::Client<SrvType>::SharedResponse)> ResponseFunc;

    GuiSrvClient(rclcpp::Node::SharedPtr node, const std::string& srv_name, 
                 StatusUpdateFunc status_update_callback, ResponseFunc response_callback)
     : node(node),
       updateStatus(status_update_callback),
       respond(response_callback),
       srvReqId(-1)
    {
        client = node->create_client<SrvType>(srv_name);
    }

    void callService(typename SrvType::Request::SharedPtr request)
    {
        if(srvReqId >= 0)
        {
            updateStatus("A service call is already in progress!", "FF0000");
            return;
        }

        srvReqId = 0;

        std::string srvName = client->get_service_name();
        updateStatus(QString("Waiting for service %1").arg(QString::fromStdString(srvName)), "000000");
        if(!client->wait_for_service(std::chrono::seconds(1)))
        {
            updateStatus("Service unavailable!", "FF0000");
            srvReqId = -1; //we allowed to call services again
            return;
        }

        updateStatus(QString("Making call to service %1").arg(QString::fromStdString(srvName)), "000000");
        auto future = client->async_send_request(request);
        srvReqId = future.request_id;
        sharedFuture = future.share();
        //this is how we wait for a service result
        QTimer::singleShot(250,
            [this] () { this->waitForService(); });
        sendTime = node->get_clock()->now();
    }

    private:

    void waitForService()
    {
        if(!sharedFuture.valid())
        {
            updateStatus("Service result has become invalid!", "FF0000");
            srvReqId = -1;
            return;
        }

        auto futureStatus = sharedFuture.wait_for(std::chrono::milliseconds(5));
        if(futureStatus != std::future_status::timeout)
        {
            //success
            std::string srvName = client->get_service_name();

            typename rclcpp::Client<SrvType>::SharedResponse response = sharedFuture.get();
            respond(srvName, response);

            srvReqId = -1;
            return;
        }

        //not ready yet
        rclcpp::Time currentTime = node->get_clock()->now();
        if(currentTime - sendTime > std::chrono::seconds(5))
        {
            updateStatus("Service timed out.", "FF0000");
            client->remove_pending_request(srvReqId);
            srvReqId = -1;
            return;
        }

        //schedule next check
        QTimer::singleShot(250,
            [this] () { this->waitForService(); });
    }

    rclcpp::Node::SharedPtr node;
    typename rclcpp::Client<SrvType>::SharedPtr client;
    StatusUpdateFunc updateStatus;
    ResponseFunc respond;

    int64_t srvReqId;
    rclcpp::Time sendTime;
    std::shared_future<typename SrvType::Response::SharedPtr> sharedFuture;
};
