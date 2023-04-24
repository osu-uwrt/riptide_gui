#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/client.hpp>

#include "ui_BagItem.h"

#include "riptide_rviz/bringup_recipe.hpp"

#include <launch_msgs/srv/start_bag.hpp>
#include <launch_msgs/srv/stop_bag.hpp>

namespace riptide_rviz
{
    class BagItem : public QWidget
    {
    public:
        BagItem(std::string hostName, std::shared_ptr<RecipeBag> bagData, std::shared_ptr<rclcpp::Node> parentNode, int local = -1);
        ~BagItem();

        bool bagAlive(std::vector<int> bids);

        // get the name of the bag
        void getName(std::string &name);
        int getPid();
        bool local();

        void startBagging();
        bool isStarting();
        void stopBagging();
        bool isStopping();

    protected:
        bool event(QEvent *event);
        void baggingConfigure();

        void startupTimer();
        void shutdownTimer();

    private:
        void stateStopped();
        void stateChanging();
        void stateRunning();

        void loadTopics(const std::vector<RecipeTopicData> &topicList);

        // UI Panel instance
        Ui_BagListElement *uiPanel;

        // list of bag information containers
        std::vector<launch_msgs::msg::TopicData> topics;
        std::shared_ptr<RecipeBag> bagData;

        // service clients
        rclcpp::Client<launch_msgs::srv::StartBag>::SharedPtr bagStartClient;
        rclcpp::Client<launch_msgs::srv::StopBag>::SharedPtr bagStopClient;

        // futures
        std::shared_future<std::shared_ptr<launch_msgs::srv::StartBag_Response>> startFuture;
        std::shared_future<std::shared_ptr<launch_msgs::srv::StopBag_Response>> stopFuture;
        int64_t startReqId = -1, stopReqId = -1;

        int timerTick = 0;

        // bag id
        int pid;

        bool isLocal;
    };
} // namespace riptide_rviz