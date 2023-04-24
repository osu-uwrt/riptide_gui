#pragma once

#include "riptide_rviz/BagItem.hpp"
#include "riptide_rviz/bringup_recipe.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <QTimer>
#include <vector>

#include "ui_BaggingPanel.h"

#include <launch_msgs/msg/list_pids.hpp>
#include <launch_msgs/srv/who_is.hpp>
#include <std_msgs/msg/bool.hpp>

namespace riptide_rviz
{
    class BaggingPanel : public rviz_common::Panel
    {
        Q_OBJECT public : BaggingPanel(QWidget *parent = 0);
        ~BaggingPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;
        void clearScrollArea();

    protected Q_SLOTS:
        // QT slots (function callbacks)
        void baggingConfigure();
        void hostListRefresh();
        void fileListRefresh();
        void handleBaggingPanelHost(const QString &text);
        void handleBaggingPanelFile(const QString &text);
        void startBagging();

    protected:
        bool event(QEvent *event);

        void waitForWhois();

    private:
        // UI Panel instance
        Ui_BaggingPanel *uiPanel;

        // parent info for child widgets
        QWidget *mainParent;
        QWidget *scrollAreaLayout = nullptr;
        QVBoxLayout *vbox = nullptr;

        // data fo storing all of the bag info
        std::vector<BagItem *> bagList;

        rclcpp::Subscription<launch_msgs::msg::ListPids>::SharedPtr baggingStateSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bagTriggerSub;
        rclcpp::Client<launch_msgs::srv::WhoIs>::SharedPtr bagWhoIsClient;

        // request info for the whois client
        std::shared_future<std::shared_ptr<launch_msgs::srv::WhoIs_Response>> whoisFuture;
        int64_t whoisReqId = -1;
        unsigned int timerTick = 0;

        // vector for keeping track of non-local bags
        std::vector<int> nonLocals;

        bool runningAll = false;
        std_msgs::msg::Bool priorBagState;

        void baggingStateCallback(const launch_msgs::msg::ListPids &msg);
        void autonomyTriggerCallback(const std_msgs::msg::Bool & msg);
    };

} // namespace riptide_rviz