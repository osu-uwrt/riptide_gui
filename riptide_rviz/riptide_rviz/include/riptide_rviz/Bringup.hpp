#pragma once
#include <rclcpp/rclcpp.hpp>
#include <launch_msgs/srv/start_launch.hpp>
#include <launch_msgs/srv/list_launch.hpp>
#include <launch_msgs/srv/stop_launch.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include "ui_Bringup.h"
#include <QTimer>

#define BRINGUP_PKG "riptide_bringup2"
#define BRINGUP_POLLING_RATE 1s

namespace riptide_rviz
{
    class Bringup : public rviz_common::Panel
    {
        Q_OBJECT public : Bringup(QWidget *parent = 0);
        ~Bringup();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

    protected Q_SLOTS:
        // QT slots (function callbacks)
        void bringupListRefresh();
        void handleBringupHost(int selection);
        void startBringup();
        void checkBringupStatus();
        void stopBringup();

    protected:
        bool event(QEvent *event);

    private:
        // UI Panel instance
        Ui_Bringup *uiPanel;

        rclcpp::Node::SharedPtr clientNode;
        QTimer * spinTimer;

        // Bringup clients
        rclcpp::Client<launch_msgs::srv::StartLaunch>::SharedPtr bringupStartClient;
        rclcpp::Client<launch_msgs::srv::ListLaunch>::SharedPtr bringupListClient;
        rclcpp::Client<launch_msgs::srv::StopLaunch>::SharedPtr bringupStopClient;
        QTimer * bringupCheckTimer;
        int bringupID = -1;
    };

} // namespace riptide_rviz