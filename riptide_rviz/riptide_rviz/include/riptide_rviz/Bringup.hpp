#pragma once
#include <rclcpp/rclcpp.hpp>
#include <launch_msgs/action/bringup_start.hpp>
#include <launch_msgs/action/bringup_end.hpp>
#include <launch_msgs/msg/list_launch.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include "ui_Bringup.h"
#include <QTimer>
#include <vector>
#include "riptide_rviz/BringupClient.hpp"

#define BRINGUP_PKG "riptide_bringup2"
#define RVIZ_PKG "riptide_rviz"

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
        void bringupFileChanged(const QString &text);

    protected:
        bool event(QEvent *event);

    private:
        // UI Panel instance
        Ui_Bringup *uiPanel;

        rclcpp::Node::SharedPtr clientNode;
        QTimer * spinTimer;

        std::vector<riptide_rviz::BringupClient*> clientList;
        rclcpp::Subscription<launch_msgs::msg::ListLaunch>::SharedPtr listLaunchSub;
        std::string bringupFilesDir;
        QWidget *mainParent;
        QWidget *scrollAreaLayout;
        void createScrollArea();
        void clearScrollArea();
        void listLaunchCallback(const launch_msgs::msg::ListLaunch &msg);
        void stagedBringupTick();
        int stage = 0;
        int totalStages = 0;
        Recipe recipe;
        QTimer *stagedTimer;
        bool startTick = true;

    };

} // namespace riptide_rviz