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
#include "ui_BringupListElement.h"
#include <QTimer>

#define BRINGUP_PKG "riptide_bringup2"
#define RVIZ_PKG "riptide_rviz"
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
    };

} // namespace riptide_rviz