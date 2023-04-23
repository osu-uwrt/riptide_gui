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
        void stopBagging();

    protected:
        bool event(QEvent *event);

    private:
        // UI Panel instance
        Ui_BaggingPanel *uiPanel;

        // parent info for child widgets
        QWidget *mainParent;
        QWidget *scrollAreaLayout = nullptr;
        QVBoxLayout *vbox = nullptr;

        // data fo storing all of the bag info
        std::vector<riptide_rviz::BagItem *> bagList;

        rclcpp::Subscription<launch_msgs::msg::ListPids>::SharedPtr baggingStateSub;
        rclcpp::Client<launch_msgs::srv::WhoIs>::SharedPtr bagWhoIsClient;


        // BaggingTopicModel *topicModel;

        void baggingStateCallback(const launch_msgs::msg::ListPids &msg);

    };

} // namespace riptide_rviz