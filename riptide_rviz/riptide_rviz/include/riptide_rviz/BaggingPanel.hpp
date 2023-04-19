#pragma once

#include "riptide_rviz/BagItem.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <QTimer>
#include <vector>

#include "ui_BaggingPanel.h"

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

        std::vector<riptide_rviz::BagItem *> bagList;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr baggingStateSub;

        // BaggingTopicModel *topicModel;

        void baggingStateCallback(const std_msgs::msg::Bool &msg);

    };

} // namespace riptide_rviz