#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "riptide_rviz/bringup_recipe.hpp"
#include <QVBoxLayout>
#include <string>
#include <QWidget>
#include "launch_msgs/msg/list_launch.hpp"
#include "ui_BringupListElement.h"

namespace riptide_rviz
{
    class BringupClient : public QWidget
    {
        public:
            BringupClient(std::string hostName, std::shared_ptr<rclcpp::Node> parentNode, std::shared_ptr<riptide_rviz::RecipeLaunch> recipeLaunch, QVBoxLayout *parent);
            void checkPids(launch_msgs::msg::ListLaunch launchMsgs);

        private:
            Ui_BringupListElement *listElement;
    };
}