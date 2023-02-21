#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "riptide_rviz/bringup_recipe.hpp"
#include <QVBoxLayout>
#include <string>
#include <QWidget>
#include "ui_BringupListElement.h"

// Message headers
#include <launch_msgs/action/bringup_end.hpp>
#include <launch_msgs/msg/list_launch.hpp>
#include <launch_msgs/action/bringup_start.hpp>

namespace riptide_rviz
{
    using BringupStart = launch_msgs::action::BringupStart;
    using GHBringupStart = rclcpp_action::ClientGoalHandle<BringupStart>;

    class BringupClient : public QWidget
    {
        public:
            BringupClient(std::string hostName, std::shared_ptr<rclcpp::Node> parentNode, std::shared_ptr<riptide_rviz::RecipeLaunch> recipeLaunch, QVBoxLayout *parent);
            void checkPids(launch_msgs::msg::ListLaunch launchMsgs);
            ~BringupClient();

        private:
            Ui_BringupListElement *listElement;
            rclcpp::Node::SharedPtr clientNode;
            rclcpp_action::Client<BringupStart>::SharedPtr bringupStart;
            std::shared_ptr<riptide_rviz::RecipeLaunch> recipeLaunchData;
            void BU_start_goal_response_cb(const GHBringupStart::SharedPtr & goal_handle);
            void BU_start_feedback_cb(GHBringupStart::SharedPtr, const std::shared_ptr<const BringupStart::Feedback> feedback);
            void BU_start_result_cb(const GHBringupStart::WrappedResult & result);
            void startButtonCallback();

    };
}