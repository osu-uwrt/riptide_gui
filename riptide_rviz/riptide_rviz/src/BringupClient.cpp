#include "riptide_rviz/BringupClient.hpp"

namespace riptide_rviz
{
    using namespace std::placeholders;

    BringupClient::BringupClient(std::string hostName, std::shared_ptr<rclcpp::Node> parentNode, std::shared_ptr<riptide_rviz::RecipeLaunch> recipeLaunch, QVBoxLayout *parent)
    {
        listElement = new Ui_BringupListElement();
        listElement->setupUi(this);
        parent->addWidget(this);

        clientNode = parentNode;

        auto goal_msg = BringupStart::Goal();
        auto send_goal_options = rclcpp_action::Client<BringupStart>::SendGoalOptions();


        //Create two action clients one for bringup start, one for bringup stop
        
    }

    //Callback functions for BringupStart Client

    void BringupClient::BU_start_goal_response_cb(const GHBringupStart::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(clientNode->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(clientNode->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void BU_start_feedback_cb(GHBringupStart::SharedPtr, const std::shared_ptr<const BringupStart::Feedback> feedback)
    {

    }

    void BU_start_result_cb(const GHBringupStart::WrappedResult & result)
    {
        auto pid = result.result->pid;
    }

    //Create two callbacks for start and stop buttons

    BringupClient::~BringupClient()
    {
        delete listElement;
    }

    void BringupClient::checkPids(launch_msgs::msg::ListLaunch launchMsgs)
    {

    }
}