#include "riptide_rviz/BringupClient.hpp"
#include <rviz_common/logging.hpp>

namespace riptide_rviz
{
    using namespace std::placeholders;

    BringupClient::BringupClient(std::string hostName, std::shared_ptr<rclcpp::Node> parentNode, std::shared_ptr<riptide_rviz::RecipeLaunch> recipeLaunch, QVBoxLayout *parent)
    {
        listElement = new Ui_BringupListElement();
        listElement->setupUi(this);
        parent->addWidget(this);

        clientNode = parentNode;
        recipeLaunchData = recipeLaunch;

        //Create two action clients one for bringup start, one for bringup stop
        RVIZ_COMMON_LOG_DEBUG("BringupClient: Hostname is:" + hostName + "/bringup_start");
        bringupStart = rclcpp_action::create_client<BringupStart>(clientNode, hostName + "/bringup_start");
        connect(listElement->startButton, &QToolButton::clicked, [this](void)
            {startButtonCallback(); });
    }

    //Callback functions for BringupStart Client

    void BringupClient::BU_start_goal_response_cb(const GHBringupStart::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RVIZ_COMMON_LOG_ERROR("BU_start_goal_response_cb: Server has rejected the goal.");
        }
    }

    void BringupClient::BU_start_feedback_cb(GHBringupStart::SharedPtr, const std::shared_ptr<const BringupStart::Feedback> feedback)
    {

    }

    void BringupClient::BU_start_result_cb(const GHBringupStart::WrappedResult & result)
    {
        auto pid = result.result->pid;
    }

    //Create two callbacks for start and stop buttons

    void BringupClient::startButtonCallback()
    {
        RVIZ_COMMON_LOG_DEBUG("startButtonCallback: Launch file start button pressed");
        auto goal_msg = BringupStart::Goal();
        goal_msg.launch_file = recipeLaunchData->name;
        goal_msg.launch_package = recipeLaunchData->package;
        std::vector<launch_msgs::msg::TopicData> topicDataVec;
        for(auto recipeTopicData : recipeLaunchData->topicList)
        {
            launch_msgs::msg::TopicData temp;
            temp.name = recipeTopicData.name;
            temp.type_name = recipeTopicData.type_name;
            if(recipeTopicData.qos_type == "sensor_data")
            {
                temp.qos_type = temp.QOS_SENSOR_DATA;
            }
            else if(recipeTopicData.qos_type == "system_default")
            {
                temp.qos_type = temp.QOS_SYSTEM_DEFAULT;
            }
            topicDataVec.push_back(temp);
        }
        goal_msg.topics = topicDataVec;
        auto send_goal_options = rclcpp_action::Client<BringupStart>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&BringupClient::BU_start_goal_response_cb, this, _1);
        send_goal_options.feedback_callback = std::bind(&BringupClient::BU_start_feedback_cb, this, _1, _2);
        send_goal_options.result_callback = std::bind(&BringupClient::BU_start_result_cb, this, _1);
        bringupStart->async_send_goal(goal_msg, send_goal_options);
    }

    BringupClient::~BringupClient()
    {
        delete listElement;
    }

    void BringupClient::checkPids(launch_msgs::msg::ListLaunch launchMsgs)
    {

    }
}