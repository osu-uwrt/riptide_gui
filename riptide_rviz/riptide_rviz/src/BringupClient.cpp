#include "riptide_rviz/BringupClient.hpp"
#include <rviz_common/logging.hpp>

namespace riptide_rviz
{
    using namespace std::placeholders;

    BringupClient::BringupClient(std::string hostName, std::shared_ptr<rclcpp::Node> parentNode, std::shared_ptr<riptide_rviz::RecipeLaunch> recipeLaunch, QVBoxLayout *parent)
    {
        started = false;
        listElement = new Ui_BringupListElement();
        listElement->setupUi(this);
        parent->addWidget(this);

        clientNode = parentNode;
        recipeLaunchData = recipeLaunch;
        
        //Remove .launch.py to save space
        std::string nameCopy = recipeLaunch->name;
        size_t indexOfExt = nameCopy.find(".launch.py");
        if(indexOfExt != std::string::npos)
        {
            nameCopy.erase(indexOfExt, 10);
        }
        QString launchFileName = QString::fromStdString(nameCopy);
        listElement->launchFileLabel->setText(launchFileName);

        QString fullFileName = QString::fromStdString(recipeLaunch->name);
        listElement->launchFileLabel->setToolTip(fullFileName);

        //Change format of progress bar
        auto topicCount = recipeLaunch->topicList.size();
        maxTopics = static_cast<int>(topicCount);
        listElement->progressBar->setMaximum(maxTopics);
        listElement->progressBar->setFormat("%v/%m");

        //Create two action clients one for bringup start, one for bringup stop
        RVIZ_COMMON_LOG_INFO("BringupClient: Hostname is:" + hostName + "/bringup_start");
        bringupStart = rclcpp_action::create_client<BringupStart>(clientNode, hostName + "/bringup_start");
        bringupEnd = rclcpp_action::create_client<BringupEnd>(clientNode, hostName + "/bringup_end");
        connect(listElement->startButton, &QToolButton::clicked, [this](void)
            {startButtonCallback(); });
        connect(listElement->stopButton, &QToolButton::clicked, [this](void)
            {stopButtonCallback(); });
    }

    void BringupClient::stopButtonCallback()
    {
        started = false;
        listElement->stopButton->setDisabled(true);
        auto goal_msg = BringupEnd::Goal();
        goal_msg.pid = pid;
        auto send_goal_options = rclcpp_action::Client<BringupEnd>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&BringupClient::BU_end_goal_response_cb, this, _1);
        send_goal_options.feedback_callback = std::bind(&BringupClient::BU_end_feedback_cb, this, _1, _2);
        send_goal_options.result_callback = std::bind(&BringupClient::BU_end_result_cb, this, _1);
        bringupEnd->async_send_goal(goal_msg, send_goal_options);
    }

    void BringupClient::BU_end_goal_response_cb(const GHBringupEnd::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RVIZ_COMMON_LOG_ERROR("BU_end_goal_response_cb: Server has rejected the goal.");
        }
    }

    void BringupClient::BU_end_feedback_cb(GHBringupEnd::SharedPtr, const std::shared_ptr<const BringupEnd::Feedback> feedback)
    {
        int killedNodeCount = static_cast<int>(feedback->killed_nodes);
        RVIZ_COMMON_LOG_INFO("BU_end_goal_response: Killed complete count is " + std::to_string(killedNodeCount));
        listElement->progressBar->setValue(maxTopics - killedNodeCount);
    }

    void BringupClient::BU_end_result_cb(const GHBringupEnd::WrappedResult & result)
    {
        listElement->progressBar->setValue(listElement->progressBar->maximum());
        listElement->startButton->setEnabled(true);
        listElement->progressBar->setValue(0);
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
        int completedCount = static_cast<int>(feedback->completed_topics);
        listElement->progressBar->setValue(completedCount);
    }

    void BringupClient::BU_start_result_cb(const GHBringupStart::WrappedResult & result)
    {
        pid = result.result->pid;
        RVIZ_COMMON_LOG_INFO("BU_start_result_cb: pid to look for is " + pid);
        listElement->stopButton->setEnabled(true);
        listElement->startButton->setDisabled(true);
        started = true;
    }

    //Create two callbacks for start and stop buttons

    void BringupClient::startButtonCallback()
    {
        RVIZ_COMMON_LOG_DEBUG("startButtonCallback: Launch file start button pressed");
        listElement->progressBar->setValue(0);
        listElement->progressBar->setStyleSheet("");
        listElement->progressBar->setFormat("%v/%m");

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

        listElement->startButton->setDisabled(true);
        listElement->stopButton->setDisabled(true);
    }
    
    void BringupClient::checkPids(launch_msgs::msg::ListLaunch launchMsgs)
    {
        if(started)
        {
            bool seen = false;
            for (auto runningPid : launchMsgs.pids)
            {
                if(pid == runningPid) seen = true;
            }
            
            if(!seen)
            {
                listElement->progressBar->setValue(0);
                listElement->progressBar->setStyleSheet("QProgressBar {color: white; background: rgb(140, 100, 100);}");
                listElement->progressBar->setFormat("ERROR");
                listElement->startButton->setEnabled(true);
                listElement->stopButton->setDisabled(true);
            }
        }
    }
}