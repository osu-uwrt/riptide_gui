#include "riptide_rviz/BagItem.hpp"

#include <riptide_rviz/BaggingConfiguration.hpp>

#include "ui_BaggingTopicSelection.h"

#include <QTimer>

#include <algorithm>
#include <chrono>

#include <rviz_common/logging.hpp>

using namespace std::chrono_literals;

namespace riptide_rviz
{
    BagItem::BagItem(std::string hostName, std::shared_ptr<RecipeBag> bagData, std::shared_ptr<rclcpp::Node> parentNode, int local)
    {
        uiPanel = new Ui_BagListElement();
        uiPanel->setupUi(this);

        isLocal = local < 0;

        RVIZ_COMMON_LOG_INFO("BagItem: loading bag " + bagData->name);

        if (isLocal)
        {
            this->bagData = bagData;
            loadTopics(bagData->topicList);

            uiPanel->bagName->setText(QString::fromStdString(bagData->name));

            // set the default state
            pid = -1;
            stateStopped();
        }
        else
        {
            uiPanel->bagName->setText(QString::fromStdString("NL: " + bagData->name));

            // for non-local save the PID
            pid = local;

            // also set it into the running state
            stateRunning();
        }

        // conect the buttons
        connect(uiPanel->bagCfg, &QPushButton::clicked, [this](void)
                { baggingConfigure(); });
        connect(uiPanel->startButton, &QPushButton::clicked, [this](void)
                { startBagging(); });
        connect(uiPanel->stopButton, &QPushButton::clicked, [this](void)
                { stopBagging(); });

        // create the service clients
        bagStartClient = parentNode->create_client<launch_msgs::srv::StartBag>(hostName + "/bag_start");
        bagStopClient = parentNode->create_client<launch_msgs::srv::StopBag>(hostName + "/bag_stop");

        RVIZ_COMMON_LOG_INFO("BagItem: created child item ");
    }

    BagItem::~BagItem()
    {
        delete uiPanel;
    }

    bool BagItem::event(QEvent *event)
    {
        return false;
    }

    void BagItem::baggingConfigure()
    {
        RVIZ_COMMON_LOG_INFO("BagItem: configuring " + uiPanel->bagName->text().toStdString());

        QDialog *selectionDialog = new QDialog();

        Ui_BaggingTopicSelection uiSelection;
        uiSelection.setupUi(selectionDialog);

        auto topicView = new BaggingTopicModel(bagData->topicList, selectionDialog);
        uiSelection.topicSelector->setModel(topicView);

        uiSelection.topicSelector->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

        selectionDialog->exec();

        auto newTopicConfig = topicView->getTopicConfig();

        // re-read the topics from the updates
        bagData->topicList = newTopicConfig;
        loadTopics(newTopicConfig);

        delete topicView;
        delete selectionDialog;
    }

    void BagItem::startBagging()
    {
        if (pid >= 0)
        {
            // cant start wont start, already started
            return;
        }

        if (!isLocal)
        {
            RVIZ_COMMON_LOG_ERROR("BagItem: Tried to start a non-local service");

            uiPanel->startButton->setToolTip("Cannot start a non-local service");

            return;
        }

        // now we can do the thing
        RVIZ_COMMON_LOG_INFO("BagItem: starting bag " + uiPanel->bagName->text().toStdString());

        // check that startt service is alive and ready
        if (!bagStartClient->service_is_ready())
        {
            RVIZ_COMMON_LOG_ERROR("BagItem: start bag service is not ready, please try again later");

            uiPanel->startButton->setToolTip("Start bag service not ready");

            return;
        }
        else
        {
            uiPanel->startButton->setToolTip("");
        }

        // set the ui element states
        stateChanging();

        // prepare the request
        auto req = std::make_shared<launch_msgs::srv::StartBag::Request>();
        getName(req->file_name);
        req->topics = topics;

        // send the request over
        auto futWithReq = bagStartClient->async_send_request(req);
        startFuture = futWithReq.share();
        startReqId = futWithReq.request_id;

        // auto fut = startFuture.future

        // clear the timer ticks
        timerTick = 0;

        // set a oneshot timer for 1s into the future and check status
        QTimer::singleShot(1000, [this]()
                           { startupTimer(); });
    }

    void BagItem::startupTimer()
    {
        auto futureStatus = startFuture.wait_for(10ms);
        if (futureStatus == std::future_status::timeout && timerTick < 10)
        {
            QTimer::singleShot(1000, [this]()
                               { startupTimer(); });

            timerTick++;
        }
        else if (futureStatus != std::future_status::timeout)
        {
            auto resp = startFuture.get();

            // parse the response
            if (resp->err_code == launch_msgs::srv::StartBag::Response::ERR_NONE)
            {
                // we have fully started
                stateRunning();

                pid = resp->pid;
            }
            else
            {
                QString btnTip;
                switch (resp->err_code)
                {
                case launch_msgs::srv::StartBag::Response::ERR_MISSING_ARGS:
                    btnTip = "Internal error occured on bagging server";
                    break;
                case launch_msgs::srv::StartBag::Response::ERR_MISSING_TOPICS:
                    btnTip = "Server recieved no topics to bag";
                    break;
                case launch_msgs::srv::StartBag::Response::ERR_OPENING_BAG:
                    btnTip = "Server failed to open bag file";
                    break;
                default:
                    btnTip = "Server returned an unknown error";
                }

                // set the tooltip for the status
                uiPanel->startButton->setToolTip(btnTip);

                stateStopped();
            }

            startReqId = -1;
        }
        else if (timerTick > 10)
        {
            RVIZ_COMMON_LOG_ERROR("BagItem: Start bag service never replied");

            uiPanel->startButton->setToolTip("Start bag service never replied");
            bagStartClient->remove_pending_request(startReqId);

            startReqId = -1;
        }
    }

    bool BagItem::bagAlive(std::vector<int> pids)
    {
        if (std::find(pids.begin(), pids.end(), pid) == pids.end())
        {
            pid = -1;

            // TODO show the bag died!
            stateStopped();

            return false;
        }

        return true;
    }

    int BagItem::getPid()
    {
        return pid;
    }

    bool BagItem::local()
    {
        return isLocal;
    }

    bool BagItem::isStarting(){
        return startReqId > -1;
    }

    void BagItem::getName(std::string &name)
    {
        name = uiPanel->bagName->text().toStdString();
    }

    void BagItem::stopBagging()
    {
        if (pid < 0)
        {
            // cant stop wont stop, already stopped
            return;
        }

        // now we can send the stop request
        RVIZ_COMMON_LOG_INFO("BagItem: stopping bag " + uiPanel->bagName->text().toStdString());

        // check that startt service is alive and ready
        if (!bagStartClient->service_is_ready())
        {
            RVIZ_COMMON_LOG_ERROR("BagItem: start bag service is not ready, please try again later");

            uiPanel->stopButton->setToolTip("Start bag service not ready");
        }
        else
        {
            uiPanel->stopButton->setToolTip("");
        }

        // set the ui element states
        stateChanging();

        // prepare the request
        auto req = std::make_shared<launch_msgs::srv::StopBag::Request>();
        req->pid = pid;

        // send the request over
        // send the request over
        auto futWithReq = bagStopClient->async_send_request(req);
        stopFuture = futWithReq.share();
        stopReqId = futWithReq.request_id;

        // clear the timer ticks
        timerTick = 0;

        // set a oneshot timer for 1s into the future and check status
        QTimer::singleShot(1000, [this]()
                           { shutdownTimer(); });
    }

    bool BagItem::isStopping(){
        return stopReqId > -1;
    }

    void BagItem::shutdownTimer()
    {
        auto futureStatus = stopFuture.wait_for(10ms);
        if (futureStatus == std::future_status::timeout && timerTick < 10)
        {
            QTimer::singleShot(1000, [this]()
                               { shutdownTimer(); });
            timerTick++;
        }
        else if (futureStatus != std::future_status::timeout)
        {
            auto resp = stopFuture.get();

            // parse the response
            if (resp->err_code = launch_msgs::srv::StopBag::Response::ERR_NONE)
            {
                // we have fully stopped
                stateStopped();

                pid = -1;
            }
            else
            {
                QString btnTip;
                switch (resp->err_code)
                {
                case launch_msgs::srv::StopBag::Response::ALR_DEAD:
                    btnTip = "Bag was already dead";
                    break;
                case launch_msgs::srv::StopBag::Response::BAD_PERM:
                    btnTip = "Server could not send signal to bag";
                    break;
                default:
                    btnTip = "Server returned an unknown error on stop";
                }

                // set the tooltip for the status
                uiPanel->stopButton->setToolTip(btnTip);

                stateStopped();
            }

            stopReqId = -1;
        }
        else if (timerTick > 10)
        {
            RVIZ_COMMON_LOG_ERROR("BagItem: Stop bag service never replied");

            uiPanel->startButton->setToolTip("Stop bag service never replied");
            bagStopClient->remove_pending_request(stopReqId);

            stopReqId = -1;
        }
    }

    void BagItem::loadTopics(const std::vector<RecipeTopicData> &topicList)
    {
        topics.clear();

        RVIZ_COMMON_LOG_INFO("BagItem: loading topics ");

        for (auto recipeTopic : topicList)
        {
            launch_msgs::msg::TopicData data;

            data.name = recipeTopic.name;
            data.type_name = recipeTopic.type_name;
            if (recipeTopic.qos_type == "sensor_data")
            {
                data.qos_type = data.QOS_SENSOR_DATA;
            }
            else if (recipeTopic.qos_type == "system_default")
            {
                data.qos_type = data.QOS_SYSTEM_DEFAULT;
            }

            RVIZ_COMMON_LOG_INFO("BagItem: adding topic " + data.name + " " + data.type_name + " " + std::to_string(data.qos_type));

            topics.push_back(data);
        }
    }

    void BagItem::stateStopped()
    {
        uiPanel->startButton->setEnabled(isLocal);
        uiPanel->stopButton->setEnabled(false);
        uiPanel->bagName->setEnabled(true);
        uiPanel->bagCfg->setEnabled(true);
    }
    void BagItem::stateChanging()
    {
        uiPanel->startButton->setEnabled(false);
        uiPanel->stopButton->setEnabled(false);
        uiPanel->bagName->setEnabled(false);
        uiPanel->bagCfg->setEnabled(false);
    }
    void BagItem::stateRunning()
    {
        uiPanel->startButton->setEnabled(false);
        uiPanel->stopButton->setEnabled(true);
        uiPanel->bagName->setEnabled(false);
        uiPanel->bagCfg->setEnabled(false);
    }

} // namespace riptide_rviz
