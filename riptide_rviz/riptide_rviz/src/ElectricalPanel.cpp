#include "riptide_rviz/ElectricalPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <math.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace riptide_rviz
{
    ElectricalPanel::ElectricalPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_ElectricalPanel();
        ui->setupUi(this);
        ui->errLabel->setText("");
        loaded = false;
    }


    ElectricalPanel::~ElectricalPanel()
    {
        delete ui;
    }


    void ElectricalPanel::load(const rviz_common::Config &config) 
    {
        config.mapGetString("robot_namespace", &robotNs);
        if(robotNs == "")
        {
            robotNs = QString::fromStdString("/talos"); 
            RVIZ_COMMON_LOG_WARNING("ElectricalPanel: Using /talos as the default value for robot_namespace"); 
        }

        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // make the publisher for electrical command
        std::string topicName = robotNs.toStdString() + "/command/electrical";
        pub = node->create_publisher<riptide_msgs2::msg::ElectricalCommand>(topicName, 10);

        //make the action client for the imu mag cal
        std::string fullActionName = robotNs.toStdString() + CALIB_ACTION_NAME;
        imuCalClient = rclcpp_action::create_client<MagCal>(node, fullActionName);

        // Make client for imu register config
        std::string fullServiceName = robotNs.toStdString() + CONFIG_SERVICE_NAME;
        imuConfigClient = node->create_client<ImuConfig>(fullServiceName);

        loaded = true;
    }


    void ElectricalPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }


    void ElectricalPanel::onInitialize()
    {
        connect(ui->commandSend, &QPushButton::clicked, this, &ElectricalPanel::sendCommand);
        connect(ui->magCalSend, &QPushButton::clicked, this, &ElectricalPanel::sendMagCal);

        connect(ui->imuRead, &QPushButton::clicked, this, &ElectricalPanel::readIMU);
        connect(ui->imuWrite, &QPushButton::clicked, this, &ElectricalPanel::writeIMU);

        //initial UI state
        ui->calibProgress->setValue(0);
        ui->errLabel->setText("");
        ui->magCalSend->setText("Calibrate");

        ui->registerNum->setText("");
        ui->registerData->setText("");
    }


    void ElectricalPanel::sendCommand()
    {
        if(loaded)
        {
            riptide_msgs2::msg::ElectricalCommand msg;
            msg.command = ui->commandSelect->currentIndex();
            pub->publish(msg);
            ui->errLabel->setText("");
        } else 
        {
            ui->errLabel->setText("Panel not loaded! Please save your config and restart RViz");
        }
    }

    void ElectricalPanel::sendMagCal(){
        if(! loaded){
            ui->errLabel->setText("Panel not loaded! Please save your config and restart RViz");
            return;
        }

        if(calInProgress){
            imuCalClient->async_cancel_all_goals();
            ui->errLabel->setText("Cancelling calibration request");
            return;
        }

        // reset the panel state for cal
        ui->calibProgress->setValue(0);
        maxVar = 1e-10;

        // make sure the cal server is online
        if(!imuCalClient->wait_for_action_server(1s)){
            RVIZ_COMMON_LOG_ERROR("ElectricalPanel: Mag calibration action server not available!");
            ui->errLabel->setText("Calibration server unavailiable!");
            return;
        }

        // create and send goal
        MagCal::Goal cal_goal;
        MagSendGoalOptions options;
        options.goal_response_callback  = std::bind(&ElectricalPanel::goalResponseCb, this, _1);
        options.feedback_callback       = std::bind(&ElectricalPanel::feedbackCb, this, _1, _2);
        options.result_callback         = std::bind(&ElectricalPanel::resultCb, this, _1);
        imuCalClient->async_send_goal(cal_goal, options);
        calInProgress = true;

        ui->magCalSend->setText("Cancel");
    }

    void ElectricalPanel::goalResponseCb(const MagGoalHandle::SharedPtr & goal_handle){
        if(goal_handle)
        {
            switch(goal_handle->get_status())
            {
                case GOAL_STATE_ACCEPTED:
                    ui->errLabel->setText("");
                    break;
                case GOAL_STATE_CANCELING:
                    ui->errLabel->setText("Canceling calibration...");
                    break;
                default:
                    ui->errLabel->setText("Unknown goal state");
                    break;
            }
        } else
        {
            ui->errLabel->setText("Calibration request rejected!");
        }   
    }

    void ElectricalPanel::feedbackCb( MagGoalHandle::SharedPtr,
        const std::shared_ptr<const MagCal::Feedback> feedback){


        // compute the new total variance
        double sum_sqaured = 0.0;
        for(double variance : feedback->curr_avg_dev){
            sum_sqaured += variance * variance;
        }
        double total_var = sqrtf64(sum_sqaured);

        // figure out if this is a new max
        if(total_var > maxVar){
            maxVar = total_var;
        }

        // take max divided by current
        float disp_var = 100 * (1.0 - total_var / maxVar);

        // show it to the user
        ui->calibProgress->setValue((int)disp_var);
    }

    void ElectricalPanel::resultCb(const MagGoalHandle::WrappedResult & result){
        switch(result.code)  //     auto request = std::make_shared<ImuConfig::Request>();
    //     request->request = "test";

    //     while (!imuConfigClient->wait_for_service(1s)) {
    //         if (!rclcpp::ok()) {
    //             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    //             return;
    //         }
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    //     }

    // auto result = imuConfigClient->async_send_request(request);
    // // Wait for the result.
    // if (rclcpp::spin_until_future_complete(this, result) == rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    // } else {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    // }

    // }
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                ui->calibProgress->setValue(100);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                ui->errLabel->setText("Calibration aborted, Examine driver logs for info");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                ui->errLabel->setText("Calibration canceled!");
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                ui->errLabel->setText("Uknown calibration result recieved???");
                break;
        }

        calInProgress = false;
        ui->magCalSend->setText("Calibrate");
    }

    void ElectricalPanel::sendIMUConfigRequest(const std::string& requestStr) {
    //     if (!imuConfigClient->wait_for_service(1s)) {
    //         ui->registerData->setText("Config server unavailable");
    //         RVIZ_COMMON_LOG_ERROR("ElectricalPanel: IMU config server not available");
    //         return;
    //     }

    //     auto request = std::make_shared<ImuConfig::Request>();
    //     request->request = requestStr;
    //     auto futureFeedback = imuConfigClient->async_send_request(request);

    //     if (futureFeedback.wait_for(1s) != std::future_status::ready) {
    //         ui->registerData->setText("Config service busy");
    //         RVIZ_COMMON_LOG_WARNING("ElectricalPanel: IMU config service busy");
    //         // return;
    //     }

    //     try {
    //         // ui->registerData->setText(futureFeedback.get()->response.c_str());
    //         std::string test = futureFeedback.get()->response;
    //     }
    //     catch(...) {
    //         RVIZ_COMMON_LOG_ERROR("ElectricalPanel: Caught error parsing config feedback");
    //     }

    //     // ui->registerData->setText(futureFeedback.get()->response.c_str());
    //     ui->registerData->setText("Service success");
    // }

    // void ElectricalPanel::readIMU() {
    //     ui->registerData->setText("Sending IMU read request");
    //     std::string requestStr = "$VNRRG," + ui->registerNum->text().toStdString();
    //     sendIMUConfigRequest(requestStr);
    //     // ui->valuesTxt->setText(imuConfigClient->async_send_request(request).get()->response.c_str());
    // }

    // void ElectricalPanel::writeIMU() {
    //     auto request = std::make_shared<ImuConfig::Request>();
    //     request->request = "$VNWRG," + ui->registerNum->text().toStdString() + "," + ui->registerData->text().toStdString();
    //     ui->valuesTxt->setText(imuConfigClient->async_send_request(request).get()->response.c_str());
    // }

        // yoink local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        auto start = node->get_clock()->now();
        while (!imuConfigClient->wait_for_service(100ms)) {
            if (node->get_clock()->now() - start > 1s || rclcpp::ok()) {
                ui->registerData->setText("Config server unavailable");
                return;
            }
        }

        auto request = std::make_shared<ImuConfig::Request>();
        auto imuConfigFutureInfo = imuConfigClient->async_send_request(request);
        imuConfigFuture = imuConfigFutureInfo.share();
        imuConfigFutureId = imuConfigFutureInfo.request_id;

        timerTick = 0;
        QTimer::singleShot(250, [this]() { waitForConfig(); });
    }

    void ElectricalPanel::waitForConfig() {
        if (!imuConfigFuture.valid()) {
            ui->registerData->setText("Future invalidated");
            return;   
        }

        auto futureStatus = imuConfigFuture.wait_for(10ms);
        if (futureStatus == std::future_status::timeout && timerTick < 10) {
            QTimer::singleShot(250, [this]() {waitForConfig();});
            timerTick++;
        }
        else if (futureStatus != std::future_status::timeout) {
            // Future validated, request returned
            auto response = imuConfigFuture.get();
            ui->registerData->setText("Got response");
        }
        else if (timerTick >= 10) {
            ui->registerData->setText("Config service never responded");
            imuConfigClient->remove_pending_request(imuConfigFutureId);
        }
    }

}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ElectricalPanel, rviz_common::Panel);
