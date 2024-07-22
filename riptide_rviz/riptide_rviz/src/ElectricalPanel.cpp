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
        setStatus("", false);
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
        std::string 
            fullMagCalActionName = robotNs.toStdString() + MAG_CAL_ACTION_NAME,
            fullTareGyroActionName = robotNs.toStdString() + TARE_GYRO_ACTION_NAME;

        imuCalClient = rclcpp_action::create_client<MagCal>(node, fullMagCalActionName);
        tareGyroClient = rclcpp_action::create_client<TareGyro>(node, fullTareGyroActionName);
        loaded = true;
    }


    void ElectricalPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }


    void ElectricalPanel::onInitialize()
    {
        connect(ui->commandSend, &QPushButton::clicked, this, &ElectricalPanel::sendElectricalCommand);
        connect(ui->magCalSend, &QPushButton::clicked, this, &ElectricalPanel::sendMagCal);
        connect(ui->commandTareFog, &QPushButton::clicked, this, &ElectricalPanel::sendTareGyro);

        //initial UI state
        ui->calibProgress->setValue(0);
        setStatus("", false);
        ui->magCalSend->setText("Calibrate");
    }


    void ElectricalPanel::sendElectricalCommand()
    {
        if(loaded)
        {
            riptide_msgs2::msg::ElectricalCommand msg;
            msg.command = ui->commandSelect->currentIndex();
            pub->publish(msg);
            setStatus("", false);
        } else 
        {
            setStatus("Panel not loaded! Please save your config and restart RViz", true);
        }
    }


    void ElectricalPanel::sendMagCal()
    {
        if(! loaded){
            setStatus("Panel not loaded! Please save your config and restart RViz", true);
            return;
        }

        if(imuCalInProgress){
            imuCalClient->async_cancel_all_goals();
            setStatus("Cancelling calibration request", false);
            return;
        }

        // reset the panel state for cal
        ui->calibProgress->setValue(0);
        maxVar = 1e-10;

        // make sure the cal server is online
        if(!imuCalClient->wait_for_action_server(1s)){
            setStatus("Calibration server unavailiable!", true);
            return;
        }

        // create and send goal
        MagCal::Goal cal_goal;
        MagSendGoalOptions options;
        options.goal_response_callback  = std::bind(&ElectricalPanel::magCalGoalResponseCb, this, _1);
        options.feedback_callback       = std::bind(&ElectricalPanel::magCalFeedbackCb, this, _1, _2);
        options.result_callback         = std::bind(&ElectricalPanel::magCalResultCb, this, _1);
        imuCalClient->async_send_goal(cal_goal, options);
        imuCalInProgress = true;

        ui->magCalSend->setText("Cancel");
    }


    void ElectricalPanel::sendTareGyro()
    {
        if(! loaded){
            setStatus("Panel not loaded! Please save your config and restart RViz", true);
            return;
        }

        if(gyroTareInProgress){
            tareGyroClient->async_cancel_all_goals();
            setStatus("Cancelling tare request", false);
            return;
        }
        
        setStatus("Sending gyro tare goal", false);

        // reset the panel state for cal
        ui->calibProgress->setValue(0);

        // make sure the cal server is online
        if(!tareGyroClient->wait_for_action_server(1s)){
            setStatus("Tare server unavailiable!", true);
            return;
        }

        // create and send goal
        TareGyro::Goal tare_goal;
        tare_goal.num_samples = ui->tareSamples->value();
        tare_goal.timeout_seconds = ui->tareTimeout->value();
        TareGyroSendGoalOptions options;
        options.goal_response_callback  = std::bind(&ElectricalPanel::tareGyroGoalResponseCb, this, _1);
        options.result_callback         = std::bind(&ElectricalPanel::tareGyroResultCb, this, _1);
        tareGyroClient->async_send_goal(tare_goal, options);
        ui->commandTareFog->setText("Cancel");
    }


    void ElectricalPanel::setStatus(const QString& status, bool error)
    {
        ui->errLabel->setText(status);
        ui->errLabel->setStyleSheet(error ? "color: red" : "");

        if(error)
        {
            RVIZ_COMMON_LOG_ERROR(status.toStdString());
        } else
        {
            RVIZ_COMMON_LOG_INFO(status.toStdString());
        }
    }


    void ElectricalPanel::magCalGoalResponseCb(const MagGoalHandle::SharedPtr & goal_handle){
        if(goal_handle)
        {
            switch(goal_handle->get_status())
            {
                case GOAL_STATE_ACCEPTED:
                    setStatus("Performing mag cal", false);
                    break;
                case GOAL_STATE_CANCELING:
                    setStatus("Canceling calibration...", false);
                    break;
                default:
                    setStatus("Unknown goal state", true);
                    break;
            }
        } else
        {
            setStatus("Calibration request rejected!", true);
        }   
    }


    void ElectricalPanel::magCalFeedbackCb( MagGoalHandle::SharedPtr,
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


    void ElectricalPanel::magCalResultCb(const MagGoalHandle::WrappedResult & result){
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                ui->calibProgress->setValue(100);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                setStatus("Calibration aborted, Examine driver logs for info", true);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                setStatus("Calibration canceled!", false);
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                setStatus("Uknown calibration result recieved???", true);
                break;
        }

        imuCalInProgress = false;
        ui->magCalSend->setText("Calibrate");
    }


    void ElectricalPanel::tareGyroGoalResponseCb(const TareGyroGoalHandle::SharedPtr & goal_handle){
        if(goal_handle)
        {
            switch(goal_handle->get_status())
            {
                case GOAL_STATE_ACCEPTED:
                    setStatus("Performing gyro tare", false);
                    gyroTareInProgress = true;
                    break;
                case GOAL_STATE_CANCELING:
                    setStatus("Canceling tare", false);
                    break;
                default:
                    setStatus("Unknown goal state", true);
                    break;
            }
        } else
        {
            setStatus("Tare request rejected!", true);
        }   
    }


    void ElectricalPanel::tareGyroResultCb(const TareGyroGoalHandle::WrappedResult & result){
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                ui->calibProgress->setValue(100);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                setStatus("Tare aborted: " + QString::fromStdString(result.result->result), true);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                setStatus("Tare canceled!", false);
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                setStatus("Unknown tare result recieved???", true);
                break;
        }

        gyroTareInProgress = false;
        ui->commandTareFog->setText("Calibrate");
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ElectricalPanel, rviz_common::Panel);
