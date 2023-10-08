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
        std::string fullActionName = "/" + CALIB_ACTION_NAME;
        imuCalClient = rclcpp_action::create_client<MagCal>(node, fullActionName);

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

        //initial UI state
        ui->calibProgress->setValue(0);
        ui->errLabel->setText("");
        ui->magCalSend->setText("Calibrate");
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
        switch(result.code)
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
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ElectricalPanel, rviz_common::Panel);
