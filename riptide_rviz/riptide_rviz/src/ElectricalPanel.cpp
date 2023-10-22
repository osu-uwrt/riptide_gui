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
        std::string fullActionName = CALIB_ACTION_NAME;
        imuCalClient = rclcpp_action::create_client<MagCal>(node, fullActionName);

        // Make publisher to write IMU config
        writeImuConfig = node->create_publisher<riptide_msgs2::msg::ImuConfig>("vectornav/config/write", 10);

        // Make subscriber to listen for IMU config
        readImuConfig = node->create_subscription<riptide_msgs2::msg::ImuConfig>("vectornav/config/read", 10, 
                                    std::bind(&ElectricalPanel::imuConfigCb, this, _1));

        requestCurrentImuConfig();

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
        connect(ui->hsiEnable, &QCheckBox::stateChanged, this, &ElectricalPanel::handleMagCalMode);
        connect(ui->hsiOutput,  &QCheckBox::stateChanged, this, &ElectricalPanel::handleMagOutputMode);
        connect(ui->convergenceRate, &QSlider::sliderReleased, this, &ElectricalPanel::handleConvergenceRate);

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

    // Handle checkboxes for realtime HSI measure and output mode
    void ElectricalPanel::handleMagCalMode() {
        imuHsiEnable = ui->hsiEnable->checkState();
        publishImuConfig();
    }

    void ElectricalPanel::handleMagOutputMode() {
        imuHsiOutput = ui->hsiOutput->checkState();
        publishImuConfig();
    }

    // Handle slider for realtime HSI convergence rate
    void ElectricalPanel::handleConvergenceRate() {
        imuConvergenceRate = ui->convergenceRate->value();
        publishImuConfig();
    }

    // Publish the requested realtime HSI config to the IMU driver
    void ElectricalPanel::publishImuConfig() {
        auto msg = riptide_msgs2::msg::ImuConfig();
        msg.hsi_enable = imuHsiEnable;
        msg.hsi_output = imuHsiOutput;
        msg.convergence_rate = imuConvergenceRate;

        writeImuConfig->publish(msg);
    }

    // Publish a request to IMU driver to ask for current realtime HSI config
    void ElectricalPanel::requestCurrentImuConfig() {
        RVIZ_COMMON_LOG_INFO("Electrical panel: Requesting IMU config");
        // Sleep for a second to make sure everything else is loaded
        rclcpp::Rate loopRate(1);
        loopRate.sleep();
        auto resendRequest = riptide_msgs2::msg::ImuConfig();
        // 100 asks the imu driver to publish its current settings
        resendRequest.convergence_rate = 100;
        writeImuConfig->publish(resendRequest);
    }

    // Get the current realtime HSI config from IMU driver
    void ElectricalPanel::imuConfigCb(const riptide_msgs2::msg::ImuConfig config) {
        // Set class-scoped variables
        imuHsiEnable = config.hsi_enable;
        imuHsiOutput = config.hsi_output;
        imuConvergenceRate = config.convergence_rate;

        // Write values to the gui elements
        ui->hsiEnable->setCheckState(processCheckState(config.hsi_enable));
        ui->hsiOutput->setCheckState(processCheckState(config.hsi_output));
        ui->convergenceRate->setValue(config.convergence_rate);
    }

    // Process bool into enum
    Qt::CheckState ElectricalPanel::processCheckState(bool state) {
        return state ? Qt::CheckState::Checked : Qt::CheckState::Unchecked;
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ElectricalPanel, rviz_common::Panel);
