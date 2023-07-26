#include "riptide_rviz/MappingPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

const static std::string CALIB_ACTION_NAME = "map/model_tf";

namespace riptide_rviz
{
    std::string getFromConfig(const rviz_common::Config &config, const QString& key, const QString& defaultValue)
    {
        QString str;
        config.mapGetString(key, &str);
        if(str == "")
        { 
            str = defaultValue;
            RVIZ_COMMON_LOG_WARNING("MappingPanel: Using " + defaultValue.toStdString() + " as the default value for " + key.toStdString()); 
        }

        return str.toStdString();
    }

    MappingPanel::MappingPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_MappingPanel();
        ui->setupUi(this);

        ui->calibStatus->setText("");

        loaded = false;
    }


    MappingPanel::~MappingPanel()
    {
        delete ui;
    }


    void MappingPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
        RVIZ_COMMON_LOG_INFO("MappingPanel: Loaded parent panel config");

        robotNs = getFromConfig(config, "robot_namespace", "/talos");
        ui->worldFrame->setText(QString::fromStdString(getFromConfig(config, "worldFrameName", "world")));
        ui->tagFrame->setText(QString::fromStdString(getFromConfig(config, "tagFrameName", "tag_36h11")));
        ui->numSamples->setValue(std::stoi(getFromConfig(config, "mapCalibNumSamples", "10")));

        std::string fullActionName = robotNs + "/" + CALIB_ACTION_NAME;
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        calibClient = rclcpp_action::create_client<ModelFrame>(node, fullActionName);
        RVIZ_COMMON_LOG_INFO("Created action client for server with name \"" + fullActionName + "\"");
        loaded = true;
    }

    void MappingPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        // write our config values
        config.mapSetValue("robot_namespace", QString::fromStdString(robotNs));
        config.mapSetValue("worldFrameName", ui->worldFrame->text());
        config.mapSetValue("tagFrameName", ui->tagFrame->text());
        config.mapSetValue("mapCalibNumSamples", ui->numSamples->value());
    }


    void MappingPanel::onInitialize()
    {
        //initial panel state
        calibrationInProgress = false;

        //initial UI state
        ui->calibProgress->setValue(0);

        // Connect UI signals for controlling the riptide vehicle
        connect(ui->calibButton, &QPushButton::clicked, this, &MappingPanel::calibMapFrame);
    }


    void MappingPanel::calibMapFrame()
    {   
        if(!loaded)
        {
            setCalibStatus("Panel not loaded! Please save your config and restart RViz.", "FF0000");
            return;
        }

        //if the calibration is in progress, cancel it
        if(calibrationInProgress)
        {
            calibClient->async_cancel_all_goals();
            setCalibStatus("Canceling calibration...", "000000");
            return;
        }

        ui->calibProgress->setValue(0);

        if(!calibClient->wait_for_action_server(1s))
        {
            RVIZ_COMMON_LOG_ERROR("MappingPanel: Map calibration action server not available!");
            setCalibStatus("Action server not available!", "FF0000");
            return;
        }

        int numSamples = ui->numSamples->value();
        ui->calibProgress->setRange(0, numSamples);

        ModelFrame::Goal calibGoal;
        calibGoal.monitor_parent       = ui->worldFrame->text().toStdString();
        calibGoal.monitor_child        = ui->tagFrame->text().toStdString();
        calibGoal.samples              = numSamples;

        SendGoalOptions options;
        options.goal_response_callback  = std::bind(&MappingPanel::goalResponseCb, this, _1);
        options.feedback_callback       = std::bind(&MappingPanel::feedbackCb, this, _1, _2);
        options.result_callback         = std::bind(&MappingPanel::resultCb, this, _1);

        setCalibStatus("Sending goal...", "000000");
        calibClient->async_send_goal(calibGoal, options);

        ui->calibButton->setText("Cancel");
    }


    void MappingPanel::setCalibStatus(const QString& text, const QString& color)
    {
        ui->calibStatus->setText(text);
        ui->calibStatus->setStyleSheet(tr("QLabel { color: #%1; }").arg(color));
    }


    void MappingPanel::goalResponseCb(const CalibGoalHandle::SharedPtr& goalHandle)
    {
        if(goalHandle)
        {
            switch(goalHandle->get_status())
            {
                case GOAL_STATE_ACCEPTED:
                    setCalibStatus("Calibrating map frame...", "000000");
                    break;
                case GOAL_STATE_CANCELING:
                    setCalibStatus("Canceling calibration...", "000000");
                    break;
                default:
                    setCalibStatus("Unknown goal state", "000000");
                    break;
            }
        } else
        {
            setCalibStatus("Calibration request rejected!", "FF0000");
        }   
    }


    void MappingPanel::feedbackCb(
        CalibGoalHandle::SharedPtr,
        const std::shared_ptr<const ModelFrame::Feedback> feedback)
    {
        ui->calibProgress->setValue(feedback->sample_count);
        setCalibStatus(
            tr("Calibrating map frame (%1/%2)...").arg(
                QString::number(feedback->sample_count),
                QString::number(ui->calibProgress->maximum())),
            "000000"
        );
    }


    void MappingPanel::resultCb(const CalibGoalHandle::WrappedResult & result)
    {        
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if(result.result->success)
                {
                    setCalibStatus("Calibration Complete.", "000000");
                } else 
                {
                    setCalibStatus(
                        tr("Calibration failed (%1)").arg(QString::fromStdString(result.result->err_msg)),
                        "FF0000"
                    );
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                setCalibStatus("Calibration aborted!", "FF0000");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                setCalibStatus("Calibration canceled!", "0000FF");
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                setCalibStatus("Calibration unknown!", "0000FF");
                break;
        }

        calibrationInProgress = false;
        ui->calibButton->setText("Calibrate");
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::MappingPanel, rviz_common::Panel);
