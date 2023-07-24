#include "riptide_rviz/MappingPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>

using ModelFrame = chameleon_tf_msgs::action::ModelFrame;
using namespace std::placeholders;
using namespace std::chrono_literals;

const static std::string CALIB_ACTION_NAME = "map/model_tf";

namespace riptide_rviz
{
    std::string getFromConfig(const rviz_common::Config &config, const std::string& key, const std::string& defaultValue)
    {
        QString str;
        if(!config.mapGetString(QString::fromStdString(key), &str))
        { 
            str = QString::fromStdString(defaultValue); 
            RVIZ_COMMON_LOG_WARNING("MappingPanel: Loading default value for 'namespace'"); 
        }

        return str.toStdString();
    }

    MappingPanel::MappingPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_MappingPanel();
        ui->setupUi(this);

        ui->calibStatus->setText("");
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
        ui->mapFrame->setPlainText(QString::fromStdString(getFromConfig(config, "mapFrameName", "map")));
        ui->tagFrame->setPlainText(QString::fromStdString(getFromConfig(config, "tagFrameName", "tag_36h11")));
        ui->numSamples->setValue(std::stoi(getFromConfig(config, "mapCalibNumSamples", "10")));

        std::string fullActionName = robotNs + "/" + CALIB_ACTION_NAME;
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        calibClient = rclcpp_action::create_client<ModelFrame>(node, fullActionName);

    }

    void MappingPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        // write our config values
        config.mapSetValue("robot_namespace", QString::fromStdString(robotNs));
        config.mapSetValue("mapFrameName", ui->mapFrame->toPlainText());
        config.mapSetValue("tagFrameName", ui->tagFrame->toPlainText());
        config.mapSetValue("mapCalibNumSamples", ui->numSamples->value());
    }


    void MappingPanel::onInitialize()
    {
        // Connect UI signals for controlling the riptide vehicle
        connect(ui->calibButton, &QPushButton::clicked, this, &MappingPanel::calibMapFrame);
    }


    void MappingPanel::calibMapFrame()
    {
        if(!calibClient->wait_for_action_server(1s))
        {
            RVIZ_COMMON_LOG_ERROR("MappingPanel: Map calibration action server not available!");
            return;
        }

        ModelFrame::Goal::SharedPtr calibGoal;
        calibGoal->monitor_parent = "map";
        calibGoal->monitor_child = "tag_36h11";
        calibGoal->samples = 10;
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::MappingPanel, rviz_common::Panel);
