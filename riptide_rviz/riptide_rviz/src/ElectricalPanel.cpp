#include "riptide_rviz/ElectricalPanel.hpp"
#include "riptide_rviz/MappingPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>

namespace riptide_rviz
{
    ElectricalPanel::ElectricalPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_ElectricalPanel();
        ui->setupUi(this);
    }


    ElectricalPanel::~ElectricalPanel()
    {
        delete ui;
    }


    void ElectricalPanel::load(const rviz_common::Config &config) 
    {
        if(!config.mapGetString("robot_namespace", &robotNs))
        {
            robotNs = QString::fromStdString("/talos"); 
            RVIZ_COMMON_LOG_WARNING("ElectricalPanel: Using /talos as the default value for robot_namespace"); 
        }

        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        std::string topicName = robotNs.toStdString() + "/command/electrical";
        pub = node->create_publisher<riptide_msgs2::msg::ElectricalCommand>(topicName, 10);
    }


    void ElectricalPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }


    void ElectricalPanel::onInitialize()
    {
        connect(ui->pushButton, &QPushButton::clicked, this, &ElectricalPanel::sendCommand);
    }


    void ElectricalPanel::sendCommand()
    {
        riptide_msgs2::msg::ElectricalCommand msg;
        msg.command = ui->comboBox->currentIndex();
        pub->publish(msg);
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ElectricalPanel, rviz_common::Panel);
