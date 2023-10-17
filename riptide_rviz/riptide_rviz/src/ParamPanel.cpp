#include "riptide_rviz/ParamPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <math.h>
#include <QDoubleSpinBox>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace riptide_rviz
{
    ParamPanel::ParamPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_ParamPanel();
        ui->setupUi(this);
    }


    ParamPanel::~ParamPanel()
    {
        delete ui;
    }


    void ParamPanel::load(const rviz_common::Config &config) 
    {
        config.mapGetString("robot_namespace", &robotNs);
        if(robotNs == "")
        {
            robotNs = QString::fromStdString("/talos"); 
            RVIZ_COMMON_LOG_WARNING("FeedforwardPanel: Using /talos as the default value for robot_namespace"); 
        }

        //create ROS peripherals
        //we want a publisher to publish to controller/ff_body_force to command the feedforward of the solver
        //we also want a timer to periodcally publish that message using the publisher
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();


        RVIZ_COMMON_LOG_INFO("ParamPanel Panel loaded. Robot NS: \"" + robotNs.toStdString() + "\"");
        loaded = true;
    }


    void ParamPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }

    //called when panel is initialized. perform any needed UI connections here
    void ParamPanel::onInitialize()
    {
        counter = 0;
        connect(ui->pushButton, &QPushButton::clicked, this, &ParamPanel::buttonPressed);
    }


    void ParamPanel::buttonPressed()
    {
        counter++;
        ui->label->setText(tr("Button pressed %1 times").arg(counter));
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ParamPanel, rviz_common::Panel);
