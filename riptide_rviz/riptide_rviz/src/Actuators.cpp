#include "riptide_rviz/Actuators.hpp"
#include <chrono>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>


using namespace std::chrono_literals;
using namespace std::placeholders;


namespace riptide_rviz
{
    Actuators::Actuators(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_Actuators();
        uiPanel->setupUi(this);
    }


    void Actuators::onInitialize()
    {
        //connect buttons to things
        connect(uiPanel->ctrlArm, &QPushButton::clicked, this, &Actuators::handleArm);
        connect(uiPanel->ctrlDisarm, &QPushButton::clicked, this, &Actuators::handleDisarm);
        connect(uiPanel->ctrlOpenClaw, &QPushButton::clicked, this, &Actuators::handleOpenClaw);
        connect(uiPanel->ctrlCloseClaw, &QPushButton::clicked, this, &Actuators::handleCloseClaw);
        connect(uiPanel->ctrlFireTorpedo, &QPushButton::clicked, this, &Actuators::handleFireTorpedo);
        connect(uiPanel->ctrlDropMarker, &QPushButton::clicked, this, &Actuators::handleDropMarker);
        connect(uiPanel->ctrlClawGoHome, &QPushButton::clicked, this, &Actuators::handleClawGoHome);
        connect(uiPanel->ctrlClawSetHome, &QPushButton::clicked, this, &Actuators::handleClawSetHome);
        connect(uiPanel->ctrlTorpGoHome, &QPushButton::clicked, this, &Actuators::handleTorpGoHome);
        connect(uiPanel->ctrlTorpSetHome, &QPushButton::clicked, this, &Actuators::handleTorpSetHome);
    }


    void Actuators::handleArm()
    {
        RVIZ_COMMON_LOG_INFO("Arm");
    }


    void Actuators::handleDisarm()
    {
        RVIZ_COMMON_LOG_INFO("Disarm");
    }


    void Actuators::handleOpenClaw()
    {
        RVIZ_COMMON_LOG_INFO("Open claw");
    }


    void Actuators::handleCloseClaw()
    {
        RVIZ_COMMON_LOG_INFO("Close claw");
    }


    void Actuators::handleFireTorpedo()
    {
        RVIZ_COMMON_LOG_INFO("Fire Torpedo");
    }


    void Actuators::handleDropMarker()
    {
        RVIZ_COMMON_LOG_INFO("Drop Marker");
    }


    void Actuators::handleClawGoHome()
    {
        RVIZ_COMMON_LOG_INFO("claw go home");
    }


    void Actuators::handleClawSetHome()
    {
        RVIZ_COMMON_LOG_INFO("claw set home");
    }


    void Actuators::handleTorpGoHome()
    {
        RVIZ_COMMON_LOG_INFO("torp go home");
    }


    void Actuators::handleTorpSetHome()
    {
        RVIZ_COMMON_LOG_INFO("torp set home");
    }


    void Actuators::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);

        // create our value containers
        QString * str = new QString();

        // load the namesapce param
        if(config.mapGetString("robot_namespace", str)){
            robot_ns = str->toStdString();
        } else {
            // default value
            robot_ns = "/talos";
            RVIZ_COMMON_LOG_WARNING("ActuatorPanel: Loading default value for 'namespace'");
        }

        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        delete str;
    }


    void Actuators::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        // write our config values
        config.mapSetValue("robot_namespace", QString::fromStdString(robot_ns));
    }


    Actuators::~Actuators()
    {
        // master window control removal
        delete uiPanel;
    }
} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::Actuators, rviz_common::Panel);
