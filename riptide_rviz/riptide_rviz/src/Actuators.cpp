#include "riptide_rviz/Actuators.hpp"
#include <chrono>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace riptide_rviz
{
    static const char *CLAW_STATUSES[] = {
        "Error",
        "Disarmed",
        "Unknown",
        "Opened",
        "Closed",
        "Opening",
        "Closing",
        "Invalid"
    };

    static const char *TORPEDO_STATUSES[] = {
        "Error",
        "Disarmed",
        "Charging",
        "Charged",
        "Firing",
        "Fired",
        "Invalid"
    };

    static const char *DROPPER_STATUSES[] = {
        "Error",
        "Disarmed",
        "Ready",
        "Dropping",
        "Dropped",
        "Invalid"
    };

    static const char
        *STATUS_ERROR_STYLESHEET = "QLabel {color: red;} QLabel:!enabled{color:gray;}",
        *STATUS_GOOD_STYLESHEET = "QLabel {color: green;} QLabel:!enabled{color:gray;}";


    Actuators::Actuators(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_Actuators();
        uiPanel->setupUi(this);
    }


    Actuators::~Actuators()
    {
        // master window control removal
        delete uiPanel;
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
        connect(uiPanel->ctrlReload, &QPushButton::clicked, this, &Actuators::handleReload);
        connect(uiPanel->ctrlClawGoHome, &QPushButton::clicked, this, &Actuators::handleClawGoHome);
        connect(uiPanel->ctrlClawSetHome, &QPushButton::clicked, this, &Actuators::handleClawSetHome);
        connect(uiPanel->ctrlTorpGoHome, &QPushButton::clicked, this, &Actuators::handleTorpGoHome);
        connect(uiPanel->ctrlTorpSetHome, &QPushButton::clicked, this, &Actuators::handleTorpSetHome);
    }


    void Actuators::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);

        // create our value containers
        QString str;

        // load the namesapce param
        if(config.mapGetString("robot_namespace", &str)){
            robotNs = str.toStdString();
        } else {
            // default value
            robotNs = "/talos";
            RVIZ_COMMON_LOG_WARNING("ActuatorPanel: Loading default value for 'namespace'");
        }

        RVIZ_COMMON_LOG_INFO("ActuatorPanel: Using namespace " + robotNs);

        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        //initialize status sub
        statusSub = node->create_subscription<ActuatorStatus>(robotNs + "/state/actuator/status", rclcpp::SensorDataQoS(),
            std::bind(&Actuators::statusCallback, this, _1));

        //initialize clients
        dropperClient           = std::make_shared<GuiSrvClient<Trigger>>(node, robotNs + "/command/actuator/dropper", 
                                    std::bind(&Actuators::updateStatus, this, _1, _2), std::bind(&Actuators::serviceResponseCb<Trigger>, this, _1, _2));
        torpedoClient           = std::make_shared<GuiSrvClient<Trigger>>(node, robotNs + "/command/actuator/torpedo", 
                                    std::bind(&Actuators::updateStatus, this, _1, _2), std::bind(&Actuators::serviceResponseCb<Trigger>, this, _1, _2));
        reloadClient            = std::make_shared<GuiSrvClient<Trigger>>(node, robotNs + "/command/actuator/notify_reload", 
                                    std::bind(&Actuators::updateStatus, this, _1, _2), std::bind(&Actuators::serviceResponseCb<Trigger>, this, _1, _2));
        torpMarkerGoHomeClient  = std::make_shared<GuiSrvClient<Trigger>>(node, robotNs + "/command/actuator/torpedo_marker/go_home", 
                                    std::bind(&Actuators::updateStatus, this, _1, _2), std::bind(&Actuators::serviceResponseCb<Trigger>, this, _1, _2));
        torpMarkerSetHomeClient = std::make_shared<GuiSrvClient<Trigger>>(node, robotNs + "/command/actuator/torpedo_marker/set_home", 
                                    std::bind(&Actuators::updateStatus, this, _1, _2), std::bind(&Actuators::serviceResponseCb<Trigger>, this, _1, _2));
        armClient               = std::make_shared<GuiSrvClient<SetBool>>(node, robotNs + "/command/actuator/arm", 
                                    std::bind(&Actuators::updateStatus, this, _1, _2), std::bind(&Actuators::serviceResponseCb<SetBool>, this, _1, _2));
        clawClient              = std::make_shared<GuiSrvClient<SetBool>>(node, robotNs + "/command/actuator/claw", 
                                    std::bind(&Actuators::updateStatus, this, _1, _2), std::bind(&Actuators::serviceResponseCb<SetBool>, this, _1, _2));
    }


    void Actuators::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        // write our config values
        config.mapSetValue("robot_namespace", QString::fromStdString(robotNs));
    }


    void Actuators::handleArm()
    {
        callSetBoolService(armClient, true);
    }


    void Actuators::handleDisarm()
    {
        callSetBoolService(armClient, false);
    }


    void Actuators::handleOpenClaw()
    {
        callSetBoolService(clawClient, true);
    }


    void Actuators::handleCloseClaw()
    {
        callSetBoolService(clawClient, false);
    }


    void Actuators::handleFireTorpedo()
    {
        callTriggerService(torpedoClient);
    }


    void Actuators::handleDropMarker()
    {
        callTriggerService(dropperClient);
    }


    void Actuators::handleReload()
    {
        callTriggerService(reloadClient);
    }


    void Actuators::handleClawGoHome()
    {
        QMessageBox::warning(uiPanel->mainWidget, "Not supported", "This action is not supported yet.");
    }


    void Actuators::handleClawSetHome()
    {
        QMessageBox::warning(uiPanel->mainWidget, "Not supported", "This action is not supported yet.");
    }


    void Actuators::handleTorpGoHome()
    {
        callTriggerService(torpMarkerGoHomeClient);
    }


    void Actuators::handleTorpSetHome()
    {
        callTriggerService(torpMarkerSetHomeClient);
    }


    void Actuators::statusCallback(const ActuatorStatus::SharedPtr msg)
    {
        //handle arm status
        bool armed = msg->actuators_armed;

        //enable or disable controls based on act states
        // Actuators prohibit reload while moving
        uiPanel->ctrlReload->setEnabled(msg->claw_state != ActuatorStatus::CLAW_OPENING &&
                                        msg->claw_state != ActuatorStatus::CLAW_CLOSING &&
                                        msg->torpedo_state != ActuatorStatus::TORPEDO_FIRING &&
                                        msg->dropper_state != ActuatorStatus::DROPPER_DROPPING);
        uiPanel->ctrlArm->setEnabled(!armed);
        uiPanel->ctrlDisarm->setEnabled(armed);
        uiPanel->ctrlOpenClaw->setEnabled(msg->claw_state == ActuatorStatus::CLAW_CLOSED || msg->claw_state == ActuatorStatus::CLAW_UNKNOWN);
        uiPanel->ctrlCloseClaw->setEnabled(msg->claw_state == ActuatorStatus::CLAW_OPENED || msg->claw_state == ActuatorStatus::CLAW_UNKNOWN);
        uiPanel->ctrlFireTorpedo->setEnabled(msg->torpedo_state == ActuatorStatus::TORPEDO_CHARGED);
        uiPanel->ctrlDropMarker->setEnabled(msg->dropper_state == ActuatorStatus::DROPPER_READY);
        uiPanel->ctrlClawGoHome->setEnabled(armed);    // Can only go home when armed
        uiPanel->ctrlClawSetHome->setEnabled(!armed);  // Can only set home while disarmed
        uiPanel->ctrlTorpGoHome->setEnabled(armed);    // Can only go home when armed
        uiPanel->ctrlTorpSetHome->setEnabled(!armed);  // Can only set home when disarmed

        //handle claw status
        std::clamp<int>(msg->claw_state, 0, sizeof(CLAW_STATUSES)/sizeof(*CLAW_STATUSES) - 1);
        uiPanel->lblClawReady->setText(QString::fromStdString(CLAW_STATUSES[msg->claw_state]));
        uiPanel->lblClawReady->setEnabled(msg->claw_state != ActuatorStatus::CLAW_DISARMED);
        uiPanel->lblClawReady->setStyleSheet((msg->claw_state == ActuatorStatus::CLAW_ERROR ? STATUS_ERROR_STYLESHEET : STATUS_GOOD_STYLESHEET));

        //handle torpedo status and available count
        std::clamp<int>(msg->torpedo_state, 0, sizeof(TORPEDO_STATUSES)/sizeof(*TORPEDO_STATUSES) - 1);
        uiPanel->lblTorpedoReady->setText(QString::fromStdString(TORPEDO_STATUSES[msg->torpedo_state]));
        uiPanel->lblTorpedoReady->setEnabled(msg->torpedo_state != ActuatorStatus::TORPEDO_DISARMED);
        uiPanel->lblTorpedoReady->setStyleSheet((msg->torpedo_state == ActuatorStatus::TORPEDO_ERROR ? STATUS_ERROR_STYLESHEET : STATUS_GOOD_STYLESHEET));

        std::string availableString = "Available: " + std::to_string(msg->torpedo_available_count);
        uiPanel->lblTorpAvailable->setText(QString::fromStdString(availableString));

        //handle dropper status and available count
        std::clamp<int>(msg->dropper_state, 0, sizeof(DROPPER_STATUSES)/sizeof(*DROPPER_STATUSES) - 1);
        uiPanel->lblDropperReady->setText(QString::fromStdString(DROPPER_STATUSES[msg->dropper_state]));
        uiPanel->lblDropperReady->setEnabled(msg->dropper_state != ActuatorStatus::DROPPER_DISARMED);
        uiPanel->lblDropperReady->setStyleSheet((msg->dropper_state == ActuatorStatus::DROPPER_ERROR ? STATUS_ERROR_STYLESHEET : STATUS_GOOD_STYLESHEET));

        availableString = "Available: " + std::to_string(msg->dropper_available_count);
        uiPanel->lblDropperAvailable->setText(QString::fromStdString(availableString));
    }


    void Actuators::updateStatus(const QString& status, const QString& color)
    {
        RVIZ_COMMON_LOG_INFO(status.toStdString());
        uiPanel->statusBrowser->setText(status);
        uiPanel->statusBrowser->setStyleSheet(tr("QLabel { color: #%1; }").arg(color));
    }


    void Actuators::callTriggerService(GuiSrvClient<Trigger>::SharedPtr client)
    {
        Trigger::Request::SharedPtr request = std::make_shared<Trigger::Request>();
        client->callService(request);
    }


    void Actuators::callSetBoolService(GuiSrvClient<SetBool>::SharedPtr client, bool value)
    {
        SetBool::Request::SharedPtr request = std::make_shared<SetBool::Request>();
        request->data = value;
        client->callService(request);
    }
} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::Actuators, rviz_common::Panel);
