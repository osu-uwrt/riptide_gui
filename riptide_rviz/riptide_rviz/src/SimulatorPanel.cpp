#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>

#include <QTimer>

#include "riptide_rviz/SimulatorPanel.hpp"

namespace riptide_rviz {

    static const struct {
        bool enabled;
        const char * text;
        const char * style;
    } BTN_START_STOP_DATA[NUM_BTN_STATES] = {
        [BTN_DISPLAY_START] = {true, "Start", "QPushButton{color:white; background: green;}"},
        [BTN_DISPLAY_WAIT] = {false, "Wait", "QPushButton{color:white; background: grey;}"},
        [BTN_DISPLAY_STOP] = {true, "Stop", "QPushButton{color:black; background: red;}"}
    };

    void SimulatorPanel::setBtnStartStopState(BtnStartStopState state) {
        uiPanel->btnStartStop->setStyleSheet(BTN_START_STOP_DATA[state].style);
        uiPanel->btnStartStop->setText(BTN_START_STOP_DATA[state].text);
        uiPanel->btnStartStop->setEnabled(BTN_START_STOP_DATA[state].enabled);
    }

    SimulatorPanel::SimulatorPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_SimulatorPanel();
        uiPanel->setupUi(this);
        simState = SIM_STOPPED;
        setBtnStartStopState(BTN_DISPLAY_START);
    }

    SimulatorPanel::~SimulatorPanel() {
        delete uiPanel;
    }

    void SimulatorPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
        QString *str = new QString();
        
        if(config.mapGetString("robot_namespace", str)){
            robotNs = str->toStdString();
        }
        
        if (robotNs == "") {
            // default value
            robotNs = "/talos";
            RVIZ_COMMON_LOG_WARNING("ControlPanel: Loading default value for 'namespace'");
        }

        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        client = node->create_client<riptide_msgs2::srv::ToggleSimulator>(robotNs + "/toggle_sim");
    }

    void SimulatorPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs.c_str());
    }

    void SimulatorPanel::onInitialize()
    {
        RVIZ_COMMON_LOG_INFO("SimulatorPanel: Initializing");
        connect(uiPanel->btnStartStop, &QPushButton::clicked, this, &SimulatorPanel::handleStartStopClick);
    }

    void SimulatorPanel::handleStartStopClick(void) {
        setBtnStartStopState(BTN_DISPLAY_WAIT);

        auto request = std::make_shared<riptide_msgs2::srv::ToggleSimulator::Request>();
        auto future = client->async_send_request(request).share();


        std::string error = std::string("Service name: ") + client->get_service_name();
        RVIZ_COMMON_LOG_ERROR(error);

        QTimer::singleShot(250, [&, future] () {
            future.wait();

            if (simState == SIM_STOPPED) {
                setBtnStartStopState(BTN_DISPLAY_STOP);
                simState = SIM_RUNNING;
                toggleUiElements(true);
            } else if (simState == SIM_RUNNING) {
                simState = SIM_STOPPED;
                setBtnStartStopState(BTN_DISPLAY_START);
                toggleUiElements(false);
            }
        });
    }

    void SimulatorPanel::toggleUiElements(bool enabled) {
        uiPanel->lineEditX->setEnabled(enabled);
        uiPanel->lineEditY->setEnabled(enabled);
        uiPanel->lineEditZ->setEnabled(enabled);
        uiPanel->lineEditRoll->setEnabled(enabled);
        uiPanel->lineEditPitch->setEnabled(enabled);
        uiPanel->lineEditYaw->setEnabled(enabled);

        uiPanel->btnSync->setEnabled(enabled);
        uiPanel->chkBoxSensorNoise->setEnabled(enabled);
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::SimulatorPanel, rviz_common::Panel);