#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>

#include <QTimer>

#include "riptide_rviz/SimulatorPanel.hpp"

namespace riptide_rviz {

    struct BtnData {
        bool enabled;
        const char * text;
        const char * style;
    };
    
    static const BtnData BTN_START_STOP_DATA[NUM_BTN_STATES] = {
        [BTN_DISPLAY_START] = {true, "Start", "QPushButton{color:white; background: green;}"},
        [BTN_DISPLAY_WAIT] = {false, "Wait", "QPushButton{color:white; background: grey;}"},
        [BTN_DISPLAY_STOP] = {true, "Stop", "QPushButton{color:black; background: red;}"}
    };

    static const BtnData BTN_SYNC_DATA[NUM_SYNC_STATES] = {
        [BTN_SYNC_IDLE] = {true, "Sync", ""},
        [BTN_SYNC_BUSY] = {false, "Syncing. . .", ""},
    };

    void SimulatorPanel::setBtnStartStopState(BtnStartStopState state) {
        uiPanel->btnStartStop->setStyleSheet(BTN_START_STOP_DATA[state].style);
        uiPanel->btnStartStop->setText(BTN_START_STOP_DATA[state].text);
        uiPanel->btnStartStop->setEnabled(BTN_START_STOP_DATA[state].enabled);
    }

    void SimulatorPanel::setBtnSyncState(BtnSyncState state) {
        uiPanel->btnSync->setStyleSheet(BTN_SYNC_DATA[state].style);
        uiPanel->btnSync->setText(BTN_SYNC_DATA[state].text);
        uiPanel->btnSync->setEnabled(BTN_SYNC_DATA[state].enabled);
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
        startStopClient = node->create_client<riptide_msgs2::srv::ToggleSimulator>(robotNs + "/toggle_sim");
        syncClient = node->create_client<robot_localization::srv::SetPose>(robotNs + "/set_sim_pose");
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
        connect(uiPanel->btnSync, &QPushButton::clicked, this, &SimulatorPanel::handleSyncClick);
    }

    void SimulatorPanel::handleSyncClick(void) {
        // first take the current readout and hold it
        // only need xy and yaw, we discard roll and pitch and z
        double x, y, z, roll, pitch, yaw;
        bool convOk[6];

        // make sure that the conversion goes okay as well
        x = uiPanel->lineEditX->text().toDouble(&convOk[0]);
        y = uiPanel->lineEditY->text().toDouble(&convOk[1]);
        z = uiPanel->lineEditZ->text().toDouble(&convOk[2]);
        roll = uiPanel->lineEditRoll->text().toDouble(&convOk[3]);
        pitch = uiPanel->lineEditPitch->text().toDouble(&convOk[4]);
        yaw = uiPanel->lineEditYaw->text().toDouble(&convOk[5]);

        if (std::any_of(std::begin(convOk), std::end(convOk), [](bool i)
                        { return !i; })) {
            RVIZ_COMMON_LOG_ERROR("ControlPanel: Failed to convert current position to floating point");
            // set the red stylesheet
            QString stylesheet = "QPushButton{color:black; background: red;}";
            uiPanel->btnSync->setStyleSheet(stylesheet);

            // create a timer to clear it in 1 second
            QTimer::singleShot(1000, [this](void)
                               { uiPanel->btnSync->setStyleSheet("");}  );
            return;
        }

        setBtnSyncState(BTN_SYNC_BUSY);
        auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        request->pose.pose.pose.position.x = x;
        request->pose.pose.pose.position.y = y;
        request->pose.pose.pose.position.z = z;
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        request->pose.pose.pose.orientation = tf2::toMsg(quat);
        auto future = syncClient->async_send_request(request).share();

        QTimer::singleShot(250, [&, future] () {
            future.wait();

            setBtnSyncState(BTN_SYNC_IDLE);
        });
    }

    void SimulatorPanel::handleStartStopClick(void) {
        setBtnStartStopState(BTN_DISPLAY_WAIT);

        auto request = std::make_shared<riptide_msgs2::srv::ToggleSimulator::Request>();
        auto future = startStopClient->async_send_request(request).share();

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