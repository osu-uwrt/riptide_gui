#include "riptide_rviz/ControlPanel.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <QMessageBox>

#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

#include <unistd.h>

#define MAX_HOST_LEN 300
#define SETPOINT_REPUB_PERIOD 1s
#define SETPOINT_MARKER_SCALE 1.5

const std::string get_hostname()
{
    // retrieve the system hostname in hopefully MAX_HOST_LEN characters -1 for null term
    char hostCstr[MAX_HOST_LEN];
    gethostname(hostCstr, MAX_HOST_LEN);

    std::string hostnameInternal(hostCstr);

    // make sure we have a null termination
    if (hostnameInternal.length() >= MAX_HOST_LEN)
    {
        hostnameInternal = "unknown_host";
        std::cerr << "Failed to discover system hostname, falling back to default, " << hostnameInternal;
    }
    else
    {
        // replace the dashes with underscores, because the spec doesnt like dashes
        std::replace(hostnameInternal.begin(), hostnameInternal.end(), '-', '_');
    }

    // kinda important.... without this strings raise a bad_alloc
    return hostnameInternal;
}


namespace riptide_rviz
{
    ControlPanel::ControlPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_ControlPanel();
        uiPanel->setupUi(this);

        // create the default message
        ctrlMode = riptide_rviz::ControlPanel::control_modes::DISABLED;

        RVIZ_COMMON_LOG_INFO("ControlPanel: Constructed control panel");
    }

    void ControlPanel::onInitialize()
    {
        // Connect UI signals for controlling the riptide vehicle
        connect(uiPanel->ctrlEnable, &QPushButton::clicked, this, &ControlPanel::handleEnable);
        connect(uiPanel->ctrlDisable, &QPushButton::clicked, this, &ControlPanel::handleDisable);
        connect(uiPanel->ctrlDegOrRad, &QPushButton::clicked, this, &ControlPanel::toggleDegrees);

        // mode seting buttons
        connect(uiPanel->ctrlModePos, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_rviz::ControlPanel::control_modes::POSITION); });
        connect(uiPanel->ctrlModeVel, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_rviz::ControlPanel::control_modes::VELOCITY); });
        connect(uiPanel->ctrlModeFFD, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_rviz::ControlPanel::control_modes::FEEDFORWARD); });
        connect(uiPanel->ctrlModeTele, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_rviz::ControlPanel::control_modes::TELEOP); });

        // command sending buttons
        connect(uiPanel->ctrlDiveInPlace, &QPushButton::clicked, this, &ControlPanel::handleLocalDive);
        connect(uiPanel->ctrlFwdCurrent, &QPushButton::clicked, this, &ControlPanel::handleCurrent);
        connect(uiPanel->CtrlSendCmd, &QPushButton::clicked, this, 
            [this] () { ControlPanel::handleCommand(true); } );

        //parameter reload buttons
        connect(uiPanel->reloadController, &QPushButton::clicked, this, &ControlPanel::handleReloadController);

        //drag cal buttons
        connect(uiPanel->dragStart, &QPushButton::clicked, this, &ControlPanel::handleStartDragCal);
        connect(uiPanel->dragStop, &QPushButton::clicked, this, &ControlPanel::handleStopDragCal);
        connect(uiPanel->dragTrigger, &QPushButton::clicked, this, &ControlPanel::handleTriggerDragCal);

        RVIZ_COMMON_LOG_INFO("ControlPanel: Initialized panel");
    }

    void ControlPanel::load(const rviz_common::Config &config)
    {
        // load the parent class config
        rviz_common::Panel::load(config);

        // create our value containers
        QString * str = new QString();
        float * configVal = new float();

        // load the namesapce param
        if(config.mapGetString("robot_namespace", str)){
            robot_ns = str->toStdString();
        } else {
            // default value
            robot_ns = "/talos";
            RVIZ_COMMON_LOG_WARNING("ControlPanel: Loading default value for 'namespace'");
        }

        if(config.mapGetFloat("odom_timeout", configVal)){
            odom_timeout = std::chrono::duration<double>(*configVal);
        } else {
            // default value
            odom_timeout = std::chrono::duration<double>(2.5);
            RVIZ_COMMON_LOG_WARNING("ControlPanel: Loading default value for 'odom_timeout' in ");
        }

        if(config.mapGetFloat("tgt_in_place_depth", configVal)){
            tgt_in_place_depth = *configVal;
        } else {
            // default value
            tgt_in_place_depth = -0.75;
            RVIZ_COMMON_LOG_WARNING("ControlPanel: Loading default value for 'tgt_in_place_depth'");
        }

        if(config.mapGetFloat("max_depth_in_place", configVal)){
            max_depth_in_place = *configVal;
        } else {
            // default value
            max_depth_in_place = -0.5;
            RVIZ_COMMON_LOG_WARNING("ControlPanel: Loading default value for 'max_depth_in_place'");
        }

        // Free the allocated containers
        delete str;
        delete configVal;

        // create the timer but hold on starting it as things may not have been fully initialized yet
        uiTimer = new QTimer(this);
        connect(uiTimer, &QTimer::timeout, [this](void)
                { refreshUI(); });

        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // setup goal_pose sub
        selectPoseSub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::selectedPose, this, _1));

        // setup the ROS topics that depend on namespace
        // make publishers
        killStatePub = node->create_publisher<riptide_msgs2::msg::KillSwitchReport>(robot_ns + "/command/software_kill", rclcpp::SystemDefaultsQoS());
        
        //controller setpoint publishers
        #if CONTROLLER_OUTPUT_TYPE == CONTROLLER_CMD
            ctrlCmdLinPub = node->create_publisher<riptide_msgs2::msg::ControllerCommand>(robot_ns + "/controller/linear", rclcpp::SystemDefaultsQoS());
            ctrlCmdAngPub = node->create_publisher<riptide_msgs2::msg::ControllerCommand>(robot_ns + "/controller/angular", rclcpp::SystemDefaultsQoS());
        #elif CONTROLLER_OUTPUT_TYPE == TARGET_POSITION
            pidSetptPub = node->create_publisher<geometry_msgs::msg::Pose>(robot_ns + "/controller/target_position", rclcpp::SystemDefaultsQoS());
        #endif
        
        dragCalTriggerPub = node->create_publisher<std_msgs::msg::Empty>(robot_ns + "/trigger", rclcpp::SystemDefaultsQoS());

        // make ROS Subscribers
        odomSub = node->create_subscription<nav_msgs::msg::Odometry>(
            robot_ns + "/odometry/filtered", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::odomCallback, this, _1));
        diagSub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::diagCallback, this, _1));        

        //create service clients
        reloadSolverClient = node->create_client<Trigger>(robot_ns + "/controller_overseer/update_thruster_solver_params");
        reloadSmcClient = node->create_client<Trigger>(robot_ns + "/controller_overseer/update_smc_params");
        reloadPidClient = node->create_client<Trigger>(robot_ns + "/controller_overseer/update_pid_params");
        reloadCompleteClient = node->create_client<Trigger>(robot_ns + "/controller_overseer/update_complete_controller_params");

        //create action clients
        calibrateDrag = rclcpp_action::create_client<CalibrateDrag>(node, robot_ns + "/calibrate_drag_new");

        //interactive marker for setpoint
        setptServer = std::make_shared<interactive_markers::InteractiveMarkerServer>("interactive_setpoint", node);

        //initialize telop server
        setTeleopClient = node->create_client<SetBool>(robot_ns + "/setTeleop");
        
        interactiveSetpointMarker.header.frame_id = "world",
        interactiveSetpointMarker.header.stamp = node->get_clock()->now();
        interactiveSetpointMarker.name = "interactive_setpt";
        interactiveSetpointMarker.description = "Interactive controller setpoint";
        interactiveSetpointMarker.scale = SETPOINT_MARKER_SCALE;

        //create controls which will move the setpoint around
        visualization_msgs::msg::InteractiveMarkerControl moveCtrlX;
        moveCtrlX.name = "moveX";
        moveCtrlX.always_visible = true;
        moveCtrlX.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        moveCtrlX.orientation.w = 1;
        interactiveSetpointMarker.controls.push_back(moveCtrlX);
        
        visualization_msgs::msg::InteractiveMarkerControl moveCtrlY;
        moveCtrlY.name = "moveCtrlY";
        moveCtrlY.always_visible = true;
        moveCtrlY.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        moveCtrlY.orientation.z = 1;
        moveCtrlY.orientation.w = 1;
        interactiveSetpointMarker.controls.push_back(moveCtrlY);

        visualization_msgs::msg::InteractiveMarkerControl moveCtrlZ;
        moveCtrlZ.name = "moveCtrlZ";
        moveCtrlZ.always_visible = true;
        moveCtrlZ.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        moveCtrlZ.orientation.y = 1;
        moveCtrlZ.orientation.w = 1;
        interactiveSetpointMarker.controls.push_back(moveCtrlZ);

        visualization_msgs::msg::InteractiveMarkerControl moveCtrlRoll;
        moveCtrlRoll.name = "moveCtrlRoll";
        moveCtrlRoll.always_visible = true;
        moveCtrlRoll.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        moveCtrlRoll.orientation.w = 1;
        interactiveSetpointMarker.controls.push_back(moveCtrlRoll);

        visualization_msgs::msg::InteractiveMarkerControl moveCtrlPitch;
        moveCtrlPitch.name = "moveCtrlPitch";
        moveCtrlPitch.always_visible = true;
        moveCtrlPitch.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        moveCtrlPitch.orientation.z = 1;
        moveCtrlPitch.orientation.w = 1;
        interactiveSetpointMarker.controls.push_back(moveCtrlPitch);

        visualization_msgs::msg::InteractiveMarkerControl moveCtrlYaw;
        moveCtrlYaw.name = "moveCtrlYaw";
        moveCtrlYaw.always_visible = true;
        moveCtrlYaw.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        moveCtrlYaw.orientation.y = 1;
        moveCtrlYaw.orientation.w = 1;
        interactiveSetpointMarker.controls.push_back(moveCtrlYaw);

        //initialize tf stuff
        tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // now we can start the UI refresh timer
        uiTimer->start(100);

        // and start the kill switch pub timer
        killPubTimer = node->create_wall_timer(50ms, std::bind(&ControlPanel::sendKillMsgTimer, this));
        
        //add timer to repub the set point every second        
        setPointPubTimer = node->create_wall_timer(SETPOINT_REPUB_PERIOD, std::bind(&ControlPanel::pubCurrentSetpoint, this));

        // save the hostanme for kill instance
        hostname = "rviz_control_" + get_hostname();

        //set a dummy for last commanded pose
        geometry_msgs::msg::Point linear;
        tf2::Quaternion quat = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

        linear.x = 0;
        linear.y = 0;
        linear.z = -1;

        this->lastCommandedPose.position = linear;
        this->lastCommandedPose.orientation = tf2::toMsg(quat);
        RVIZ_COMMON_LOG_INFO("ControlPanel: Loading config complete");
    }

    void ControlPanel::pubCurrentSetpoint(){

        #if CONTROLLER_OUTPUT_TYPE == TARGET_POSITION
            this->pidSetptPub->publish(this->lastCommandedPose);
        #else
            RVIZ_COMMON_LOG_INFO("Not Republishing Set Point: not supported control mode.");
        #endif
    }

    void ControlPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        // write our config values
        config.mapSetValue("robot_namespace", QString::fromStdString(robot_ns));
        config.mapSetValue("odom_timeout", odom_timeout.count());
        config.mapSetValue("max_depth_in_place", max_depth_in_place);
        config.mapSetValue("tgt_in_place_depth", tgt_in_place_depth);
    }

    bool ControlPanel::event(QEvent *event)
    {
        return false;
    }

    ControlPanel::~ControlPanel()
    {
        // master window control removal
        delete uiPanel;

        // remove the timers
        delete uiTimer;
    }

    // slots for handling mode setting of the controller
    void ControlPanel::handleEnable()
    {
        vehicleEnabled = true;
        uiPanel->ctrlEnable->setEnabled(false);
        uiPanel->ctrlDisable->setEnabled(true);
    }

    void ControlPanel::handleDisable()
    {
        vehicleEnabled = false;
        uiPanel->ctrlEnable->setEnabled(true);
        uiPanel->ctrlDisable->setEnabled(false);

        // clear the controller command mode
        switchMode(riptide_rviz::ControlPanel::control_modes::DISABLED, true);

        //disable controllers
        riptide_msgs2::msg::ControllerCommand disableCmd;
        disableCmd.mode = riptide_rviz::ControlPanel::control_modes::DISABLED;
        ctrlCmdLinPub->publish(disableCmd);
        ctrlCmdAngPub->publish(disableCmd);
    }

    void ControlPanel::switchMode(uint8_t mode, bool override)
    {
        // check the vehicle is enabled or we are overriding
        if(vehicleEnabled || override) {
            ctrlMode = mode;
            visualization_msgs::msg::InteractiveMarker testMarker;


            switch (ctrlMode)
            {
            case riptide_rviz::ControlPanel::control_modes::POSITION:
                RVIZ_COMMON_LOG_INFO("ControlPanel: Starting Position Control");

                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(false);
                uiPanel->ctrlModeTele->setEnabled(true);

                //also enable the interactive marker
                setptServer->insert(interactiveSetpointMarker,
                    std::bind(&ControlPanel::setptMarkerFeedback, this, _1));

                syncSetptMarkerToTextboxes(false);
                setptServer->applyChanges();

                callSetBoolService(this->setTeleopClient, false);
                break;
            case riptide_rviz::ControlPanel::control_modes::VELOCITY:
                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(false);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(true);

                callSetBoolService(this->setTeleopClient, false);
                break;
            case riptide_rviz::ControlPanel::control_modes::FEEDFORWARD:
                uiPanel->ctrlModeFFD->setEnabled(false);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(true);

                callSetBoolService(this->setTeleopClient, false);
                break;

            case riptide_rviz::ControlPanel::control_modes::TELEOP:
                //call service to enable teleop
                RVIZ_COMMON_LOG_INFO("ControlPanel: Starting Teleop");

                callSetBoolService(this->setTeleopClient, true);

                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(false);

                setptServer->erase(interactiveSetpointMarker.name);
                setptServer->applyChanges();
                break;

            case riptide_rviz::ControlPanel::control_modes::DISABLED:

                RVIZ_COMMON_LOG_INFO("ControlPanel: Disabled Control");

                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(true);

                setptServer->erase(interactiveSetpointMarker.name);
                setptServer->applyChanges();
                break;

            default:
                RVIZ_COMMON_LOG_ERROR("ControlPanel: Button not yet operable");
                break;
            }
        }
    }

    void ControlPanel::refreshUI()
    {
        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // handle timing out the UI buttons if odom gets too stale
        auto diff = node->get_clock()->now() - odomTime;
        if (diff.to_chrono<std::chrono::seconds>() > odom_timeout || !vehicleEnabled)
        {
            // the odom has timed out
            if (uiPanel->CtrlSendCmd->isEnabled()){
                if(diff.to_chrono<std::chrono::seconds>() > odom_timeout)
                    RVIZ_COMMON_LOG_WARNING("ControlPanel: Odom timed out! disabling local control buttons");

                if(!vehicleEnabled)
                    RVIZ_COMMON_LOG_WARNING("ControlPanel: vehicle disabled! disabling local control buttons");

                // disable the vehicle
                handleDisable();
            }

            // also disable command sending
            uiPanel->ctrlDiveInPlace->setEnabled(false);
            uiPanel->CtrlSendCmd->setEnabled(false);
        }
        else
        {
            uiPanel->CtrlSendCmd->setEnabled(true);

            // check the current depth. if we are below 0.5m, disable the submerge in place button
            bool convOk;
            double z = uiPanel->cmdCurrZ->text().toDouble(&convOk);
            if (convOk && z < max_depth_in_place)
            {
                uiPanel->ctrlDiveInPlace->setEnabled(false);
            }
            else
            {
                uiPanel->ctrlDiveInPlace->setEnabled(true);
            }
        }
    }

    // slots for sending commands to the vehicle
    void ControlPanel::handleLocalDive()
    {
        uiPanel->cmdReqX->setText(uiPanel->cmdCurrX->text());
        uiPanel->cmdReqY->setText(uiPanel->cmdCurrY->text());
        uiPanel->cmdReqZ->setText("-0.75");
        uiPanel->cmdReqR->setText("0");
        uiPanel->cmdReqP->setText("0");
        uiPanel->cmdReqYaw->setText(uiPanel->cmdCurrYaw->text());

        handleCommand(true);
    }

    void ControlPanel::toggleDegrees()
    {
        degreeReadout = !degreeReadout;
        if (degreeReadout)
        {
            uiPanel->ctrlDegOrRad->setText("Degrees");
        }
        else
        {
            uiPanel->ctrlDegOrRad->setText("Radians");
        }
    }

    void ControlPanel::handleCurrent()
    {
        // take the values from the readouts
        QString x, y, z, roll, pitch, yaw;
        x = uiPanel->cmdCurrX->text();
        y = uiPanel->cmdCurrY->text();
        z = uiPanel->cmdCurrZ->text();
        roll = uiPanel->cmdCurrR->text();
        pitch = uiPanel->cmdCurrP->text();
        yaw = uiPanel->cmdCurrYaw->text();

        // take the values and propagate them into the requested values
        if (uiPanel->cmdCopyCurrX->isChecked()) {
            uiPanel->cmdReqX->setText(x);
        }
        if (uiPanel->cmdCopyCurrY->isChecked()) {
            uiPanel->cmdReqY->setText(y);
        }
        if (uiPanel->cmdCopyCurrZ->isChecked()) {
            uiPanel->cmdReqZ->setText(z);
        }
        if (uiPanel->cmdCopyCurrRoll->isChecked()) {
            uiPanel->cmdReqR->setText(roll);
        }
        if (uiPanel->cmdCopyCurrPitch->isChecked()) {
            uiPanel->cmdReqP->setText(pitch);
        }
        if (uiPanel->cmdCopyCurrYaw->isChecked()) {
            uiPanel->cmdReqYaw->setText(yaw);
        }

        syncSetptMarkerToTextboxes();
    }

    void ControlPanel::handleCommand(bool updateInteractiveMarker)
    {
        // first take the current readout and hold it
        // only need xy and yaw, we discard roll and pitch and z
        double desiredValues[6];
        if (!getDesiredSetpointFromTextboxes(desiredValues))
        {
            RVIZ_COMMON_LOG_ERROR("ControlPanel: Failed to convert current position to floating point");
            // set the red stylesheet
            QString stylesheet = "QPushButton{color:black; background: red;}";
            uiPanel->CtrlSendCmd->setStyleSheet(stylesheet);
            uiPanel->ctrlDiveInPlace->setStyleSheet(stylesheet);

            // create a timer to clear it in 1 second
            QTimer::singleShot(1000, [this](void)
                               { uiPanel->CtrlSendCmd->setStyleSheet(""); 
                                 uiPanel->ctrlDiveInPlace->setStyleSheet(""); });
            return;
        }

        // now we can build the command and send it
        // build the linear control message
        geometry_msgs::msg::Point linear;
        linear.x = desiredValues[0];
        linear.y = desiredValues[1];
        linear.z = desiredValues[2];

        geometry_msgs::msg::Quaternion angularPosition;
        // geometry_msgs::msg::Vector3 angularVelocity;
        
        // if we are in position, we use quat, otherwise use the vector
        if (ctrlMode == riptide_rviz::ControlPanel::control_modes::POSITION)
        {
            // check the angle mode button
            if(degreeReadout){
                desiredValues[3] *= M_PI / 180.0;
                desiredValues[4] *= M_PI / 180.0;
                desiredValues[5] *= M_PI / 180.0;
            }

            // convert RPY to quaternion
            tf2::Quaternion quat;
            quat.setRPY(desiredValues[3], desiredValues[4], desiredValues[5]);

            // build the angular quat for message
            angularPosition = tf2::toMsg(quat);
        } else {
            // // build the vector
            // angularVelocity.x = desiredValues[3];
            // angularVelocity.y = desiredValues[4];
            // angularVelocity.z = desiredValues[5];

            QMessageBox::warning(uiPanel->CtrlSendCmd, "Control mode not supported!", "Control mode is not supported by the controller! It will be removed from the panel soon.");
        }

        //assemble pose to command
        geometry_msgs::msg::Pose untransformed_command;
        untransformed_command.position = linear;
        untransformed_command.orientation = angularPosition;

        //transform command from command frame to world
        std::string command_frame = uiPanel->ctrlFrame->currentText().toStdString();
        geometry_msgs::msg::Pose world_command;
        if(!transformBetweenFrames(untransformed_command, world_command, command_frame, "world"))
        {
            return;
        }

        #if CONTROLLER_OUTPUT_TYPE == CONTROLLER_CMD
            auto linCmd = riptide_msgs2::msg::ControllerCommand();
            linCmd.setpoint_vect.x = world_command.position.x;
            linCmd.setpoint_vect.y = world_command.position.y;
            linCmd.setpoint_vect.z = world_command.position.z;
            linCmd.mode = ctrlMode;

            auto angCmd = riptide_msgs2::msg::ControllerCommand();
            angCmd.mode = ctrlMode;
            angCmd.setpoint_quat = world_command.orientation;
            // angCmd.setpoint_vect = angularVelocity;

            // send the control messages
            ctrlCmdLinPub->publish(linCmd);
            ctrlCmdAngPub->publish(angCmd);

        #elif CONTROLLER_OUTPUT_TYPE == TARGET_POSITION
            if(ctrlMode == riptide_rviz::ControlPanel::control_modes::POSITION)
            {
                geometry_msgs::msg::Pose setpt;
                setpt.position = linear;
                setpt.orientation = angularPosition;

                this->lastCommandedPose = setpt;

                pidSetptPub->publish(setpt);
            } else
            {
                QMessageBox::warning(uiPanel->CtrlSendCmd, "Control mode not supported!", "Control mode is not supported by the PID controller!");
            }
            
        #endif

        if(updateInteractiveMarker)
        {
            syncSetptMarkerToTextboxes();
        }
    }

    void ControlPanel::odomCallback(const nav_msgs::msg::Odometry &msg)
    {
        std::string odom_frame = msg.header.frame_id;

        //transform command to command frame
        std::string command_frame = uiPanel->ctrlFrame->currentText().toStdString();
        geometry_msgs::msg::Pose frame_coordinates;
        if(!transformBetweenFrames(msg.pose.pose, frame_coordinates, msg.header.frame_id, command_frame))
        {
            return;
        }

        // parse the quaternion to RPY
        tf2::Quaternion q(frame_coordinates.orientation.x, frame_coordinates.orientation.y,
                          frame_coordinates.orientation.z, frame_coordinates.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // save the header timestamp
        odomTime = msg.header.stamp;

        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();


        auto diff = node->get_clock()->now() - odomTime;
        if (diff.to_chrono<std::chrono::seconds>() > odom_timeout * 2)
        {
            RVIZ_COMMON_LOG_WARNING("ControlPanel: Recieved odom message with old timestamp");
        }

        // convert to degrees if its what we're showing
        if (degreeReadout)
        {
            roll *= 180.0 / M_PI;
            pitch *= 180.0 / M_PI;
            yaw *= 180.0 / M_PI;
        }

        // show it to the user
        uiPanel->cmdCurrR->setText(QString::number(roll, 'f', 2));
        uiPanel->cmdCurrP->setText(QString::number(pitch, 'f', 2));
        uiPanel->cmdCurrYaw->setText(QString::number(yaw, 'f', 2));

        uiPanel->cmdCurrX->setText(QString::number(frame_coordinates.position.x, 'f', 2));
        uiPanel->cmdCurrY->setText(QString::number(frame_coordinates.position.y, 'f', 2));
        uiPanel->cmdCurrZ->setText(QString::number(frame_coordinates.position.z, 'f', 2));
    }
    

    void ControlPanel::selectedPose(const geometry_msgs::msg::PoseStamped & msg){
        // check position control mode !!!
        if (ctrlMode == riptide_rviz::ControlPanel::control_modes::POSITION){
            // use z depth from odom
            auto z = uiPanel->cmdCurrZ->text();
            // update the values set by the selected pose
            // take the values and propagate them into the requested values
            uiPanel->cmdReqZ->setText(z);
            uiPanel->cmdReqX->setText(QString::number(msg.pose.position.x, 'f', 2));
            uiPanel->cmdReqY->setText(QString::number(msg.pose.position.y, 'f', 2));

            // convert yaw only quat to rpy and populate
            tf2::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
                            msg.pose.orientation.z, msg.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // convert to degrees if its what we're showing
            if (degreeReadout)
            {
                roll *= 180.0 / M_PI;
                pitch *= 180.0 / M_PI;
                yaw *= 180.0 / M_PI;
            }

            uiPanel->cmdReqR->setText(QString::number(roll, 'f', 2));
            uiPanel->cmdReqP->setText(QString::number(pitch, 'f', 2));
            uiPanel->cmdReqYaw->setText(QString::number(yaw, 'f', 2));
        }
    }

    

    void ControlPanel::diagCallback(const diagnostic_msgs::msg::DiagnosticArray &msg){
        
        //add in color changing for boxes...
        
        
        if(sizeof(msg.status) != 0 ){

            if(msg.status[0].name.find("ekf") != std::string::npos){
                //pull status from ekf message

                for(int i = 0; i < (sizeof(msg.status[1].values) / 6); i++){

                    //odom frequency
                    if(msg.status[1].values[i].key.find("Actual frequency") != std::string::npos){
                        uiPanel->OdomDiagnostics->setText(QString::fromStdString(msg.status[1].values[i].value));
                    }
                }
            }else if(msg.status[0].name.find("Controller") != std::string::npos){

                for(int i = 0; i < (sizeof(msg.status[0].values) / 6); i++){

                    //active control frequency
                    if(msg.status[0].values[i].key.find("Active Control") != std::string::npos){
                        uiPanel->ACDiagnostics->setText(QString::fromStdString(msg.status[0].values[i].value));
                    }

                    //thruster flip rate
                    if(msg.status[0].values[i].key.find("Flips Frequency") != std::string::npos){
                        uiPanel->FlipDiagnostics->setText(QString::fromStdString(msg.status[0].values[i].value));
                    }

                    //system limit saturation
                    if(msg.status[0].values[i].key.find("System Limit") != std::string::npos){
                        uiPanel->SLDiagnostics->setText(QString::fromStdString(msg.status[0].values[i].value));
                    }

                    //individual limit saturation
                    if(msg.status[0].values[i].key.find("Individual Limit") != std::string::npos){
                        uiPanel->ILDiagnostics->setText(QString::fromStdString(msg.status[0].values[i].value));
                    }
            
                }         
            }         
        }
    }
    // ROS timer callbacks
    void ControlPanel::sendKillMsgTimer()
    {
        auto killMsg = riptide_msgs2::msg::KillSwitchReport();
        killMsg.kill_switch_id = riptide_msgs2::msg::KillSwitchReport::KILL_SWITCH_RQT_CONTROLLER;
        killMsg.sender_id = hostname;
        killMsg.switch_asserting_kill = !vehicleEnabled;
        killMsg.switch_needs_update = uiPanel->ctrlRequireKill->isChecked();

        killStatePub->publish(killMsg);
    }


    void ControlPanel::handleReloadController()
    {
        std::string model_type = uiPanel->reloadControllerSelect->currentText().toStdString();
        updateCalStatus("Attempting to invoke " + model_type + " reload service");
        switch(uiPanel->reloadControllerSelect->currentIndex())
        {
            case 0:
                callTriggerService(reloadCompleteClient);
                break;
            case 1:
                callTriggerService(reloadSmcClient);
                break;
            case 2:
                callTriggerService(reloadPidClient);
                break;
            case 3:
                callTriggerService(reloadSolverClient);
                break;
        }
        
    }


    void ControlPanel::handleStartDragCal()
    {
        updateCalStatus("Attempting to start drag calibration");

        if(!calibrateDrag->wait_for_action_server(1s))
        {
            updateCalStatus("Drag calibration action server unavailable!");
            setDragCalRunning(false);
            return;
        }

        CalibrateDrag::Goal goal;
        goal.start_axis = uiPanel->dragCalStartAxis->currentIndex();

        rclcpp_action::Client<CalibrateDrag>::SendGoalOptions sendgoalOptions;
        sendgoalOptions.goal_response_callback = std::bind(&ControlPanel::dragGoalResponseCb, this, _1);
        sendgoalOptions.result_callback = std::bind(&ControlPanel::dragResultCb, this, _1);

        calibrateDrag->async_send_goal(goal, sendgoalOptions);
        setDragCalRunning(true);
    }


    void ControlPanel::handleStopDragCal()
    {
        calibrateDrag->async_cancel_all_goals();
    }


    void ControlPanel::handleTriggerDragCal()
    {
        dragCalTriggerPub->publish(std_msgs::msg::Empty());
    }


    bool ControlPanel::transformBetweenFrames(
        geometry_msgs::msg::Pose pose_in, 
        geometry_msgs::msg::Pose& pose_out, 
        const std::string& from_frame, 
        const std::string& to_frame)
    {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer->lookupTransform(
                to_frame, from_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RVIZ_COMMON_LOG_INFO(
                "Could not transform " + from_frame + " to " + to_frame + ": " + ex.what());
            return false;
        }

        tf2::doTransform(pose_in, pose_out, transform);
        return true;
    }

    /**
     * Populates the results array with the values of the setpoint number boxes.
     * Ordered xyzrpw 
     */
    bool ControlPanel::getDesiredSetpointFromTextboxes(double results[6])
    {
        bool convOk[6];

        // make sure that the conversion goes okay as well
        results[0] = uiPanel->cmdReqX->text().toDouble(&convOk[0]);
        results[1] = uiPanel->cmdReqY->text().toDouble(&convOk[1]);
        results[2] = uiPanel->cmdReqZ->text().toDouble(&convOk[2]);
        results[3] = uiPanel->cmdReqR->text().toDouble(&convOk[3]);
        results[4] = uiPanel->cmdReqP->text().toDouble(&convOk[4]);
        results[5] = uiPanel->cmdReqYaw->text().toDouble(&convOk[5]);
        
        //returns false if any of the conversions failed
        return !(std::any_of(std::begin(convOk), std::end(convOk), [](bool i)
                        { return !i; }));
    }


    void ControlPanel::syncSetptMarkerToTextboxes(bool applyChanges)
    {
        double desired[6];

        if(getDesiredSetpointFromTextboxes(desired))
        {
            if(degreeReadout)
            {
                desired[3] *= M_PI / 180.0;
                desired[4] *= M_PI / 180.0;
                desired[5] *= M_PI / 180.0;
            }

            //convert the rpy to quat
            tf2::Quaternion quat;
            quat.setRPY(desired[3], desired[4], desired[5]);

            //make the pose 
            geometry_msgs::msg::Pose newMarkerPose;
            newMarkerPose.position.x = desired[0];
            newMarkerPose.position.y = desired[1];
            newMarkerPose.position.z = desired[2];
            newMarkerPose.orientation = tf2::toMsg(quat);

            //transform new marker pose to world (currently in the selected frame)
            std::string command_frame = uiPanel->ctrlFrame->currentText().toStdString();
            geometry_msgs::msg::Pose newMarkerPoseWorld;
            if(!transformBetweenFrames(newMarkerPose, newMarkerPoseWorld, command_frame, "world"))
            {
                return;
            }

            //set the pose
            setptServer->setPose(interactiveSetpointMarker.name, newMarkerPoseWorld);

            if(applyChanges)
            {
                RVIZ_COMMON_LOG_INFO("Applying marker sync");
                setptServer->applyChanges();
            }
        } else
        {
            RVIZ_COMMON_LOG_ERROR("Could not convert the content of the boxes to doubles");
        }
    }


    /**
     * @brief invoked when the interactive setpoint marker is moved
     * @param feedback feedback containing the new pose
     */
    void ControlPanel::setptMarkerFeedback(interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr feedback)
    {
        uiPanel->cmdReqX->setText(QString::number(feedback->pose.position.x, 'f', 2));
        uiPanel->cmdReqY->setText(QString::number(feedback->pose.position.y, 'f', 2));
        uiPanel->cmdReqZ->setText(QString::number(feedback->pose.position.z, 'f', 2));

        //convert the quaternion to RPY
        tf2::Quaternion q (
            feedback->pose.orientation.x, 
            feedback->pose.orientation.y,
            feedback->pose.orientation.z,
            feedback->pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        if (degreeReadout)
        {
            roll *= 180.0 / M_PI;
            pitch *= 180.0 / M_PI;
            yaw *= 180.0 / M_PI;
        }

        uiPanel->cmdReqR->setText(QString::number(roll, 'f', 2));
        uiPanel->cmdReqP->setText(QString::number(pitch, 'f', 2));
        uiPanel->cmdReqYaw->setText(QString::number(yaw, 'f', 2));

        handleCommand(false);
    }


    void ControlPanel::updateCalStatus(const std::string& status)
    {
        RVIZ_COMMON_LOG_INFO(status);
        uiPanel->calStatus->setText(QString::fromStdString(status));
    }


    void ControlPanel::callTriggerService(rclcpp::Client<Trigger>::SharedPtr client)
    {
        if(!client->wait_for_service(5s))
        {
            updateCalStatus("Service unavailable!");
            return;
        }

        Trigger::Request::SharedPtr request = std::make_shared<Trigger::Request>();
        auto future = client->async_send_request(request);
        srvReqId = future.request_id;
        activeClientFuture = future.share();
        QTimer::singleShot(250, 
            [this, client] () { ControlPanel::waitForTriggerResponse(client); });
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        clientSendTime = node->get_clock()->now();
    }

    void ControlPanel::callSetBoolService(rclcpp::Client<SetBool>::SharedPtr client, bool value){
        //check to ensure service is availabl
        if(!client->wait_for_service(5s)){
            return;
        }

        SetBool::Request::SharedPtr request = std::make_shared<SetBool::Request>();

        //fill out request
        request->data = value;

        //send request
        auto future = client->async_send_request(request);
        srvReqId = future.request_id;
        activeSetBoolClientFuture = future.share();
        QTimer::singleShot(250, 
            [this, client] () { ControlPanel::waitForSetBoolResponse(client); });
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        clientSendTime = node->get_clock()->now();
    }


    void ControlPanel::waitForTriggerResponse(rclcpp::Client<Trigger>::SharedPtr client)
    {
        if(!activeClientFuture.valid())
        {
            //updateCalStatus("Service result has become invalid!");
            return;
        }

        auto futureStatus = activeClientFuture.wait_for(10ms);
        if(futureStatus != std::future_status::timeout)
        {
            //success
            rclcpp::Client<Trigger>::SharedResponse response = activeClientFuture.get();
            updateCalStatus(response->message);
            return;
        }

        //not ready yet
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        rclcpp::Time currentTime = node->get_clock()->now();
        if(currentTime - clientSendTime > 5s)
        {
            //updateCalStatus("Service timed out.");
            client->remove_pending_request(srvReqId);
            return;
        }

        //schedule next check
        QTimer::singleShot(250, 
            [this, client] () { ControlPanel::waitForTriggerResponse(client); });
    }

    void ControlPanel::waitForSetBoolResponse(rclcpp::Client<SetBool>::SharedPtr client){
        if(!activeSetBoolClientFuture.valid())
        {
            updateCalStatus("Service result has become invalid!");
            return;
        }

        auto futureStatus = activeSetBoolClientFuture.wait_for(10ms);
        if(futureStatus != std::future_status::timeout)
        {
            //success
            rclcpp::Client<SetBool>::SharedResponse response = activeSetBoolClientFuture.get();
            updateCalStatus(response->message);
            return;
        }

        //not ready yet
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        rclcpp::Time currentTime = node->get_clock()->now();
        if(currentTime - clientSendTime > 5s)
        {
            updateCalStatus("Service timed out.");
            client->remove_pending_request(srvReqId);
            return;
        }

        //schedule next check
        QTimer::singleShot(250, 
            [this, client] () { ControlPanel::waitForSetBoolResponse(client); });
    }


    void ControlPanel::setDragCalRunning(bool running)
    {
        uiPanel->dragStart->setEnabled(!running);
        uiPanel->dragStop->setEnabled(running);
    }


    void ControlPanel::dragGoalResponseCb(const CalibrateDragGH::SharedPtr &goal_handle)
    {
        if (!goal_handle) {
            updateCalStatus("Drag calibration rejected by server!");
            setDragCalRunning(false);
        } else {
            updateCalStatus("Drag calibration in progress");
            setDragCalRunning(true);
        }
    }


    void ControlPanel::dragResultCb(const CalibrateDragGH::WrappedResult &result)
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::ABORTED:
                updateCalStatus("Drag calibration aborted!");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                updateCalStatus("Drag calibration canceled.");
                break;
            case rclcpp_action::ResultCode::SUCCEEDED:
                updateCalStatus("Drag calibration succeeded.");
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                updateCalStatus("Drag calibration status unknown");
                break;
        }

        setDragCalRunning(false);
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ControlPanel, rviz_common::Panel);
