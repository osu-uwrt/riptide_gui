#include "riptide_rviz/ControlPanel.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>

#include <rviz_common/logging.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace riptide_rviz
{
    ControlPanel::ControlPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_ControlPanel();
        uiPanel->setupUi(this);

        // create the RCLCPP client
        auto options = rclcpp::NodeOptions().arguments({});
        clientNode = std::make_shared<rclcpp::Node>("riptide_rviz_control", options);

        // create the default message
        ctrlMode = riptide_msgs2::msg::ControllerCommand::DISABLED;

        RVIZ_COMMON_LOG_INFO("Constructed control panel");
    }

    void ControlPanel::onInitialize()
    {
        // RVIZ_COMMON_LOG_INFO("Initializing");
        // create a spin timer
        spinTimer = new QTimer(this);
        connect(spinTimer, &QTimer::timeout, [this](void)
                { rclcpp::spin_some(clientNode); });
        spinTimer->start(50);

        // Connect UI signals for controlling the riptide vehicle
        connect(uiPanel->ctrlEnable, &QPushButton::clicked, [this](void)
                { handleEnable(); });
        connect(uiPanel->ctrlDisable, &QPushButton::clicked, [this](void)
                { handleDisable(); });
        connect(uiPanel->ctrlDegOrRad, &QPushButton::clicked, [this](void)
                { toggleDegrees(); });

        // mode seting buttons
        connect(uiPanel->ctrlModePos, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::POSITION); });
        connect(uiPanel->ctrlModeVel, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::VELOCITY); });
        connect(uiPanel->ctrlModeFFD, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::FEEDFORWARD); });
        connect(uiPanel->ctrlModeVel, &QPushButton::clicked,
                [this](void)
                { switchMode(22); });

        // command sending buttons
        connect(uiPanel->ctrlDiveInPlace, &QPushButton::clicked, [this](void)
                { handleLocalDive(); });
        connect(uiPanel->ctrlFwdCurrent, &QPushButton::clicked, [this](void)
                { handleCurrent(); });
        connect(uiPanel->CtrlSendCmd, &QPushButton::clicked, [this](void)
                { handleCommand(); });

        RVIZ_COMMON_LOG_INFO("Initialized control panel");
    }

    void ControlPanel::load(const rviz_common::Config &config)
    {
        // load the parent class config
        rviz_common::Panel::load(config);

        RVIZ_COMMON_LOG_INFO("Loaded parent panel config");

        // create our value containers
        QString * str = new QString();
        float * configVal = new float();

        // load the namesapce param
        if(config.mapGetString("robot_namespace", str)){
            robot_ns = str->toStdString();
        } else {
            // default value
            robot_ns = "/talos";
            RVIZ_COMMON_LOG_WARNING("Loading default value for 'namespace'");
        }

        if(config.mapGetFloat("odom_timeout", configVal)){
            odom_timeout = std::chrono::duration<double>(*configVal);
        } else {
            // default value
            odom_timeout = std::chrono::duration<double>(2.5);
            RVIZ_COMMON_LOG_WARNING("Loading default value for 'odom_timeout'");
        }

        if(config.mapGetFloat("tgt_in_place_depth", configVal)){
            tgt_in_place_depth = *configVal;
        } else {
            // default value
            tgt_in_place_depth = -0.75;
            RVIZ_COMMON_LOG_WARNING("Loading default value for 'tgt_in_place_depth'");
        }

        if(config.mapGetFloat("max_depth_in_place", configVal)){
            max_depth_in_place = *configVal;
        } else {
            // default value
            max_depth_in_place = -0.5;
            RVIZ_COMMON_LOG_WARNING("Loading default value for 'max_depth_in_place'");
        }

        // Free the allocated containers
        delete str;
        delete configVal; 

        // create the timer but hold on starting it as things may not have been fully initialized yet
        uiTimer = new QTimer(this);
        connect(uiTimer, &QTimer::timeout, [this](void)
                { refreshUI(); });

        // setup goal_pose sub
        selectPoseSub = clientNode->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::selectedPose, this, _1)); 

        // setup the ROS topics that depend on namespace
        // make publishers
        killStatePub = clientNode->create_publisher<riptide_msgs2::msg::KillSwitchReport>(robot_ns + "/control/software_kill", rclcpp::SystemDefaultsQoS());
        ctrlCmdLinPub = clientNode->create_publisher<riptide_msgs2::msg::ControllerCommand>(robot_ns + "/controller/linear", rclcpp::SystemDefaultsQoS());
        ctrlCmdAngPub = clientNode->create_publisher<riptide_msgs2::msg::ControllerCommand>(robot_ns + "/controller/angular", rclcpp::SystemDefaultsQoS());

        // make ROS Subscribers
        odomSub = clientNode->create_subscription<nav_msgs::msg::Odometry>(
            robot_ns + "/odometry/filtered", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::odomCallback, this, _1));
        steadySub = clientNode->create_subscription<std_msgs::msg::Bool>(
            robot_ns + "/controller/steady", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::steadyCallback, this, _1));  

        // now we can start the UI refresh timer
        uiTimer->start(100);
        
        // and start the kill switch pub timer
        killPubTimer = clientNode->create_wall_timer(50ms, std::bind(&ControlPanel::sendKillMsgTimer, this));

        RVIZ_COMMON_LOG_INFO("Loading config complete");
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
        delete spinTimer;
        delete uiTimer;

        rclcpp::shutdown();
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
        switchMode(riptide_msgs2::msg::ControllerCommand::DISABLED, true);
    }

    void ControlPanel::switchMode(uint8_t mode, bool override)
    {
        // check the vehicle is enabled or we are overriding
        if(vehicleEnabled || override){
            ctrlMode = mode;
            switch (ctrlMode)
            {
            case riptide_msgs2::msg::ControllerCommand::POSITION:
                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(false);
                uiPanel->ctrlModeTele->setEnabled(true);

                break;
            case riptide_msgs2::msg::ControllerCommand::VELOCITY:
                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(false);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(true);

                break;
            case riptide_msgs2::msg::ControllerCommand::FEEDFORWARD:
                uiPanel->ctrlModeFFD->setEnabled(false);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(true);
                break;
            case riptide_msgs2::msg::ControllerCommand::DISABLED:
                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(true);
                break;
            default:
                RVIZ_COMMON_LOG_ERROR("Button not yet operable");
                break;
            }
        }
    }

    void ControlPanel::refreshUI()
    {
        // handle timing out the UI buttons if odom gets too stale
        auto diff = clientNode->get_clock()->now() - odomTime;
        if (diff.to_chrono<std::chrono::seconds>() > odom_timeout || !vehicleEnabled)
        {
            // the odom has timed out
            if (uiPanel->CtrlSendCmd->isEnabled()){
                if(diff.to_chrono<std::chrono::seconds>() > odom_timeout)
                    RVIZ_COMMON_LOG_WARNING("Odom timed out! disabling local control buttons");

                if(!vehicleEnabled)
                    RVIZ_COMMON_LOG_WARNING("vehicle disabled! disabling local control buttons");

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
        // first take the current readout and hold it
        // only need xy and yaw, we discard roll and pitch and z
        double x, y, yaw;
        bool convOk[3];

        // make sure that the conversion goes okay as well
        x = uiPanel->cmdCurrX->text().toDouble(&convOk[0]);
        y = uiPanel->cmdCurrY->text().toDouble(&convOk[1]);
        yaw = uiPanel->cmdCurrX->text().toDouble(&convOk[2]);

        if (std::any_of(std::begin(convOk), std::end(convOk), [](bool i)
                        { return !i; }))
        {
            RVIZ_COMMON_LOG_ERROR("Failed to convert current position to floating point");
            
            // set the red stylesheet
            uiPanel->ctrlDiveInPlace->setStyleSheet("QPushButton{color:black; background: red;}");

            // create a timer to clear it in 1 second
            QTimer::singleShot(1000, [this](void)
                               { uiPanel->ctrlDiveInPlace->setStyleSheet(""); });
            return;
        }

        // build the linear control message
        // auto override the control mode to position 
        auto linCmd = riptide_msgs2::msg::ControllerCommand();
        linCmd.setpoint_vect.x = x;
        linCmd.setpoint_vect.y = y;
        // automatically go to configured depth below surface
        linCmd.setpoint_vect.z = tgt_in_place_depth; 
        linCmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;

        // check the angle mode button
        if(degreeReadout){
            yaw *= M_PI / 180.0; 
        }

        // convert RPY to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);

        // build the angular message
        auto angular = tf2::toMsg(quat);
        auto angCmd = riptide_msgs2::msg::ControllerCommand();
        angCmd.setpoint_quat = angular;
        angCmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;

        // send the control messages
        ctrlCmdLinPub->publish(linCmd);
        ctrlCmdAngPub->publish(angCmd);
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
        uiPanel->cmdReqX->setText(x);
        uiPanel->cmdReqY->setText(y);
        uiPanel->cmdReqZ->setText(z);
        uiPanel->cmdReqR->setText(roll);
        uiPanel->cmdReqP->setText(pitch);
        uiPanel->cmdReqYaw->setText(yaw);
    }

    void ControlPanel::handleCommand()
    {
        // first take the current readout and hold it
        // only need xy and yaw, we discard roll and pitch and z
        double x, y, z, roll, pitch, yaw;
        bool convOk[6];

        // make sure that the conversion goes okay as well
        x = uiPanel->cmdReqX->text().toDouble(&convOk[0]);
        y = uiPanel->cmdReqY->text().toDouble(&convOk[1]);
        z = uiPanel->cmdReqZ->text().toDouble(&convOk[2]);
        roll = uiPanel->cmdReqR->text().toDouble(&convOk[3]);
        pitch = uiPanel->cmdReqP->text().toDouble(&convOk[4]);
        yaw = uiPanel->cmdReqYaw->text().toDouble(&convOk[5]);

        if (std::any_of(std::begin(convOk), std::end(convOk), [](bool i)
                        { return !i; }))
        {
            RVIZ_COMMON_LOG_ERROR("Failed to convert current position to floating point");
            // set the red stylesheet
            uiPanel->CtrlSendCmd->setStyleSheet("QPushButton{color:black; background: red;}");

            // create a timer to clear it in 1 second
            QTimer::singleShot(1000, [this](void)
                               { uiPanel->CtrlSendCmd->setStyleSheet(""); });
            return;
        }

        // now we can build the command and send it
        // build the linear control message
        auto linCmd = riptide_msgs2::msg::ControllerCommand();
        linCmd.setpoint_vect.x = x;
        linCmd.setpoint_vect.y = y;
        linCmd.setpoint_vect.z = z;
        linCmd.mode = ctrlMode;

        // if we are in position, we use quat, otherwise use the vector
        auto angCmd = riptide_msgs2::msg::ControllerCommand();
        angCmd.mode = ctrlMode;

        if (ctrlMode == riptide_msgs2::msg::ControllerCommand::POSITION)
        {
            // check the angle mode button
            if(degreeReadout){
                roll *= M_PI / 180.0; 
                pitch *= M_PI / 180.0; 
                yaw *= M_PI / 180.0; 
            }

            // convert RPY to quaternion
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);

            // build the angular quat for message
            angCmd.setpoint_quat = tf2::toMsg(quat);
        } else {
            // build the vector
            angCmd.setpoint_vect.x = roll;
            angCmd.setpoint_vect.y = pitch;
            angCmd.setpoint_vect.z = yaw;
        }

        // send the control messages
        ctrlCmdLinPub->publish(linCmd);
        ctrlCmdAngPub->publish(angCmd);
    }

    void ControlPanel::odomCallback(const nav_msgs::msg::Odometry &msg)
    {
        // parse the quaternion to RPY
        tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // save the header timestamp
        odomTime = msg.header.stamp;

        auto diff = clientNode->get_clock()->now() - odomTime;
        if (diff.to_chrono<std::chrono::seconds>() > odom_timeout * 2)
        {
            RVIZ_COMMON_LOG_WARNING("Recieved odom message with old timestamp");
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

        uiPanel->cmdCurrX->setText(QString::number(msg.pose.pose.position.x, 'f', 2));
        uiPanel->cmdCurrY->setText(QString::number(msg.pose.pose.position.y, 'f', 2));
        uiPanel->cmdCurrZ->setText(QString::number(msg.pose.pose.position.z, 'f', 2));
    }

    void ControlPanel::selectedPose(const geometry_msgs::msg::PoseStamped & msg){
        // check position control mode !!!
        if (ctrlMode == riptide_msgs2::msg::ControllerCommand::POSITION){
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

    void ControlPanel::steadyCallback(const std_msgs::msg::Bool &msg)
    {
        uiPanel->cmdSteady->setEnabled(msg.data);
    }

    // ROS timer callbacks
    void ControlPanel::sendKillMsgTimer()
    {
        auto killMsg = riptide_msgs2::msg::KillSwitchReport();
        killMsg.kill_switch_id = riptide_msgs2::msg::KillSwitchReport::KILL_SWITCH_RQT_CONTROLLER;
        killMsg.sender_id = "/riptide_rviz_control";
        killMsg.switch_asserting_kill = !vehicleEnabled;
        killMsg.switch_needs_update = uiPanel->ctrlRequireKill->isChecked();

        killStatePub->publish(killMsg);
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ControlPanel, rviz_common::Panel);