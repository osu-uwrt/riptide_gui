#include "riptide_rviz/ControlPanel.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>

#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

#include <unistd.h>

#define MAX_HOST_LEN 300

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
        ctrlMode = riptide_msgs2::msg::ControllerCommand::DISABLED;

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
                { switchMode(riptide_msgs2::msg::ControllerCommand::POSITION); });
        connect(uiPanel->ctrlModeVel, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::VELOCITY); });
        connect(uiPanel->ctrlModeFFD, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::FEEDFORWARD); });
        connect(uiPanel->ctrlModeTele, &QPushButton::clicked,
                [this](void)
                { RVIZ_COMMON_LOG_INFO("ControlPanel: bad button >:("); });

        // command sending buttons
        connect(uiPanel->ctrlDiveInPlace, &QPushButton::clicked, this, &ControlPanel::handleLocalDive);
        connect(uiPanel->ctrlFwdCurrent, &QPushButton::clicked, this, &ControlPanel::handleCurrent);
        connect(uiPanel->CtrlSendCmd, &QPushButton::clicked, this, &ControlPanel::handleCommand);

        //parameter reload buttons
        connect(uiPanel->reloadSolver, &QPushButton::clicked, this, &ControlPanel::handleReloadSolver);
        connect(uiPanel->reloadActive, &QPushButton::clicked, this, &ControlPanel::handleReloadActive);

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

        // setup the last duplicate state
        last_duplicate_state = false;

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
        #if CONTROLLER_TYPE == OLD
            ctrlCmdLinPub = node->create_publisher<riptide_msgs2::msg::ControllerCommand>(robot_ns + "/controller/linear", rclcpp::SystemDefaultsQoS());
            ctrlCmdAngPub = node->create_publisher<riptide_msgs2::msg::ControllerCommand>(robot_ns + "/controller/angular", rclcpp::SystemDefaultsQoS());
        #elif CONTROLLER_TYPE == SMC

        #elif CONTROLLER_TYPE == PID
            pidSetptPub = node->create_publisher<geometry_msgs::msg::Pose>(robot_ns + "/pid/target_position", rclcpp::SystemDefaultsQoS());
        #endif
        
        dragCalTriggerPub = node->create_publisher<std_msgs::msg::Empty>(robot_ns + "/trigger", rclcpp::SystemDefaultsQoS());

        // make ROS Subscribers
        odomSub = node->create_subscription<nav_msgs::msg::Odometry>(
            robot_ns + "/odometry/filtered", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::odomCallback, this, _1));
        steadySub = node->create_subscription<std_msgs::msg::Bool>(
            robot_ns + "/controller/steady", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::steadyCallback, this, _1));

        //create service clients
        reloadSolverClient = node->create_client<Trigger>(robot_ns + "/controller_overseer/update_ts_params");
        reloadActiveClient = node->create_client<Trigger>(robot_ns + "/controller_overseer/update_active_params");

        //create action clients
        calibrateDrag = rclcpp_action::create_client<CalibrateDrag>(node, robot_ns + "/calibrate_drag_new");

        // now we can start the UI refresh timer
        uiTimer->start(100);

        // and start the kill switch pub timer
        killPubTimer = node->create_wall_timer(50ms, std::bind(&ControlPanel::sendKillMsgTimer, this));

        // save the hostanme for kill instance
        hostname = "rviz_control_" + get_hostname();

        RVIZ_COMMON_LOG_INFO("ControlPanel: Loading config complete");
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
                RVIZ_COMMON_LOG_ERROR("ControlPanel: Button not yet operable");
                break;
            }
        }
    }

    void ControlPanel::refreshUI()
    {
        // Check for duplicate topics and disable UI if duplicates are found
        if (checkForDuplicateTopics()) {
            handleDisable();
            uiPanel->ctrlEnable->setEnabled(false);
            uiPanel->ctrlDiveInPlace->setEnabled(false);
            uiPanel->CtrlSendCmd->setEnabled(false);
            
            if (!last_duplicate_state){
                last_duplicate_state = true;
                // Display a popup window
                std::string warningMessage = "Duplicate topics detected!";
                displayPopupWindow(warningMessage, duplicate_topics_list);
                
            }

            return; // Early return to prevent further UI updates  

        } else{
            if (last_duplicate_state){
                uiPanel->ctrlEnable->setEnabled(true);
                last_duplicate_state = false;
            }
        }

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

    bool ControlPanel::checkForDuplicateTopics() {
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        auto topics = node->get_topic_names_and_types();
        std::map<std::string, int> topic_count;
        bool anyDuplicates = false;
        duplicate_topics_list = "";

        // Count each topic's occurrences
        for (const auto& topic_info : topics) {
            topic_count[topic_info.first]++;
        }

        // Check for changes in duplicate status
        for (const auto& count : topic_count) {

            bool is_duplicate = count.second > 1;
            anyDuplicates |= is_duplicate;  // If any topic is a duplicate, set to true
            auto it = topic_duplicate_status.find(count.first);

            if (it == topic_duplicate_status.end() || it->second != is_duplicate) {

                topic_duplicate_status[count.first] = is_duplicate;  // Update the status in the map
                if (is_duplicate) {
                    RVIZ_COMMON_LOG_WARNING_STREAM("Duplicate topic detected: " << count.first);
                    duplicate_topics_list += count.first + "\n";
                } else if (it != topic_duplicate_status.end()) {
                    RVIZ_COMMON_LOG_INFO_STREAM("Duplicate resolved: " << count.first);
                }
            }
        }

        // Cleanup map entries for topics no longer present
        for (auto it = topic_duplicate_status.begin(); it != topic_duplicate_status.end(); ) {
            
            if (topic_count.find(it->first) == topic_count.end()) {
                it = topic_duplicate_status.erase(it);  // Remove from map if topic no longer exists
            } else {
                ++it;
            }
        }

        return anyDuplicates;
    }

    void ControlPanel::displayPopupWindow(const std::string& warningMessage, const std::string& text){

        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText(QString::fromStdString(warningMessage));

        QString informativeText = QString::fromStdString(text);
        msgBox.setInformativeText(informativeText);

        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();  
        
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

        handleCommand();
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
        linear.x = x;
        linear.y = y;
        linear.z = z;

        geometry_msgs::msg::Quaternion angularPosition;
        geometry_msgs::msg::Vector3 angularVelocity;
        

        // if we are in position, we use quat, otherwise use the vector
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
            angularPosition = tf2::toMsg(quat);
        } else {
            // build the vector
            angularVelocity.x = roll;
            angularVelocity.y = pitch;
            angularVelocity.z = yaw;
        }
    
        #if CONTROLLER_TYPE == OLD
            auto linCmd = riptide_msgs2::msg::ControllerCommand();
            linCmd.setpoint_vect.x = linear.x;
            linCmd.setpoint_vect.y = linear.y;
            linCmd.setpoint_vect.z = linear.z;
            linCmd.mode = ctrlMode;

            auto angCmd = riptide_msgs2::msg::ControllerCommand();
            angCmd.mode = ctrlMode;
            angCmd.setpoint_quat = angularPosition;
            angCmd.setpoint_vect = angularVelocity;

            // send the control messages
            ctrlCmdLinPub->publish(linCmd);
            ctrlCmdAngPub->publish(angCmd);
        #elif CONTROLLER_TYPE == SMC

        #elif CONTROLLER_TYPE == PID
            geometry_msgs::msg::Pose setpt;
            setpt.position = linear;
            setpt.orientation = angularPosition;
            pidSetptPub->publish(setpt);
        #endif
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
        killMsg.sender_id = hostname;
        killMsg.switch_asserting_kill = !vehicleEnabled;
        killMsg.switch_needs_update = uiPanel->ctrlRequireKill->isChecked();

        killStatePub->publish(killMsg);
    }


    void ControlPanel::handleReloadSolver()
    {
        updateCalStatus("Attempting to invoke solver reload service");
        callTriggerService(reloadSolverClient);
    }


    void ControlPanel::handleReloadActive()
    {
        updateCalStatus("Attempting to invoke active reload service");
        callTriggerService(reloadActiveClient);
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


    void ControlPanel::waitForTriggerResponse(rclcpp::Client<Trigger>::SharedPtr client)
    {
        if(!activeClientFuture.valid())
        {
            updateCalStatus("Service result has become invalid!");
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
            updateCalStatus("Service timed out.");
            client->remove_pending_request(srvReqId);
            return;
        }

        //schedule next check
        QTimer::singleShot(250, 
            [this, client] () { ControlPanel::waitForTriggerResponse(client); });
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
