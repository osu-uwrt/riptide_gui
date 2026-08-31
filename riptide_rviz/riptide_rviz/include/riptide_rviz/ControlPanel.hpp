#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <riptide_msgs2/msg/controller_command.hpp>
#include <riptide_msgs2/msg/kill_switch_report.hpp>
#include <riptide_msgs2/msg/tree_stack.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <interactive_markers/interactive_marker_server.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <riptide_msgs2/action/calibrate_drag_new.hpp>

#include "ui_ControlPanel.h"
#include <QTimer>
#include <QMessageBox>
#include <QString>

#include <vector>

//
// CONTROLLER SELECTION
//
#define CONTROLLER_CMD (0)
#define TARGET_POSITION (1)
#define CONTROLLER_OUTPUT_TYPE CONTROLLER_CMD

namespace riptide_rviz
{
    struct BallastLogData
    {
        double
            depth,
            regPressure,
            regHousingPressure,
            tankPressure;
        
        bool
            exaustState,
            pressureState,
            waterState;
    };

    class ControlPanel : public rviz_common::Panel
    {
        using Trigger = std_srvs::srv::Trigger;
        using SetBool = std_srvs::srv::SetBool;
        using SetPose = robot_localization::srv::SetPose;
        using CalibrateDrag = riptide_msgs2::action::CalibrateDragNew;
        using CalibrateDragGH = rclcpp_action::ClientGoalHandle<CalibrateDrag>;

        Q_OBJECT public : ControlPanel(QWidget *parent = 0);
        ~ControlPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

        // ROS Subscriber callbacks
        void odomCallback(const nav_msgs::msg::Odometry &msg);
        void diagCallback(const diagnostic_msgs::msg::DiagnosticArray &msg);
        void selectedPose(const geometry_msgs::msg::PoseStamped & msg);
        void treeStackCallback(const riptide_msgs2::msg::TreeStack &msg);

        // ROS timer callbacks
        void sendKillMsgTimer();

        enum control_modes {DISABLED, FEEDFORWARD, VELOCITY, POSITION, TELEOP = 255};

    protected Q_SLOTS:
        // QT slots (function callbacks)
        // slots for handling mode setting of the controller
        void handleEnable();
        void handleDisable(); // pressing disable asserts kill and clears command
        void switchMode(uint8_t mode, bool override=false);

        void handleAuxDown();
        void handleAuxUp();

        // slots for controlling the UI
        void toggleDegrees();
        void refreshUI();

        // slots for sending commands to the vehicle
        void handleLocalDive();
        void handleCurrent();
        void handleCommand(bool updateInteractiveMarker);

        //slots for parameter relaod buttons
        void handleReloadController();

        //slots for drag cal buttons
        void handleStartDragCal();
        void handleStopDragCal();
        void handleTriggerDragCal();

        //simualtor apply
        void simulator_apply_clickec();

        //buoyancy buttons
        void handleBallastDive();
        void handleBallastSurface();
        void handleBallastHold();

        void handleBallastToggleExaust();
        void handleBallastTogglePressure();
        void handleBallastToggleWater();

        void startBallastLogData();

        //publish the current set point
        void pubCurrentSetpoint();

    protected:
        bool event(QEvent *event);

    private:
        bool transformBetweenFrames(geometry_msgs::msg::Pose pose_in, geometry_msgs::msg::Pose& pose_out, const std::string& from_frame, const std::string& to_frame);
        bool getDesiredSetpointFromTextboxes(double results[6]);
        void syncSetptMarkerToTextboxes(bool applyChanges = true);
        void setptMarkerFeedback(interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr feedback);
        void updateCalStatus(const std::string& status);
        void callTriggerService(rclcpp::Client<Trigger>::SharedPtr client);
        void callSetBoolService(rclcpp::Client<SetBool>::SharedPtr client, bool value);
        void callSetPoseService(rclcpp::Client<SetPose>::SharedPtr client, std::vector<double> pose);
        void waitForTriggerResponse(rclcpp::Client<Trigger>::SharedPtr client);
        void waitForSetBoolResponse(rclcpp::Client<SetBool>::SharedPtr client);
        void waitForSetPoseResponse(rclcpp::Client<SetPose>::SharedPtr client);
        void setDragCalRunning(bool running);
        void dragGoalResponseCb(const CalibrateDragGH::SharedPtr &goal_handle);
        void dragResultCb(const CalibrateDragGH::WrappedResult &result);
        void setSolenoidStatuses(const bool statuses[3]);
        void getSolenoidStatuses(bool statuses[3]);
        void publishSolenoidStatuses(const bool statuses[3]);
        bool isBallastStateIllegal(bool statuses[3]);
        void exaustSolenoidCb(const std_msgs::msg::Bool& msg);
        void pressureSolenoidCb(const std_msgs::msg::Bool& msg);
        void waterSolenoidCb(const std_msgs::msg::Bool& msg);
        void regPressureCallback(const std_msgs::msg::Float32& msg);
        void regHousingPressureCallback(const std_msgs::msg::Float32& msg);
        void tankPressureCallback(const std_msgs::msg::Float32& msg);
        void updateBallastLog(const rclcpp::Time& now);
        void displayPopupWindow(const std::string& warningMessage, const std::string& text);
        bool checkForDuplicateTopics();
        bool last_duplicate_state;
        std::map<std::string, bool> topic_duplicate_status;
        std::string duplicate_topics_list;

        // UI Panel instance
        Ui_ControlPanel *uiPanel;

        // robot namespace used for config save and load
        std::string robot_ns;
        std::string hostname;

        // doubles for max depth and duration for odom timeout
        double max_depth_in_place, tgt_in_place_depth;
        std::chrono::duration<double> odom_timeout;

        // mode for sending commands to the controller
        uint8_t ctrlMode;

        // last time we have recieved odom
        builtin_interfaces::msg::Time odomTime;

        // internal flags
        bool vehicleEnabled = false;
        bool degreeReadout = true;

        // true while an autonomy tree is running (setpoint marker is hidden in this state)
        bool autonomyActive = false;
        
        //buoyancy parameters
        bool activeBallastEnabled = false;

        //ballast system logging
        bool ballastLogRunning = false;
        std::string ballastLogFileName;
        rclcpp::Time lastBallastLogTime;
        BallastLogData ballastLogData;

        //pixmap for ballast diagram
        std::shared_ptr<QPixmap> ballastDiagram;

        // QT ui timer for handling data freshness
        QTimer *uiTimer;

        //pid for teleop fork-exec
        pid_t teleopPID;

        // publishers
        #if CONTROLLER_TYPE == CONTROLLER_CMD
            rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr ctrlCmdLinPub, ctrlCmdAngPub;
        #elif CONTROLLER_TYPE == TARGET_POSITION
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pidSetptPub;
        #endif

        // ROS Publishers
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auxPub;
        rclcpp::Publisher<riptide_msgs2::msg::KillSwitchReport>::SharedPtr killStatePub;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dragCalTriggerPub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr clawObjectPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
            exaustSolenoidPub,
            pressureSolenoidPub,
            waterSolenoidPub;

        // ROS Timers
        rclcpp::TimerBase::SharedPtr killPubTimer;
        rclcpp::TimerBase::SharedPtr setPointPubTimer;

        //Last Commanded position
        geometry_msgs::msg::Pose lastCommandedPose;

        // ROS Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagSub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selectPoseSub;
        rclcpp::Subscription<riptide_msgs2::msg::TreeStack>::SharedPtr treeStackSub;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
            exaustSolenoidSub,
            pressureSolenoidSub,
            waterSolenoidSub;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
            regPressureSub,
            regHousingPressureSub,
            tankPressureSub;

        //service clients
        rclcpp::Client<Trigger>::SharedPtr 
            reloadCompleteClient,
            reloadLiltankClient;

        rclcpp::Client<SetBool>::SharedPtr setTeleopClient;

        rclcpp::Client<SetPose>::SharedPtr setSimPoseClient;
        
        std::shared_future<Trigger::Response::SharedPtr> activeClientFuture;
        std::shared_future<SetBool::Response::SharedPtr> activeSetBoolClientFuture;
        std::shared_future<SetPose::Response::SharedPtr> activeSetPoseClientFuture;
        int64_t srvReqId;
        rclcpp::Time clientSendTime;

        // action clients
        rclcpp_action::Client<CalibrateDrag>::SharedPtr calibrateDrag;

        //interactive marker server
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> setptServer;
        visualization_msgs::msg::InteractiveMarker interactiveSetpointMarker;

        //tf buffer and listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    };

} // namespace riptide_rviz
