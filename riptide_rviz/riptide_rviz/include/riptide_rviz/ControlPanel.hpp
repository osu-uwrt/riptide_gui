#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <riptide_msgs2/msg/controller_command.hpp>
#include <riptide_msgs2/msg/kill_switch_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

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

//
// CONTROLLER SELECTION
//
#define CONTROLLER_CMD (0)
#define TARGET_POSITION (1)
#define CONTROLLER_TYPE CONTROLLER_CMD

namespace riptide_rviz
{

    class ControlPanel : public rviz_common::Panel
    {
        using Trigger = std_srvs::srv::Trigger;
        using CalibrateDrag = riptide_msgs2::action::CalibrateDragNew;
        using CalibrateDragGH = rclcpp_action::ClientGoalHandle<CalibrateDrag>;

        Q_OBJECT public : ControlPanel(QWidget *parent = 0);
        ~ControlPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

        // ROS Subscriber callbacks
        void odomCallback(const nav_msgs::msg::Odometry &msg);
        void limitsCallback(const std_msgs::msg::Int8 &msg);
        void selectedPose(const geometry_msgs::msg::PoseStamped & msg);

        // ROS timer callbacks
        void sendKillMsgTimer();

    protected Q_SLOTS:
        // QT slots (function callbacks)
        // slots for handling mode setting of the controller
        void handleEnable();
        void handleDisable(); // pressing disable asserts kill and clears command
        void switchMode(uint8_t mode, bool override=false);

        // slots for controlling the UI
        void toggleDegrees();
        void refreshUI();

        // slots for sending commands to the vehicle
        void handleLocalDive();
        void handleCurrent();
        void handleCommand(bool updateInteractiveMarker);

        //slots for parameter relaod buttons
        void handleReloadSolver();
        void handleReloadActive();

        //slots for drag cal buttons
        void handleStartDragCal();
        void handleStopDragCal();
        void handleTriggerDragCal();

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
        void waitForTriggerResponse(rclcpp::Client<Trigger>::SharedPtr client);
        void setDragCalRunning(bool running);
        void dragGoalResponseCb(const CalibrateDragGH::SharedPtr &goal_handle);
        void dragResultCb(const CalibrateDragGH::WrappedResult &result);
        
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

        // QT ui timer for handling data freshness
        QTimer *uiTimer;

        // publishers
        #if CONTROLLER_TYPE == CONTROLLER_CMD
            rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr ctrlCmdLinPub, ctrlCmdAngPub;
        #elif CONTROLLER_TYPE == TARGET_POSITION
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pidSetptPub;
        #endif

        rclcpp::Publisher<riptide_msgs2::msg::KillSwitchReport>::SharedPtr killStatePub;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dragCalTriggerPub;

        // ROS Timers
        rclcpp::TimerBase::SharedPtr killPubTimer;
        rclcpp::TimerBase::SharedPtr setPointPubTimer;

        //Last Commanded position
        geometry_msgs::msg::Pose lastCommandedPose;

        // ROS Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr limitsSub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selectPoseSub;

        //service clients
        rclcpp::Client<Trigger>::SharedPtr 
            reloadSolverClient,
            reloadSmcClient,
            reloadPidClient;
        
        std::shared_future<Trigger::Response::SharedPtr> activeClientFuture;
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
