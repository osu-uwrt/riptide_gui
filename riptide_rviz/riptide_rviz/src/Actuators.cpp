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
        armed_flag = false;
        // Connect UI signals for controlling the riptide vehicle
        connect(uiPanel->actDrop1, &QPushButton::clicked, [this](void)
                { handleDroppers(1); });
        connect(uiPanel->actDrop2, &QPushButton::clicked, [this](void)
                { handleDroppers(2); });
        connect(uiPanel->actTorp1, &QPushButton::clicked, [this](void)
                { handleTorpedos(1); });
        connect(uiPanel->actTorp2, &QPushButton::clicked, [this](void)
                { handleTorpedos(2); });
        connect(uiPanel->actDropArm, &QPushButton::clicked, [this](void)
                { handleArming(false, true); });
        connect(uiPanel->actTorpArm, &QPushButton::clicked, [this](void)
                { handleArming(true, false); });
        connect(uiPanel->actClawClose, &QPushButton::clicked, [this](void)
                { handleClaw(true); });
        connect(uiPanel->actClawOpen, &QPushButton::clicked, [this](void)
                { handleClaw(false); });
    }

    void Actuators::handleArming(bool arm_torpedos, bool arm_droppers)
    {
        auto goal = ArmTorpedoDropper::Goal();
        if (armed_flag) {
            goal.arm_torpedos = false;
            goal.arm_droppers = false;
        } else {
            goal.arm_torpedos = arm_torpedos;
            goal.arm_droppers = arm_droppers;
        }
        
        // create the goal callbacks to bind to
        auto sendGoalOptions = rclcpp_action::Client<ArmTorpedoDropper>::SendGoalOptions();
        sendGoalOptions.goal_response_callback =
            std::bind(&Actuators::armTaskStartCb, this, _1);
        sendGoalOptions.feedback_callback =
            std::bind(&Actuators::armTaskFeedbackCb, this, _1, _2);
        sendGoalOptions.result_callback =
            std::bind(&Actuators::armTaskCompleteCb, this, _1);

        // send the goal with the callbacks configured
        armTorpedoDropper->async_send_goal(goal, sendGoalOptions); 
    }

    void Actuators::armTaskStartCb(const GHArmTorpedoDropper::SharedPtr &goalHandle)
    { 
        if (!goalHandle) {
            RVIZ_COMMON_LOG_ERROR("ActuatorPanel: REJECTED: Failed to arm");
            
            // set the red stylesheet
            uiPanel->actDropArm->setStyleSheet("QPushButton{color:black; background: red;}");
            uiPanel->actTorpArm->setStyleSheet("QPushButton{color:black; background: red;}");

            // create a timer to clear it in 1 second
            QTimer::singleShot(
                1000, 
                [this](void) {
                    uiPanel->actDropArm->setStyleSheet("QPushButton{color:black; background: green;}");
                    uiPanel->actTorpArm->setStyleSheet("QPushButton{color:black; background: green;}"); 
                }
            );
        }
    }

    void Actuators::armTaskCompleteCb(const GHArmTorpedoDropper::WrappedResult &result)
    {
        // we can re-enable the staret button and disable the stop button
        uiPanel->actDropArm->setStyleSheet("QPushButton{color:black; background: green;}");
        uiPanel->actTorpArm->setStyleSheet("QPushButton{color:black; background: green;}");
    }

    void Actuators::armTaskFeedbackCb(GHArmTorpedoDropper::SharedPtr goalHandle,
                                      ArmTorpedoDropper::Feedback::ConstSharedPtr feedback)
    {
        armed_flag = feedback->is_armed;
        if (armed_flag) {
            uiPanel->actDropArm->setStyleSheet("QPushButton{color:black; background: red;}");
            uiPanel->actTorpArm->setStyleSheet("QPushButton{color:black; background: red;}");
        }
    }

    void Actuators::handleClaw(bool clawopen)
    {
       
        
        auto goal = ChangeClawState::Goal();
        goal.clawopen = clawopen;

        // create the goal callbacks to bind to
        auto sendGoalOptions = rclcpp_action::Client<ChangeClawState>::SendGoalOptions();
        sendGoalOptions.goal_response_callback =
            std::bind(&Actuators::clawTaskStartCb, this, _1);
        sendGoalOptions.result_callback =
            std::bind(&Actuators::clawTaskCompleteCb, this, _1);

        // send the goal with the callbacks configured
        changeClawState->async_send_goal(goal, sendGoalOptions);
    }

    void Actuators::clawTaskStartCb(const GHChangeClawState::SharedPtr &goalHandle)
    { 
        if (!goalHandle) {
            RVIZ_COMMON_LOG_ERROR("ActuatorPanel: REJECTED: Failed to change claw state");
            
            // set the red stylesheet
            uiPanel->actClawOpen->setStyleSheet("QPushButton{color:black; background: red;}");
            uiPanel->actClawClose->setStyleSheet("QPushButton{color:black; background: red;}");

            // create a timer to clear it in 1 second
            QTimer::singleShot(
                1000, 
                [this](void) {
                    uiPanel->actClawOpen->setStyleSheet("");
                    uiPanel->actClawClose->setStyleSheet(""); 
                }
            );
        }
    }

    void Actuators::clawTaskCompleteCb(const GHChangeClawState::WrappedResult &result)
    {
        // we can re-enable the staret button and disable the stop button
        uiPanel->actClawOpen->setEnabled(true);
        uiPanel->actClawClose->setEnabled(false);
    }


    void Actuators::handleDroppers(int dropper_id)
    {
       
        if (!uiPanel->actDropArm->isEnabled())
        {
            auto goal = ActuateDroppers::Goal();
            goal.dropper_id = dropper_id;

            // create the goal callbacks to bind to
            auto sendGoalOptions = rclcpp_action::Client<ActuateDroppers>::SendGoalOptions();
            sendGoalOptions.goal_response_callback =
                std::bind(&Actuators::dropperTaskStartCb, this, _1);
            sendGoalOptions.result_callback =
                std::bind(&Actuators::dropperTaskCompleteCb, this, _1);

            // send the goal with the callbacks configured
            actuateDroppers->async_send_goal(goal, sendGoalOptions);
        }
    }

    void Actuators::dropperTaskStartCb(const GHActuateDropper::SharedPtr &goalHandle)
    { 
        uiPanel->actDrop1->setEnabled(false);
        uiPanel->actDrop2->setEnabled(false);
            
    }

    void Actuators::dropperTaskCompleteCb(const GHActuateDropper::WrappedResult &result)
    {
        // we can re-enable the staret button and disable the stop button
        uiPanel->actDropArm->setEnabled(true);
    }

    void Actuators::handleTorpedos(int torpedo_id)
    {
       
        if (!uiPanel->actTorpArm->isEnabled())
        {
            auto goal = ActuateTorpedos::Goal();
            goal.torpedo_id = torpedo_id;

            // create the goal callbacks to bind to
            auto sendGoalOptions = rclcpp_action::Client<ActuateTorpedos>::SendGoalOptions();
            sendGoalOptions.goal_response_callback =
                std::bind(&Actuators::torpedoTaskStartCb, this, _1);
            sendGoalOptions.result_callback =
                std::bind(&Actuators::torpedoTaskCompleteCb, this, _1);

            // send the goal with the callbacks configured
            actuateTorpedos->async_send_goal(goal, sendGoalOptions);   
        }
    }

    void Actuators::torpedoTaskStartCb(const GHActuateTorpedos::SharedPtr &goalHandle)
    { 
            uiPanel->actTorp1->setEnabled(false);
            uiPanel->actTorp2->setEnabled(false);
            
    }

    void Actuators::torpedoTaskCompleteCb(const GHActuateTorpedos::WrappedResult &result)
    {
        // we can re-enable the staret button and disable the stop button
        uiPanel->actTorpArm->setEnabled(true);
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


        // now init the clients
        armTorpedoDropper = rclcpp_action::create_client<ArmTorpedoDropper>(node, robot_ns + "/arm_actuators");
        changeClawState = rclcpp_action::create_client<ChangeClawState>(node, robot_ns + "/change_claw_state");
        actuateTorpedos = rclcpp_action::create_client<ActuateTorpedos>(node, robot_ns + "/actuate_torpedos");
        actuateDroppers = rclcpp_action::create_client<ActuateDroppers>(node, robot_ns + "/actuate_droppers");

        delete str;
    }

    void Actuators::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        // write our config values
        config.mapSetValue("robot_namespace", QString::fromStdString(robot_ns));
    }

    bool Actuators::event(QEvent *event)
    {
    }
        
    Actuators::~Actuators()
    {
        // master window control removal
        delete uiPanel;
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::Actuators, rviz_common::Panel);
