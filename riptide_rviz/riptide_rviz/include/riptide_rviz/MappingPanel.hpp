#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

#include <chameleon_tf_msgs/action/model_frame.hpp>
#ifdef USE_ZED_INTERFACES
    #include <zed_interfaces/srv/start_svo_rec.hpp>
#endif
#include <std_srvs/srv/trigger.hpp>
#include <riptide_msgs2/msg/mapping_target_info.hpp>
#include <riptide_msgs2/srv/mapping_target.hpp>

#include "ui_MappingPanel.h"

#include "riptide_rviz/GuiSrvClient.hpp"

namespace riptide_rviz
{
    class MappingPanel : public rviz_common::Panel
    {
        using ModelFrame = chameleon_tf_msgs::action::ModelFrame;
        using SendGoalOptions = rclcpp_action::Client<ModelFrame>::SendGoalOptions;
        using CalibGoalHandle = rclcpp_action::Client<ModelFrame>::GoalHandle;

        using MappingTarget = riptide_msgs2::srv::MappingTarget;
        #ifdef USE_ZED_INTERFACES
            using StartSvoRec = zed_interfaces::srv::StartSvoRec;
        #endif
        using Trigger = std_srvs::srv::Trigger;

        Q_OBJECT 
        public:
        MappingPanel(QWidget *parent = 0);
        ~MappingPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        private Q_SLOTS:
        void calibMapFrame();
        void setMappingTarget();
        void zedSvoStart();
        void zedSvoStop();
        void dfcRecordStart();
        void dfcRecordStop();

        private:
        void setStatus(const QString& text, const QString &color);

        template<typename T>
        void serviceResponseCb(const std::string& srvName, typename rclcpp::Client<T>::SharedResponse response)
        {
            std::string successStr = (response->success ? "Succeeded" : "Failed");

            setStatus(QString::fromStdString("Call to %1 %2; %3").arg(
                QString::fromStdString(srvName), QString::fromStdString(successStr), QString::fromStdString(response->message)),
                (response->success ? "000000" : "FF0000"));
        }

        // tag cal stuff
        void goalResponseCb(const CalibGoalHandle::SharedPtr & goal_handle);
        void feedbackCb(
            CalibGoalHandle::SharedPtr,
            const std::shared_ptr<const ModelFrame::Feedback> feedback);
        void resultCb(const CalibGoalHandle::WrappedResult & result);        

        // mappingtarget stuff
        void mappingStatusCb(const riptide_msgs2::msg::MappingTargetInfo::SharedPtr msg);
        void mappingTargetResultCb(const std::string& srvName, rclcpp::Client<MappingTarget>::SharedResponse response);

        Ui_MappingPanel *ui;
        std::string robotNs;
        bool calibrationInProgress,
            loaded = false;
        
        rclcpp_action::Client<chameleon_tf_msgs::action::ModelFrame>::SharedPtr calibClient;
        rclcpp::Subscription<riptide_msgs2::msg::MappingTargetInfo>::SharedPtr mappingTargetInfoSub;
        GuiSrvClient<MappingTarget>::SharedPtr mappingTargetClient;

        #ifdef USE_ZED_INTERFACES
            GuiSrvClient<StartSvoRec>::SharedPtr startSvoClient;
            GuiSrvClient<Trigger>::SharedPtr stopSvoClient;
        #endif

        //TODO add dfc clients
    };
}
