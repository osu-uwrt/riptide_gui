#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

#include "chameleon_tf_msgs/action/model_frame.hpp"
#include "ui_MappingPanel.h"

namespace riptide_rviz
{
    class MappingPanel : public rviz_common::Panel
    {
        using ModelFrame = chameleon_tf_msgs::action::ModelFrame;
        using SendGoalOptions = rclcpp_action::Client<ModelFrame>::SendGoalOptions;
        using CalibGoalHandle = rclcpp_action::Client<ModelFrame>::GoalHandle;
        
        Q_OBJECT 
        public:
        MappingPanel(QWidget *parent = 0);
        ~MappingPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        private Q_SLOTS:
        void calibMapFrame();

        private:
        void setCalibStatus(const QString& text, const QString& color);
        void goalResponseCb(const CalibGoalHandle::SharedPtr & goal_handle);
        void feedbackCb(
            CalibGoalHandle::SharedPtr,
            const std::shared_ptr<const ModelFrame::Feedback> feedback);
        void resultCb(const CalibGoalHandle::WrappedResult & result);

        Ui_MappingPanel *ui;
        std::string robotNs;
        bool 
            calibrationInProgress,
            loaded = false;

        rclcpp_action::Client<chameleon_tf_msgs::action::ModelFrame>::SharedPtr calibClient;
    };
}
