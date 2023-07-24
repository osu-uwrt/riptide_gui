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
        Ui_MappingPanel *ui;
        std::string robotNs;

        rclcpp_action::Client<chameleon_tf_msgs::action::ModelFrame>::SharedPtr calibClient;
    };
}
