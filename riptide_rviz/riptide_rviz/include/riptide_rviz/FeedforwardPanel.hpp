#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rviz_common/panel.hpp>

#include <riptide_msgs2/msg/electrical_command.hpp>
#include <vectornav_msgs/action/mag_cal.hpp>

#include "ui_FeedforwardPanel.h"

namespace riptide_rviz
{
    const static std::string CALIB_ACTION_NAME = "vectornav/mag_cal";

    class FeedforwardPanel : public rviz_common::Panel
    {
        Q_OBJECT
        public:
        FeedforwardPanel(QWidget *parent = 0);
        ~FeedforwardPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        private slots:
        void someButtonClicked();
        void XFunction(double value);
        void YFunction(double value);
        void ZFunction(double value);
        void RFunction(double value);
        void PFunction(double value);
        void YAWFunction(double value);

       



        private:
        // electrical command vars
        bool loaded = false;
        int counter;
        Ui_FeedforwardPanel *ui;
        QString robotNs;
    };
}