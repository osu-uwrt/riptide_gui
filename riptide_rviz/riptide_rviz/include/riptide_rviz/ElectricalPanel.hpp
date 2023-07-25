#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include "riptide_msgs2/msg/electrical_command.hpp"
#include "ui_ElectricalPanel.h"

namespace riptide_rviz
{
    class ElectricalPanel : public rviz_common::Panel
    {
        Q_OBJECT
        public:
        ElectricalPanel(QWidget *parent = 0);
        ~ElectricalPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        private Q_SLOTS:
        void sendCommand();

        private:
        Ui_ElectricalPanel *ui;
        QString robotNs;
        rclcpp::Publisher<riptide_msgs2::msg::ElectricalCommand>::SharedPtr pub;
    };
}