#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "ui_ParamPanel.h"

namespace riptide_rviz
{
    class ParamPanel : public rviz_common::Panel
    {
        Q_OBJECT
        public:
        ParamPanel(QWidget *parent = 0);
        ~ParamPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

        private slots:
        void buttonPressed();

        private:

        bool
            loaded = false;
                
        int counter;
        Ui_ParamPanel *ui;
        QString robotNs;
    };
}