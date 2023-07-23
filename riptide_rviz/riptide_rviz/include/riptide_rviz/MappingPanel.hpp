#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "ui_MappingPanel.h"

namespace riptide_rviz
{
    class MappingPanel : public rviz_common::Panel
    {
        Q_OBJECT public: MappingPanel(QWidget *parent = 0);
        ~MappingPanel();

        void load(const rviz_common::Config &config) override { };
        void save(rviz_common::Config config) const override { };
        void onInitialize() override { };


        private:
        Ui_MappingPanel *ui;
    };
}
