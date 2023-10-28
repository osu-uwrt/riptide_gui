#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <QGridLayout>
#include <QPushButton>
#include <QDoubleSpinBox>

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
        void setValue(double val);

        private:

        bool
            loaded = false;
                
        QGridLayout *paramGrid;
        std::vector<QDoubleSpinBox*> widgetVec;
        QPushButton* apply;
        Ui_ParamPanel *ui;
        QString robotNs;
    };
}