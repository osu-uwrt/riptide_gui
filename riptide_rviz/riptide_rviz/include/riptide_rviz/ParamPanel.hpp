#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <QGridLayout>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QComboBox>

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
        void apply();
        void setNode(const QString &text);
        void refresh();

        private:

        void getNodes();
        std::vector<rclcpp::Parameter> getParams();
        void createParams(std::vector<rclcpp::Parameter> params);
        void eraseLayout(QLayout* layout);

        bool loaded = 0, connected = 0;

        std::shared_ptr<rclcpp::Node> node;
        std::shared_ptr<rclcpp::SyncParametersClient> param_client;
        QComboBox* nodes;
        QVBoxLayout* paramLayout;
        QPushButton* applyButton;
        QPushButton* refreshButton;
        QString robotNs;
    };
}