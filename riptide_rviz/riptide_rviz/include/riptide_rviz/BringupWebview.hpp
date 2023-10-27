#pragma once
#include <rclcpp/rclcpp.hpp>
#include <launch_msgs/msg/list_pids.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <QString>
#include <QVBoxLayout>
#include <QWebEngineView>
#include <vector>

namespace riptide_rviz
{
    class BringupWebview : public rviz_common::Panel
    {
        Q_OBJECT public : BringupWebview(QWidget *parent = 0);
        ~BringupWebview();

        std::string getFromConfig(const rviz_common::Config &config, const QString& key, const QString& defaultValue);
        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

    private:
        void delayedLoadUi();
    
        QVBoxLayout *vLayout; 
        QWebEngineView *webEngineView;

        rclcpp::TimerBase::SharedPtr delayTimer;

        // This should be the Orin's IP
        QString host = "127.0.0.1";
        QString port = "8080";

    protected:
        bool event(QEvent *event);
        
    };

} // namespace riptide_rviz