#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ui_BagItem.h"

namespace riptide_rviz
{
    class BagItem : public QWidget{
    public:
        BagItem(std::string hostName, std::shared_ptr<rclcpp::Node> parentNode, QWidget *overallParent);
        ~BagItem();
        

    protected:
        bool event(QEvent *event);
        void baggingConfigure();

    private:
        // UI Panel instance
        Ui_BagItem *uiPanel;

        // bag id
        int bid;
    };
} // namespace riptide_rviz