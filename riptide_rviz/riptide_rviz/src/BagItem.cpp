#include "riptide_rviz/BagItem.hpp"

#include <rviz_common/logging.hpp>

namespace riptide_rviz
{
    BagItem::BagItem(std::string hostName, std::shared_ptr<rclcpp::Node> parentNode, QWidget *overallParent){
        uiPanel = new Ui_BagItem();
        uiPanel->setupUi(this);
        // mainParent = overallParent;

        //set the item to blank state
        uiPanel->bagStart->setEnabled(true);
        uiPanel->bagStop->setEnabled(false);
        uiPanel->bagCfg->setEnabled(true);

        // conect the buttons
        connect(uiPanel->bagCfg, &QPushButton::clicked, [this](void){baggingConfigure();});

        // set the default state
        bid = -1;

        RVIZ_COMMON_LOG_INFO("BagItem: created child item");
    }

    BagItem::~BagItem(){
        delete uiPanel;
    }

    bool BagItem::event(QEvent *event){
        return false;
    }

    void BagItem::baggingConfigure(){
        RVIZ_COMMON_LOG_INFO("BagItem: configuring");

    }
} // namespace riptide_rviz
