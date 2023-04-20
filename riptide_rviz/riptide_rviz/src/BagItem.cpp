#include "riptide_rviz/BagItem.hpp"

#include <algorithm>

#include <rviz_common/logging.hpp>

namespace riptide_rviz
{
    BagItem::BagItem(std::string hostName, std::shared_ptr<rclcpp::Node> parentNode, QWidget *overallParent){
        uiPanel = new Ui_BagListElement();
        uiPanel->setupUi(this);
        // mainParent = overallParent;

        //set the item to blank state
        uiPanel->startButton->setEnabled(true);
        uiPanel->stopButton->setEnabled(false);
        uiPanel->bagName->setText("CCCCCC");
        uiPanel->bagCfg->setEnabled(true);

        // conect the buttons
        connect(uiPanel->bagCfg, &QPushButton::clicked, [this](void){baggingConfigure();});
        connect(uiPanel->startButton, &QPushButton::clicked, [this](void){startBagging();});
        connect(uiPanel->stopButton, &QPushButton::clicked, [this](void){stopBagging();});

        // set the default state
        bid = -1;

        RVIZ_COMMON_LOG_INFO("BagItem: created child item " + uiPanel->bagName->text().toStdString());
    }

    BagItem::~BagItem(){
        delete uiPanel;
    }

    bool BagItem::event(QEvent *event){
        return false;
    }

    void BagItem::baggingConfigure(){
        RVIZ_COMMON_LOG_INFO("BagItem: configuring " + uiPanel->bagName->text().toStdString());

    }

    void BagItem::startBagging(){
        if(bid >= 0){
            //cant start wont start, already started
            return;
        }

        // now we can do the thing
        RVIZ_COMMON_LOG_INFO("BagItem: starting bag " + uiPanel->bagName->text().toStdString());
    }   

    void BagItem::bagAlive(std::vector<int> bids){
        if(std::find(bids.begin(), bids.end(), bid) != bids.end()){
            // bag is alive
        } else {
            // bag is dead
        }
    }

    void BagItem::stopBagging(){
        if(bid < 0){
            // cant stop wont stop, already stopped
            return;
        }

        // now we can send the stop request
        RVIZ_COMMON_LOG_INFO("BagItem: stopping bag " + uiPanel->bagName->text().toStdString());
    }

} // namespace riptide_rviz
