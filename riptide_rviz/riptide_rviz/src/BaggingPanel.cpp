#include "riptide_rviz/BaggingPanel.hpp"

#include <iostream>
#include <filesystem>
#include <chrono>
#include <string>

#include <QLabel>
#include <QDialog>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

#define RVIZ_PKG "riptide_rviz"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace riptide_rviz
{

    BaggingPanel::BaggingPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);
        mainParent = parent;

        uiPanel = new Ui_BaggingPanel();
        uiPanel->setupUi(this);
    }

    void BaggingPanel::onInitialize()
    {
        // refresh UI elements so they start displayed correctly
        hostListRefresh();
        fileListRefresh();

        // also create the scroll area
        clearScrollArea();

        // Connect UI signals for bagging
        connect(uiPanel->baggingRefresh, &QPushButton::clicked, [this](void)
                { fileListRefresh(); hostListRefresh();});
        connect(uiPanel->baggingStart, &QPushButton::clicked, [this](void) { startBagging(); });
        connect(uiPanel->baggingHost, SIGNAL(currentTextChanged(const QString &)), SLOT(handleBaggingPanelHost(const QString &)));
        connect(uiPanel->baggingFile, SIGNAL(currentTextChanged(const QString &)), SLOT(handleBaggingPanelFile(const QString &)));
    }

    void BaggingPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }

    void BaggingPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
    }

    bool BaggingPanel::event(QEvent *event)
    {
        return false;
    }

    void BaggingPanel::clearScrollArea()
    {
        RVIZ_COMMON_LOG_INFO("BaggingPanel: clearing scroll area");

        // if the old stuff exists, remove it
        if(bagList.size() > 0){
            for(auto listObject : bagList)
            {
                delete listObject;
            }

            bagList.clear();
        }

        if(vbox != nullptr){
            RVIZ_COMMON_LOG_DEBUG("BaggingPanels: deleting vbox");
            delete vbox;
        }
        
        // also clear the scroll area data if it exists
        if(scrollAreaLayout != nullptr){
            RVIZ_COMMON_LOG_DEBUG("BaggingPanel: deleting scroll area");
            delete scrollAreaLayout;
        }

        RVIZ_COMMON_LOG_DEBUG("BaggingPanel: creating new scroll area");
        
        // Add Vertical Box layout to the Scroll area so we can actually add items to it
        scrollAreaLayout = new QWidget(mainParent);
        vbox = new QVBoxLayout(scrollAreaLayout);
        vbox->setAlignment(Qt::AlignTop);
        vbox->setSpacing(0);
        scrollAreaLayout->setLayout(vbox);
        uiPanel->scrollArea->setWidget(scrollAreaLayout);
    }

    BaggingPanel::~BaggingPanel()
    {
        // master window control removal
        delete uiPanel;
    }

    void BaggingPanel::baggingConfigure()
    {
        // QDialog* selectionDialog = new QDialog();

        // Ui_BaggingTopicSelection uiSelection;
        // uiSelection.setupUi(selectionDialog);

        // uiSelection.topicSelector->setModel(topicModel);

        // selectionDialog->exec();

        // delete selectionDialog;
    }

    void BaggingPanel::hostListRefresh()
    {
        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // get the list of nodes available
        std::vector<std::string> names = node->get_node_names();

        // clear the entries in the combo boxes
        uiPanel->baggingHost->clear();

        // filter names down to bagging nodes
        std::vector<std::string> baggingNodes;
        auto filt = [](const auto &s)
        { return s.find("_bagging_manager") != std::string::npos; };
        std::copy_if(names.begin(), names.end(), std::back_inserter(baggingNodes), filt);

        // stop any events while we refresh the list
        uiPanel->baggingHost->blockSignals(false);

        // make sure we have availiable nodes in the list, otherwise place a blank list entry
        if (baggingNodes.size() > 0)
        {
            uiPanel->baggingHost->addItem("None Selected");
            uiPanel->baggingHost->setCurrentIndex(0);
            for (std::string const &name : baggingNodes)
            {
                // push these into the combo box
                uiPanel->baggingHost->addItem(QString::fromStdString(name.substr(0, name.find_last_of('b') - 1)));
            }
        }
        else
        {
            uiPanel->baggingHost->addItem("None");
        }

        // restart events while we refresh the list
        uiPanel->baggingHost->blockSignals(true);
    }

    void BaggingPanel::fileListRefresh(){
        // clear the list of bringup files
        uiPanel->baggingFile->blockSignals(true);
        uiPanel->baggingFile->clear();
        uiPanel->baggingFile->addItem("None Selected");
        uiPanel->baggingFile->setCurrentIndex(0);
        
        // populate with new files
        auto bringupFilesDir = ament_index_cpp::get_package_share_directory(RVIZ_PKG) + "/recipies";
        for (const auto &entry : std::filesystem::directory_iterator(bringupFilesDir))
        {
            std::string file = std::string(entry.path());
            if (file.find(".xml") != std::string::npos)
            {
                uiPanel->baggingFile->addItem(QString::fromStdString(file.substr(file.find_last_of('/') + 1)));
            }
        }
        uiPanel->baggingFile->blockSignals(false);
        uiPanel->baggingFile->setEnabled(true); // SET this back to false

        // clear the scroll area
        clearScrollArea();
    }

    void BaggingPanel::handleBaggingPanelHost(const QString & str)
    {
        std::string targetNode = str.toStdString();

        // Remove previous ros stuff 
        if (baggingStateSub != nullptr) {
            baggingStateSub.reset();
        }

        // make sure the event wasnt generated by us
        if (targetNode != "None Selected")
        {
            // get our local rosnode
            auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

            //Selected host is valid, now create action server clients (?)
            baggingStateSub = node->create_subscription<std_msgs::msg::Bool>(
                targetNode + "/bagging_status", rclcpp::SystemDefaultsQoS(), std::bind(&BaggingPanel::baggingStateCallback, this, _1)
            );


            // re enable file selection
            uiPanel->baggingFile->setEnabled(true);
        }
    }

    void BaggingPanel::handleBaggingPanelFile(const QString &text){
        const std::string stdStrFile = text.toStdString();
        if(stdStrFile != "None Selected")
        {
            RVIZ_COMMON_LOG_INFO("BaggingPanel: Bagging file changed to: " + stdStrFile);

            // reset the scroll area
            clearScrollArea();

            // get our local rosnode
            auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

            // start creating the ui elements
            for(size_t i = 0; i < 15; i++){
                auto bagItem = new BagItem(uiPanel->baggingHost->currentText().toStdString(), node, mainParent);
                vbox->addWidget(bagItem);
                bagItem->show();

                bagList.push_back(bagItem);
            }
        }
    }

    void BaggingPanel::baggingStateCallback(const std_msgs::msg::Bool &msg)
    {
        std::vector<int> bids_active = {};

        for(auto item : bagList){
            item->bagAlive(bids_active);
        }
    }

    void BaggingPanel::startBagging()
    {
        uiPanel->baggingStart->setEnabled(false);

        // TODO: Perform service call
    }

    void BaggingPanel::stopBagging()
    {
        uiPanel->baggingStart->setEnabled(false);

        // TODO: Perform service call
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::BaggingPanel, rviz_common::Panel);