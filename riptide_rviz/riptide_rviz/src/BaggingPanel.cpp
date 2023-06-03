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

        // set the intial start button state
        uiPanel->baggingStart->setStyleSheet("QPushButton{color:white; background: green;}");
        uiPanel->baggingTrigger->setChecked(true);

        // Connect UI signals for bagging
        connect(uiPanel->baggingRefresh, &QPushButton::clicked, [this](void)
                { hostListRefresh(); fileListRefresh(); clearScrollArea(); });
        connect(uiPanel->baggingStart, &QPushButton::clicked, [this](void)
                { startBagging(); });
        connect(uiPanel->baggingHost, SIGNAL(currentTextChanged(const QString &)), SLOT(handleBaggingPanelHost(const QString &)));
        connect(uiPanel->baggingFile, SIGNAL(currentTextChanged(const QString &)), SLOT(handleBaggingPanelFile(const QString &)));
    }

    void BaggingPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);

        RVIZ_COMMON_LOG_INFO("BaggingPanel: loaded panel");
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
        RVIZ_COMMON_LOG_DEBUG("BaggingPanel: clearing scroll area");

        // if the old stuff exists, remove it
        if (bagList.size() > 0)
        {
            for (auto listObject : bagList)
            {
                delete listObject;
            }

            bagList.clear();
        }

        if (vbox != nullptr)
        {
            RVIZ_COMMON_LOG_DEBUG("BaggingPanels: deleting vbox");

            delete vbox;
        }

        // also clear the scroll area data if it exists
        if (scrollAreaLayout != nullptr)
        {
            RVIZ_COMMON_LOG_DEBUG("BaggingPanel: deleting scroll area");

            delete scrollAreaLayout;
        }

        RVIZ_COMMON_LOG_DEBUG("BaggingPanel: creating new scroll area");

        // Add Vertical Box layout to the Scroll area so we can actually add items to it
        scrollAreaLayout = new QWidget(uiPanel->scrollArea);
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
        { return s.find("_bag_manager") != std::string::npos; };
        std::copy_if(names.begin(), names.end(), std::back_inserter(baggingNodes), filt);

        // stop any events while we refresh the list
        uiPanel->baggingHost->blockSignals(true);

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
        uiPanel->baggingHost->blockSignals(false);
    }

    void BaggingPanel::fileListRefresh()
    {
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
        uiPanel->baggingFile->setEnabled(false);

        // reset the start all synchronization
        runningAll = false;
        priorBagState = std_msgs::msg::Bool();
    }

    void BaggingPanel::handleBaggingPanelHost(const QString &str)
    {
        std::string targetNode = str.toStdString();

        RVIZ_COMMON_LOG_DEBUG("BaggingPanel: selected node " + targetNode);

        // make sure the event wasnt generated by us
        if (targetNode != "None Selected")
        {
            // get our local rosnode
            auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

            // Remove previous ros stuff
            if (baggingStateSub != nullptr)
            {
                baggingStateSub.reset();
            }

            // Selected host is valid, now create action server clients (?)
            baggingStateSub = node->create_subscription<launch_msgs::msg::ListPids>(
                targetNode + "/bag_status", rclcpp::SystemDefaultsQoS(), std::bind(&BaggingPanel::baggingStateCallback, this, _1));

            bagTriggerSub = node->create_subscription<std_msgs::msg::Bool>(
                targetNode + "/autonomy/bag_trigger", rclcpp::SystemDefaultsQoS(), std::bind(&BaggingPanel::autonomyTriggerCallback, this, _1));

            bagWhoIsClient = node->create_client<launch_msgs::srv::WhoIs>(targetNode + "/bag_whois");

            RVIZ_COMMON_LOG_INFO("BaggingPanel: configured node " + targetNode);

            // re enable file selection
            uiPanel->baggingFile->setEnabled(true);
        }
        else
        {
            fileListRefresh();
        }
    }

    void BaggingPanel::handleBaggingPanelFile(const QString &text)
    {
        const std::string stdStrFile = text.toStdString();
        if (stdStrFile != "None Selected")
        {
            RVIZ_COMMON_LOG_DEBUG("BaggingPanel: Bagging file changed to: " + stdStrFile);

            // reset the scroll area
            clearScrollArea();

            // get our local rosnode
            auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

            // hunt down the file location for search
            auto bringupFilesDir = ament_index_cpp::get_package_share_directory(RVIZ_PKG) + "/recipies";

            // parse the bag file
            auto bags = Bag();
            auto errCode = bags.loadXml(bringupFilesDir + "/" + text.toStdString());
            if (errCode.errorCode != RecipeXMLErrorCode::SUCCESS)
            {
                uiPanel->baggingFile->setToolTip("Unable to read bag cfg file. " + QString::fromStdString(getRecipeXMLErrorMessage(errCode)));
                RVIZ_COMMON_LOG_ERROR("BaggingPanels: unable to read cfg file. " + getRecipeXMLErrorMessage(errCode));
            }
            else
            {
                // start creating the ui elements
                for (auto bag : bags.getAllBags())
                {
                    auto bagItem = new BagItem(uiPanel->baggingHost->currentText().toStdString(), bag, node);
                    vbox->addWidget(bagItem);
                    bagItem->show();

                    bagList.push_back(bagItem);
                }

                // now load in the ones from the server
            }
        }
    }

    void BaggingPanel::autonomyTriggerCallback(const std_msgs::msg::Bool &msg)
    {
        if (uiPanel->baggingTrigger->isChecked() && msg.data != priorBagState.data)
            setGlobalBagState(msg.data);

        priorBagState = msg;
    }

    void BaggingPanel::baggingStateCallback(const launch_msgs::msg::ListPids &msg)
    {
        // go through the list of alive pids and make sure we didnt lose one
        std::unordered_set<int> locals;
        for (auto it = bagList.begin(); it != bagList.end();)
        {
            // check if the bag is still alive
            if ((*it)->bagAlive(msg.pids))
            {
                locals.insert((*it)->getPid());
                it++;
            }
            else
            {
                if (!(*it)->local() && !(*it)->isStopping())
                {
                    std::string name;
                    (*it)->getName(name);
                    RVIZ_COMMON_LOG_INFO("BaggingPanels: removing non-local and dead bag " + name);

                    // remove the widget from the vbox
                    vbox->removeWidget(*it);

                    // delete the object
                    delete *it;

                    // erase it from the list
                    it = bagList.erase(it);
                }
                else
                {
                    // otherwise leave it
                    it++;
                }
            }
        }

        // check that we dont have any bags starting currently
        bool notStarting = std::none_of(bagList.begin(), bagList.end(), [](BagItem *item)
                                        { return item->isStarting(); });

        // only make a request if there is not one pending nor anything starting
        if (notStarting && whoisReqId == -1)
        {

            // clear the non-locals
            nonLocals.clear();

            // prepare the request if we need it
            auto req = std::make_shared<launch_msgs::srv::WhoIs::Request>();

            // check for ones we didnt match
            for (auto pid : msg.pids)
            {
                if (locals.find(pid) == locals.end())
                {
                    // if we have one that doesnt match add it to the list to qery info for
                    nonLocals.push_back(pid);
                    req->pids.push_back(pid);
                }
            }

            // check that we have mismatches
            if (nonLocals.size() > 0)
            {

                // send the lookup request over
                auto futWithReq = bagWhoIsClient->async_send_request(req);
                whoisFuture = futWithReq.share();
                whoisReqId = futWithReq.request_id;

                // set a timer to get the results so we dont lock the thread :)
                // clear the timer ticks
                timerTick = 0;

                // set a oneshot timer for 1s into the future and check status
                QTimer::singleShot(250, [this]()
                                   { waitForWhois(); });
            }
        }
    }

    void BaggingPanel::waitForWhois()
    {
        if (!whoisFuture.valid())
        {

            RVIZ_COMMON_LOG_ERROR("BagItem: stop future invalidated while sending request");

            // clear the lookup lockout
            whoisReqId = -1;

            // prevent a re-schedule
            return;
        }

        // check if future has validated
        auto futureStatus = whoisFuture.wait_for(10ms);
        if (futureStatus == std::future_status::timeout && timerTick < 10)
        {
            QTimer::singleShot(250, [this]()
                               { waitForWhois(); });

            timerTick++;
        }
        else if (futureStatus != std::future_status::timeout)
        {
            // future has come back parse the PID information
            auto resp = whoisFuture.get();

            // get our local rosnode
            auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

            // compare with pids sent for lookup and create the addtional elements as running non-local
            // the order should match between the two
            for (size_t i = 0; i < resp->names.size(); i++)
            {
                // make a dumb bag data and chuck the name in
                auto bag = std::make_shared<RecipeBag>();
                bag->name = resp->names.at(i);

                // create the ui element with the running PID
                auto bagItem = new BagItem(uiPanel->baggingHost->currentText().toStdString(), bag, node, nonLocals.at(i));
                vbox->addWidget(bagItem);
                bagItem->show();

                // add it to our bag list
                bagList.push_back(bagItem);
            }

            // clear the lookup lockout
            whoisReqId = -1;
        }
        else if (timerTick == 10)
        {
            RVIZ_COMMON_LOG_ERROR("BagItem: Whois service never responded");

            // remove the failed request
            bagWhoIsClient->remove_pending_request(whoisReqId);
        }
    }

    void BaggingPanel::setGlobalBagState(bool run){
        if (run)
        {
            // we are going to stop them all
            for (auto item : bagList)
            {
                item->stopBagging();
            }

            // set the text back to start
            uiPanel->baggingStart->setText("start");
            uiPanel->baggingStart->setStyleSheet("QPushButton{color:white; background: green;}");
        }
        else
        {
            // we are going to start them all
            for (auto item : bagList)
            {
                item->startBagging();
            }

            // set the text to stop
            uiPanel->baggingStart->setText("stop");
            uiPanel->baggingStart->setStyleSheet("QPushButton{color:black; background: red;}");
        }
    }

    void BaggingPanel::startBagging()
    {
        setGlobalBagState(runningAll);
        runningAll = !runningAll;
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::BaggingPanel, rviz_common::Panel);