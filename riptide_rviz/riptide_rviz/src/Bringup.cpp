#include "riptide_rviz/Bringup.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>
#include <string>
#include <rviz_common/display_context.hpp>
#include <QVBoxLayout>
#include <rviz_common/logging.hpp>

#include "riptide_rviz/bringup_recipe.hpp"
#include "riptide_rviz/BringupClient.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

QVBoxLayout *vbox;

namespace riptide_rviz
{
    Bringup::Bringup(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);
        mainParent = parent;

        uiPanel = new Ui_Bringup();
        uiPanel->setupUi(this);

        auto options = rclcpp::NodeOptions().arguments({});
        clientNode = std::make_shared<rclcpp::Node>("riptide_rviz_bringup", options);

        createScrollArea();
        recipe = std::make_shared<riptide_rviz::Recipe>();
    }

    void Bringup::onInitialize()
    {
        // refresh UI elements so they start displayed correctly
        bringupListRefresh();

        // create a spin timer
        spinTimer = new QTimer(this);
        connect(spinTimer, &QTimer::timeout, [this](void)
                { rclcpp::spin_some(clientNode); });
        spinTimer->start(50);

        uiPanel->bringupStart->setDisabled(true);

        // Connect UI signals for bringup
        connect(uiPanel->bringupRefresh, &QPushButton::clicked, [this](void)
                { bringupListRefresh(); });
        connect(uiPanel->bringupStart, &QPushButton::clicked, [this](void)
                { startBringup(); });
        connect(uiPanel->bringupHost, SIGNAL(currentIndexChanged(int)), SLOT(handleBringupHost(int)));
        connect(uiPanel->bringupFile, SIGNAL(currentTextChanged(const QString &)), SLOT(bringupFileChanged(const QString &)));
    }

    void Bringup::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }

    void Bringup::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
    }

    bool Bringup::event(QEvent *event)
    {
    }

    Bringup::~Bringup()
    {
        // master window control removal
        delete uiPanel;

        // remove the timers
        delete spinTimer;

        // free all BringupClients
        for (auto element : clientList)
        {
            delete element;
        }
    }

    void Bringup::createScrollArea()
    {
        // Add Vertical Box layout to the Scroll area so we can actually add items to it
        scrollAreaLayout = new QWidget(mainParent);
        vbox = new QVBoxLayout(scrollAreaLayout);
        vbox->setAlignment(Qt::AlignTop);
        vbox->setSpacing(0);
        scrollAreaLayout->setLayout(vbox);
        uiPanel->scrollArea->setWidget(scrollAreaLayout);
    }

    void Bringup::clearScrollArea()
    {
        for(auto listObject : clientList)
        {
            delete listObject;
        }
        clientList.clear();

        delete scrollAreaLayout;
        createScrollArea();
    }

    void Bringup::bringupListRefresh()
    {
        clearScrollArea();
        // get the list of nodes available
        std::vector<std::string> names = clientNode->get_node_names();
        // clear the entries in the combo boxes
        uiPanel->bringupHost->clear();

        // filter names down to launch nodes
        std::vector<std::string> launchNodes;
        auto filt = [](const auto &s)
        { return s.find("_launch_manager") != std::string::npos; };
        std::copy_if(names.begin(), names.end(), std::back_inserter(launchNodes), filt);

        // make sure we have availiable nodes in the list, otherwise place a blank list entry
        if (launchNodes.size() > 0)
        {
            uiPanel->bringupHost->addItem("None Selected");
            for (std::string const &name : launchNodes)
            {
                // push these into the combo box
                uiPanel->bringupHost->addItem(QString::fromStdString(name.substr(0, name.find_last_of('l') - 1)));
            }
            //Disable while ordered start does is not implemented
            //TODO remove when staggered launch is working
            uiPanel->bringupStart->setEnabled(true);
        }
        else
        {
            uiPanel->bringupHost->addItem("None");
        }

        // clear the list of bringup files
        uiPanel->bringupFile->blockSignals(true);
        uiPanel->bringupFile->clear();
        uiPanel->bringupFile->addItem("None Selected");
        
        // populate with new files
        bringupFilesDir = ament_index_cpp::get_package_share_directory(RVIZ_PKG) + "/recipies";
        for (const auto &entry : std::filesystem::directory_iterator(bringupFilesDir))
        {
            std::string file = std::string(entry.path());
            if (file.find(".xml") != std::string::npos)
            {
                uiPanel->bringupFile->addItem(QString::fromStdString(file.substr(file.find_last_of('/') + 1)));
            }
        }
        uiPanel->bringupFile->blockSignals(false);

        uiPanel->bringupStart->setDisabled(true);
    }

    void Bringup::bringupFileChanged(const QString &text)
    {
        clearScrollArea();
        const std::string stdStrFile = text.toStdString();
        if(stdStrFile != "None Selected")
        {
            RVIZ_COMMON_LOG_INFO("Bringup file changed to: " + stdStrFile);
            recipe.reset();
            recipe = std::make_shared<riptide_rviz::Recipe>();
            auto recipeError = recipe->loadXml(bringupFilesDir + "/" + stdStrFile);
            if (recipeError.errorCode == RecipeXMLErrorCode::SUCCESS)
            {
                uiPanel->bringupStart->setEnabled(true);
                auto launchList = recipe->getAllLaunches();
                std::string hostName = uiPanel->bringupHost->currentText().toStdString();
                if(hostName != "None Selected")
                {
                    for(auto launch : launchList)
                    {
                        riptide_rviz::BringupClient *launchClient = new riptide_rviz::BringupClient(hostName, clientNode, launch, vbox, mainParent);
                        clientList.push_back(launchClient);
                    }
                    RVIZ_COMMON_LOG_INFO("Loaded XML Successfully");
                    RVIZ_COMMON_LOG_INFO("Printing launch sequence");
                    for(auto stage : recipe->getLaunchOrder())
                    {
                        for (auto topic : stage)
                        {
                            std::string temp = std::to_string(topic) + launchList[topic]->name;
                            RVIZ_COMMON_LOG_INFO(temp);
                        }
                    }
                }
            }
            else
            {
                RVIZ_COMMON_LOG_ERROR("Failed to load XML into Recipe");
                RVIZ_COMMON_LOG_ERROR(getRecipeXMLErrorMessage(recipeError));
            }
        }
    }

    void Bringup::handleBringupHost(int selection)
    {
        std::string targetNode = uiPanel->bringupHost->itemText(selection).toStdString();

        // make sure the event wasnt generated by us
        if (targetNode != "None Selected")
        {
            //Selected host is valid, now create action server clients (?)
            listLaunchSub = clientNode->create_subscription<launch_msgs::msg::ListLaunch>(
                targetNode + "/launch_status", rclcpp::SystemDefaultsQoS(), std::bind(&Bringup::listLaunchCallback, this, _1)
            );
        }
    }

    void Bringup::listLaunchCallback(const launch_msgs::msg::ListLaunch &msg)
    {
        for(auto client : clientList)
        {
            client->checkPids(msg);
        }
    }

    void Bringup::startBringup()
    {
        // make sure that a bringup file is selected
        std::string targetFile = uiPanel->bringupFile->currentText().toStdString();
        if (targetFile != "None" && targetFile != "None Selected")
        {
            stage = 0;
            RVIZ_COMMON_LOG_INFO("Staged Launch Started");
            uiPanel->bringupStart->setDisabled(true);
            totalStages = recipe->getLaunchOrder().size();
            stagedTimer = new QTimer(this);
            connect(stagedTimer, &QTimer::timeout, [this](void) { Bringup::stagedBringupTick(); });
            stagedTimer->setInterval(500);
            RVIZ_COMMON_LOG_INFO("Starting timer");
            stagedTimer->start();
        }
    }

    void Bringup::stagedBringupTick()
    {
            bool complete = true;
            auto launchFileIndexes = recipe->getLaunchOrder().at(stage);
            for(auto launchFileIndex : launchFileIndexes)
            {
                if(startTick)
                {
                    RVIZ_COMMON_LOG_INFO("Launching launch file");
                    clientList.at(launchFileIndex)->startButtonCallback();
                    complete = false;
                }
                else
                {
                    RVIZ_COMMON_LOG_INFO("Checking launch file");
                    auto client = clientList.at(launchFileIndex);
                    if(client->hasError())
                    {
                        stagedTimer->stop();
                    }
                    complete = complete && client->complete();
                }
            }

            startTick = false;

            if(complete)
            {
                stage++;
                startTick = true;
                if(stage == totalStages)
                {
                    //uiPanel->bringupStart->setEnabled(true);
                    stage = 0;
                    stagedTimer->stop();
                }
            }
    }

    void Bringup::checkBringupStatus()
    {
        //Modify to check if bringups are still alive
        //Callback function for a subscriber subscribed to pid alive topic ListLaunch
        // for (auto client : clientList)
        // {    
        //     client->checkPids();
        // }
    }

    void Bringup::stopBringup()
    {
        
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::Bringup, rviz_common::Panel);