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
        delete scrollAreaLayout;
        createScrollArea();
    }

    void Bringup::bringupListRefresh()
    {
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
            uiPanel->bringupStart->setDisabled(false);
        }
        else
        {
            uiPanel->bringupHost->addItem("None");
            uiPanel->bringupStart->setDisabled(true);
        }

        // clear the list of bringup files
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
    }

    void Bringup::bringupFileChanged(const QString &text)
    {
        clearScrollArea();
        const std::string stdStrFile = text.toStdString();
        if(stdStrFile != "None Selected")
        {
            RVIZ_COMMON_LOG_INFO("Bringup file changed to: " + stdStrFile);
            Recipe recipe;
            auto recipeError = recipe.loadXml(bringupFilesDir + "/" + stdStrFile);
            if (recipeError.errorCode == RecipeXMLErrorCode::SUCCESS)
            {
                auto launchList = recipe.getAllLaunches();
                std::string hostName = uiPanel->bringupHost->currentText().toStdString();
                if(hostName != "None Selected")
                {
                    for(auto launch : launchList)
                    {
                        riptide_rviz::BringupClient *launchClient = new riptide_rviz::BringupClient(hostName, clientNode, launch, vbox);
                        clientList.push_back(launchClient);
                    }
                    RVIZ_COMMON_LOG_INFO("Loaded XML Successfully");
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
        // make sure that the bringup file is selected
        std::string targetFile = uiPanel->bringupFile->currentText().toStdString();

        // std::shared_ptr<riptide_rviz::RecipeLaunch> recipeLaunch = std::make_shared<riptide_rviz::RecipeLaunch>();

        // RecipeTopicData t1;
        // t1.name = "/joint_states";
        // t1.type_name = "sensor_msgs/msg/JointState";
        // t1.qos_type = "sensor_data";

        // recipeLaunch->name = "dummy_robot_bringup.launch.py";
        // recipeLaunch->package = "dummy_robot_bringup";
        // recipeLaunch->topicList.push_back(t1);


        // std::string hostName = uiPanel->bringupHost->currentText().toStdString();
        // if(hostName != "None Selected")
        // {
        //     riptide_rviz::BringupClient *launchClient = new riptide_rviz::BringupClient(hostName, clientNode, recipeLaunch, vbox);
        //     clientList.push_back(launchClient);
        // }
        
        // validate selection
        if (targetFile != "None" && targetFile != "None Selected")
        {
            // disable the start button and enable the stop button
            //uiPanel->bringupStart->setDisabled(true);
            uiPanel->bringupRefresh->setDisabled(true);

            
            // launch_msgs::srv::StartLaunch::Request::SharedPtr startReq = std::make_shared<launch_msgs::srv::StartLaunch::Request>();
            // startReq->launch_file = targetFile;
            // startReq->package = BRINGUP_PKG;

            // // check client not busy
            // while (!bringupStartClient->wait_for_service(100ms))
            //     if (!rclcpp::ok())
            //         return;

            // // make the request and wait for the future to be valid
            // auto bringupFuture = bringupStartClient->async_send_request(startReq);
            // if (rclcpp::spin_until_future_complete(clientNode, bringupFuture, 1s) == rclcpp::FutureReturnCode::SUCCESS)
            // {

            //     auto result = bringupFuture.get();
            //     if (result->started)
            //     {
            //         // record the bringup id
            //         bringupID = result->formed_launch.launch_id;

            //         // start the check timer to make sure the service is still alive
            //         bringupCheckTimer->start(BRINGUP_POLLING_RATE);

            //         // bail early if everything went okay
            //         return;
            //     }
            // }

            // bringupID = -1;

            // reset the buttons durig an error
            uiPanel->bringupStart->setDisabled(false);
            uiPanel->bringupRefresh->setDisabled(false);
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