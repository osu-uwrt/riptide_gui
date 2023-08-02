#include "riptide_rviz/Bringup.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>
#include <string>
#include <cstring>
#include <system_error>
#include <unistd.h>
#include <sys/wait.h>
#include <QVBoxLayout>
#include <QMessageBox>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

#include "riptide_rviz/bringup_recipe.hpp"
#include "riptide_rviz/BringupClient.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

#define SSH_USERNAME "ros"
#define SSH_COMMAND "sudo systemctl restart ros2_launch_service_c.service"

bool issueRemoteLaunchServiceRestart(std::string const& hostname, std::string &messageOut);

namespace riptide_rviz
{
    Bringup::Bringup(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);
        mainParent = parent;

        uiPanel = new Ui_Bringup();
        uiPanel->setupUi(this);

        recipe = std::make_shared<riptide_rviz::Recipe>();
    }

    void Bringup::onInitialize()
    {
        RVIZ_COMMON_LOG_INFO("BringupPanel: Initialzing");

        // refresh UI elements so they start displayed correctly
        bringupListRefresh();

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
        return false;
    }

    Bringup::~Bringup()
    {
        // master window control removal
        delete uiPanel;

        // free all BringupClients
        for (auto element : clientList)
        {
            delete element;
        }
    }

    void Bringup::clearScrollArea()
    {
        RVIZ_COMMON_LOG_INFO("BringupPanel: clearing scroll area");

        // if the old stuff exists, remove it
        if(clientList.size() > 0){
            for(auto listObject : clientList)
            {
                delete listObject;
            }

            clientList.clear();
        }

        if(vbox != nullptr){
            RVIZ_COMMON_LOG_INFO("BringupPanel: deleting vbox");
            delete vbox;
        }
        
        // also clear the scroll area data if it exists
        if(scrollAreaLayout != nullptr){
            RVIZ_COMMON_LOG_INFO("BringupPanel: deleting scroll area");
            delete scrollAreaLayout;
        }

        RVIZ_COMMON_LOG_INFO("BringupPanel: creating new scroll area");
        
        // Add Vertical Box layout to the Scroll area so we can actually add items to it
        scrollAreaLayout = new QWidget(mainParent);
        vbox = new QVBoxLayout(scrollAreaLayout);
        vbox->setAlignment(Qt::AlignTop);
        vbox->setSpacing(0);
        scrollAreaLayout->setLayout(vbox);
        uiPanel->scrollArea->setWidget(scrollAreaLayout);
    }

    void Bringup::bringupListRefresh()
    {
        clearScrollArea();

        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // get the list of nodes available
        std::vector<std::string> names = node->get_node_names();
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
            RVIZ_COMMON_LOG_INFO("BringupPanel: Bringup file changed to: " + stdStrFile);
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
                    // get our local rosnode
                    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

                    for(auto launch : launchList)
                    {
                        riptide_rviz::BringupClient *launchClient = new riptide_rviz::BringupClient(hostName, node, launch, vbox, mainParent);
                        clientList.push_back(launchClient);
                    }
                    RVIZ_COMMON_LOG_INFO("BringupPanel: Loaded XML Successfully");
                    RVIZ_COMMON_LOG_INFO("BringupPanel: Printing launch sequence");
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
                RVIZ_COMMON_LOG_ERROR_STREAM("BringupPanel: Failed to load XML into Recipe: \n" << getRecipeXMLErrorMessage(recipeError));
            }
        }
    }
    
    void Bringup::restartService()
    {
        RVIZ_COMMON_LOG_INFO("BringupPanel: Remotely restarting launch service");
        std::string
            hostname = uiPanel->bringupHost->itemText(uiPanel->bringupHost->currentIndex()).toStdString(),
            outMsg;
        
        bool success = issueRemoteLaunchServiceRestart(hostname, outMsg);
        if(!success)
        {
            QMessageBox::warning(this, "Error", QString::fromStdString(outMsg));
        }
    }

    void Bringup::handleBringupHost(int selection)
    {
        std::string targetNode = uiPanel->bringupHost->itemText(selection).toStdString();

        // make sure the event wasnt generated by us
        if (targetNode != "None Selected")
        {
            // get our local rosnode
            auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

            //Selected host is valid, now create action server clients (?)
            listLaunchSub = node->create_subscription<launch_msgs::msg::ListPids>(
                targetNode + "/launch_status", rclcpp::SystemDefaultsQoS(), std::bind(&Bringup::listLaunchCallback, this, _1)
            );
        }
    }

    void Bringup::listLaunchCallback(const launch_msgs::msg::ListPids &msg)
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
            RVIZ_COMMON_LOG_INFO("BringupPanel: Staged Launch Started");
            uiPanel->bringupStart->setDisabled(true);
            totalStages = recipe->getLaunchOrder().size();
            stagedTimer = new QTimer(this);
            connect(stagedTimer, &QTimer::timeout, [this](void) { Bringup::stagedBringupTick(); });
            stagedTimer->setInterval(500);
            RVIZ_COMMON_LOG_INFO("BringupPanel: Starting timer");
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
                    RVIZ_COMMON_LOG_INFO("BringupPanel: Launching launch file");
                    clientList.at(launchFileIndex)->startButtonCallback();
                    complete = false;
                }
                else
                {
                    RVIZ_COMMON_LOG_INFO("BringupPanel: Checking launch file");
                    auto client = clientList.at(launchFileIndex);
                    if(client->hasError())
                    {
                        stagedTimer->stop();
                        delete stagedTimer;
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
                    delete stagedTimer;
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


bool issueRemoteLaunchServiceRestart(std::string const& hostname, std::string &messageOut) {
        std::string destination = SSH_USERNAME "@" + hostname;

        pid_t child = fork();
        if (!child) {
            // You're a child
            execlp("ssh", "ssh", "-n", destination.c_str(), SSH_COMMAND, NULL);
            // If this fails, then the fork failed
            fprintf(stderr, "Failed to execlp: %s\n", strerror(errno));
            exit(255);
        }
        else if (child < 0) {
            messageOut = std::string("Error during fork: ") + strerror(errno);
            return false;
        }

        int ssh_status;
        waitpid(child, &ssh_status, 0);
        if (WIFEXITED(ssh_status)) {
            int exit_code = WEXITSTATUS(ssh_status);
            if (exit_code == 0) {
                return true;
            }
            else {
                messageOut = "Returned with non-zero exit code " + std::to_string(exit_code);
                return false;
            }
        }
        else if (WIFSIGNALED(ssh_status)) {
            messageOut = "Exited due to signal " + std::to_string(WTERMSIG(ssh_status));
            return false;
        }
        else {
            messageOut = "Unknown exit reason with status " + std::to_string(ssh_status);
            return false;
        }
    }

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::Bringup, rviz_common::Panel);