#include "riptide_rviz/MissionPanel.hpp"
#include <chrono>
#include <algorithm>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace riptide_rviz
{
    MissionPanel::MissionPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_MissionPanel();
        uiPanel->setupUi(this);

        treeList = std::vector<std::string>();
    }

    void MissionPanel::onInitialize()
    {
        // create the item model and attach it to the panel
        model = new QStandardItemModel();
        uiPanel->btStackView->setModel(model);

        // Connect UI signals for controlling the riptide vehicle
        connect(uiPanel->btSelect, SIGNAL(currentIndexChanged(int)), SLOT(handleSelectTree(int)));
        connect(uiPanel->btStart, &QPushButton::clicked, [this](void)
                { startTask(); });
        connect(uiPanel->btStop, &QPushButton::clicked, [this](void)
                { cancelTask(); });
        connect(uiPanel->btRefresh, &QPushButton::clicked, [this](void)
                { refresh(); });
    }

    void MissionPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);

        RVIZ_COMMON_LOG_INFO("MissionPanel: Loaded parent panel config");

        // create our value containers
        QString * str = new QString();
        float * configVal = new float();

        // load the namesapce param
        if(config.mapGetString("robot_namespace", str)){
            robot_ns = str->toStdString();
        } else {
            // default value
            robot_ns = "/talos";
            RVIZ_COMMON_LOG_WARNING("MissionPanel: Loading default value for 'namespace'");
        }

        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        actionServer = rclcpp_action::create_client<ExecuteTree>(node, robot_ns + "/autonomy/run_tree");
    
        stackSub = node->create_subscription<riptide_msgs2::msg::TreeStack>(
            robot_ns + "/autonomy/tree_stack", rclcpp::SystemDefaultsQoS(),
            std::bind(&MissionPanel::stackCb, this, _1)
        );

        refreshClient = node->create_client<riptide_msgs2::srv::ListTrees>(robot_ns + "/autonomy/list_trees");

        // refresh the UI
        refresh();
    }

    void MissionPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        // write our config values
        config.mapSetValue("robot_namespace", QString::fromStdString(robot_ns));
    }

    bool MissionPanel::event(QEvent *event)
    {
    }

    MissionPanel::~MissionPanel()
    {
        // master window control removal
        delete uiPanel;

        delete model;
    }

    void MissionPanel::refresh()
    {
        // reset the GUI in case of tree crash
        if(!uiPanel->btStart->isEnabled()){
            uiPanel->btStart->setEnabled(true);
            uiPanel->btStop->setEnabled(false);
        }

        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        riptide_msgs2::srv::ListTrees::Request::SharedPtr startReq = std::make_shared<riptide_msgs2::srv::ListTrees::Request>();
        auto start = node->get_clock()->now();
        while (!refreshClient->wait_for_service(100ms))
            if (node->get_clock()->now() - start > 1s || !rclcpp::ok())
                return;

        auto refreshFuture = refreshClient->async_send_request(startReq);
        if (rclcpp::spin_until_future_complete(node, refreshFuture, 1s) == rclcpp::FutureReturnCode::SUCCESS)
        {

            auto response = refreshFuture.get();

            uiPanel->btSelect->clear();

            uiPanel->btSelect->addItem("None Selected");

            // keep a list of the valid trees
            treeList.clear();
            treeList.insert(treeList.end(), response->trees.begin(), response->trees.end());

            for (auto tree : response->trees)
            {
                // push these into the combo box
                uiPanel->btSelect->addItem(QString::fromStdString(tree));
            }
        }
    }

    void MissionPanel::handleSelectTree(int selection){
        std::string treeFile = uiPanel->btSelect->itemText(selection).toStdString();

        bool found = std::find(treeList.begin(), treeList.end(), treeFile) != treeList.end();
        if(found){
            // enable the start button
            uiPanel->btStart->setEnabled(true);
        } else {
            uiPanel->btStart->setEnabled(false);
        }
    }

    void MissionPanel::startTask()
    {
        // disable the start button
        uiPanel->btStart->setEnabled(false);

        // get our local rosnode
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        auto start = node->get_clock()->now();
        while (!actionServer->wait_for_action_server(100ms))
            if (node->get_clock()->now() - start > 1s || !rclcpp::ok())
            {
                // we have timed out waiting, re-enable the button
                uiPanel->btStart->setEnabled(true);
                return;
            }

        auto goal = ExecuteTree::Goal();
        goal.tree = uiPanel->btSelect->currentText().toStdString();

        // create the goal callbacks to bind to
        auto sendGoalOptions = rclcpp_action::Client<ExecuteTree>::SendGoalOptions();
        sendGoalOptions.goal_response_callback =
            std::bind(&MissionPanel::taskStartCb, this, _1);
        sendGoalOptions.feedback_callback =
            std::bind(&MissionPanel::taskFeedbackCb, this, _1, _2);
        sendGoalOptions.result_callback =
            std::bind(&MissionPanel::taskCompleteCb, this, _1);

        // send the goal with the callbacks configured
        actionServer->async_send_goal(goal, sendGoalOptions);
    }

    void MissionPanel::taskStartCb(const GHExecuteTree::SharedPtr &goalHandle)
    {
        if (!goalHandle)
        {
            // the server did not accept
            uiPanel->btStart->setEnabled(true);
        }
        else
        {
            // the server accepted, enable the cancel
            uiPanel->btStop->setEnabled(true);
        }
    }

    void MissionPanel::taskCompleteCb(const GHExecuteTree::WrappedResult &result)
    {
        // we can re-enable the staret button and disable the stop button
        uiPanel->btStart->setEnabled(true);
        uiPanel->btStop->setEnabled(false);

        // if the completion was unintended or , change the stylesheet of the stack view
        if(result.code == rclcpp_action::ResultCode::ABORTED){
            // set the red stylesheet
            uiPanel->btStackView->setStyleSheet("QTreeView{background: rgb(200, 100, 100)}");

            // create a timer to clear it in 1 second
            QTimer::singleShot(1000, [this](void)
                               { uiPanel->btStackView->setStyleSheet(""); });
        }
    }

    void MissionPanel::taskFeedbackCb(GHExecuteTree::SharedPtr goalHandle,
                                      ExecuteTree::Feedback::ConstSharedPtr feedback)
    {
        //TODO figure out how to render the stack
    }

    void MissionPanel::cancelAccept(const action_msgs::srv::CancelGoal::Response::SharedPtr resp){
        // anything we want to do when the cancels are accepted
    }

    void MissionPanel::stackCb(const riptide_msgs2::msg::TreeStack & msg ){
        
        // clean the tree
        model->clear();

        // get the root again
        QStandardItem* parentItem = model->invisibleRootItem();

        // add the new tree
        for(auto name : msg.stack){
            QStandardItem * item = new QStandardItem(QString::fromStdString(name));
            parentItem->appendRow(item);
            parentItem = item;
        }

        uiPanel->btStackView->expandAll();
    }

    void MissionPanel::cancelTask()
    {
        actionServer->async_cancel_all_goals(std::bind(&MissionPanel::cancelAccept, this, _1));
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::MissionPanel, rviz_common::Panel);