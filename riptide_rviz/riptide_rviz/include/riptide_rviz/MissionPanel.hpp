#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <riptide_msgs2/srv/list_trees.hpp>
#include <riptide_msgs2/action/execute_tree.hpp>
#include <riptide_msgs2/msg/tree_stack.hpp>
#include <riptide_msgs2/msg/led_command.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <QTimer>
#include <QStandardItemModel>

#include "ui_MissionPanel.h"

namespace riptide_rviz
{
    using ExecuteTree = riptide_msgs2::action::ExecuteTree;
    using GHExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

    class MissionPanel : public rviz_common::Panel
    {
        Q_OBJECT public : MissionPanel(QWidget *parent = 0);
        ~MissionPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;
    protected Q_SLOTS:
        // QT slots (function callbacks)
        void refresh();
        void handleSelectTree(int);

        void startTask();
        void cancelTask();
        

    protected:
        bool event(QEvent *event);

        // action server callbacks
        void taskStartCb(const GHExecuteTree::SharedPtr & goalHandle);
        void cancelAccept(const action_msgs::srv::CancelGoal::Response::SharedPtr );
        void taskCompleteCb(const GHExecuteTree::WrappedResult & result);
        void taskFeedbackCb(GHExecuteTree::SharedPtr goalHandle,
                            ExecuteTree::Feedback::ConstSharedPtr feedback);

        //subscriber callback on the tree stack
        void stackCb(const riptide_msgs2::msg::TreeStack & stack);

        //subscriber callback for led commands
        void ledCb(const riptide_msgs2::msg::LedCommand & cmd);

        // timer callabck for refresh future
        void waitForRefresh(void);

    private:
        void updateLedReadout(void);

        Ui_MissionPanel *uiPanel;

        // tree view item model
        QStandardItemModel * model;

        // stack
        std::vector<std::string> treeList;

        // led command
        riptide_msgs2::msg::LedCommand lastLedCommand;
        rclcpp::TimerBase::SharedPtr ledTimer;

        rclcpp::Subscription<riptide_msgs2::msg::TreeStack>::SharedPtr stackSub;
        rclcpp::Subscription<riptide_msgs2::msg::LedCommand>::SharedPtr ledSub;

        // refresh request info
        rclcpp::Client<riptide_msgs2::srv::ListTrees>::SharedPtr refreshClient;
        std::shared_future<std::shared_ptr<riptide_msgs2::srv::ListTrees_Response>> refreshFuture;
        int64_t refreshFutureid = -1;

        int timerTick = -1;

        rclcpp_action::Client<ExecuteTree>::SharedPtr actionServer;
        
        std::string robot_ns;
    };

} // namespace riptide_rviz