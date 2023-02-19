#include "riptide_rviz/BringupClient.hpp"

namespace riptide_rviz
{
    BringupClient::BringupClient(std::string hostName, std::shared_ptr<rclcpp::Node> parentNode, std::shared_ptr<riptide_rviz::RecipeLaunch> recipeLaunch, QVBoxLayout *parent)
    {
        listElement = new Ui_BringupListElement();
        listElement->setupUi(this);
        parent->addWidget(this);

        //Create two action clients one for bringup start, one for bringup stop

        //Create two callbacks for start and stop buttons
    }

    BringupClient::~BringupClient()
    {
        delete listElement;
    }

    void BringupClient::checkPids(launch_msgs::msg::ListLaunch launchMsgs)
    {

    }
}