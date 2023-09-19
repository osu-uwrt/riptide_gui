#include "riptide_rviz/FeedforwardPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <math.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace riptide_rviz
{
    FeedforwardPanel::FeedforwardPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_FeedforwardPanel();
        ui->setupUi(this);
    }


    FeedforwardPanel::~FeedforwardPanel()
    {
        delete ui;
    }


    void FeedforwardPanel::load(const rviz_common::Config &config) 
    {
        config.mapGetString("robot_namespace", &robotNs);
        if(robotNs == "")
        {
            robotNs = QString::fromStdString("/talos"); 
            RVIZ_COMMON_LOG_WARNING("FeedforwardPanel: Using /talos as the default value for robot_namespace"); 
        }

        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        loaded = true;
    }


    void FeedforwardPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }

    //called when panel is initialized. perform any needed UI connections here
    void FeedforwardPanel::onInitialize()
    {
        counter = 0;
        connect(ui->someButton, &QPushButton::clicked, this, &FeedforwardPanel::someButtonClicked);
    }

    //called when someButton is clicked
    void FeedforwardPanel::someButtonClicked()
    {
        counter++;
        ui->someLabel->setText(tr("hello world! : %1").arg(counter));
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::FeedforwardPanel, rviz_common::Panel);
