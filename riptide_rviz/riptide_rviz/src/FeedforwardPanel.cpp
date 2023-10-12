#include "riptide_rviz/FeedforwardPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <math.h>
#include <QDoubleSpinBox>

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
        connect(ui->XdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::XFunction);
        connect(ui->YdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::YFunction);
        connect(ui->ZdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::ZFunction);
        connect(ui->RdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::RFunction);
        connect(ui->PdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::PFunction);
        connect(ui->YAWdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::YAWFunction);
    }



    //called when someButton is clicked
    void FeedforwardPanel::someButtonClicked()
    {
        counter++;
        ui->someLabel->setText(tr("hello world! : %1").arg(counter));
    }

    //called when the doubleSpinBoxes are interacted with
    void FeedforwardPanel::XFunction(double value)
    {
        ui->Xlabel->setText(QString::number(value));
    }
    void FeedforwardPanel::YFunction(double value)
    {
        ui->Ylabel->setText(QString::number(value));
    }
    void FeedforwardPanel::ZFunction(double value)
    {
        ui->Zlabel->setText(QString::number(value));
    }
    void FeedforwardPanel::RFunction(double value)
    {
        ui->Rlabel->setText(QString::number(value));
    }
    void FeedforwardPanel::PFunction(double value)
    {
        ui->Plabel->setText(QString::number(value));
    }
    void FeedforwardPanel::YAWFunction(double value)
    {
        ui->YAWlabel->setText(QString::number(value));
    }


    
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::FeedforwardPanel, rviz_common::Panel);
