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

        //create ROS peripherals
        //we want a publisher to publish to controller/ff_body_force to command the feedforward of the solver
        //we also want a timer to periodcally publish that message using the publisher
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        std::string fullTopicName = robotNs.toStdString() + "/controller/FF_body_force";
        ffForcePub = node->create_publisher<geometry_msgs::msg::Twist>(fullTopicName, 10);
        pubTimer = node->create_wall_timer(20ms, std::bind(&FeedforwardPanel::timerCb, this));
        pubTimer->cancel(); //dont start publishing right away

        RVIZ_COMMON_LOG_INFO("FeedForward Panel loaded. Robot NS: \"" + robotNs.toStdString() + "\"");
        
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
        connect(ui->XdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::XFunction);
        connect(ui->YdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::YFunction);
        connect(ui->ZdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::ZFunction);
        connect(ui->RdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::RFunction);
        connect(ui->PdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::PFunction);
        connect(ui->YAWdoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FeedforwardPanel::YAWFunction);
        connect(ui->publishButton, &QPushButton::clicked, this, &FeedforwardPanel::togglePublish);

    }

    
    //called when the doubleSpinBoxes are interacted with
    void FeedforwardPanel::XFunction(double value)
    {
        ui->Xlabel->setText(tr("%1 N").arg(value));
        msgToPublish.linear.x = value;
    }

    void FeedforwardPanel::YFunction(double value)
    {
        ui->Ylabel->setText(tr("%1 N").arg(value));
        msgToPublish.linear.y = value;
    }

    void FeedforwardPanel::ZFunction(double value)
    {
        ui->Zlabel->setText(tr("%1 N").arg(value));
        msgToPublish.linear.z = value;

        // Volume calculations
        double density = 1000;
        double g = 9.8;
        // Finds mass from talos.yaml file
        double mass = config["mass"].as<double>(); 
        // Takes z-value from the doublespin box in rviz
        double Zforce = ui->ZdoubleSpinBox->value();
        // net Force = 0 = buoyancyForce = mg + Zforce -> density*g*volume = mg + Zforce. So solving for volume...
        double volume = (mass*g + (Zforce))/(density*g);
        ui->VolumeOutput->setText(tr("%1 L").arg(volume));

        // Buoyant force calculations
        double buoyancyForce = mass*g + Zforce;
        // Can also use: buoyancyForce = density * g * V;
        ui->BuoyancyOutput->setText(tr("%1 N").arg(buoyancyForce));
    
    }

    void FeedforwardPanel::RFunction(double value)
    {
        ui->Rlabel->setText(tr("%1 N").arg(value));
        msgToPublish.angular.x = value;
    }

    void FeedforwardPanel::PFunction(double value)
    {
        ui->Plabel->setText(tr("%1 N").arg(value));
        msgToPublish.angular.y = value;
    }

    void FeedforwardPanel::YAWFunction(double value)
    {
        ui->YAWlabel->setText(tr("%1 N").arg(value));
        msgToPublish.angular.z = value;
    }

    void FeedforwardPanel::togglePublish()
    {
        publishing = !publishing;
        if(publishing)
        {
            ui->publishButton->setText("Stop Publishing");
            pubTimer->reset();
        } else
        {
            ui->publishButton->setText("Start Publishing");
            pubTimer->cancel();
        }
    }

    void FeedforwardPanel::timerCb()
    {
        ffForcePub->publish(msgToPublish);
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::FeedforwardPanel, rviz_common::Panel);
