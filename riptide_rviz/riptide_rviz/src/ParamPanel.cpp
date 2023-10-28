#include "riptide_rviz/ParamPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <math.h>
#include <QGridLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QString>
#include <QPushButton>
#include <vector>
#include <limits.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace riptide_rviz
{
    ParamPanel::ParamPanel(QWidget *parent) : rviz_common::Panel(parent)
    {

        std::vector<const char*> params = {"test", "test2", "test4", "test5", "test6", "test7", "bigpp"};

        setFocusPolicy(Qt::ClickFocus);

        //ui = new Ui_ParamPanel();
        //ui->setupUi(this);

        paramGrid = new QGridLayout;

        widgetVec.push_back(new QDoubleSpinBox());
        widgetVec.at(0)->setObjectName("increment");
        widgetVec.at(0)->setValue(0.01);
        widgetVec.at(0)->setSingleStep(0.01);
        widgetVec.at(0)->setDecimals(5);

        paramGrid->addWidget(new QLabel("Increment"), 0, 0, 1, 1, Qt::AlignRight);
        paramGrid->addWidget(widgetVec.at(0), 0, 1, 1, 2);


        // Creats and adds label and spinbox for every element in vector
        for(int i = 0; i < (int)params.size(); i++){
            
            // Create Label and SpinBox
            QLabel* label = new QLabel(params.at(i));
            widgetVec.push_back(new QDoubleSpinBox());
            
            // Set attributes of spinbox
            widgetVec.at(i+1)->setMinimum(INT_MIN);
            widgetVec.at(i+1)->setMaximum(INT_MAX);
            widgetVec.at(i+1)->setObjectName(params.at(i));
            widgetVec.at(i+1)->setSingleStep(widgetVec.at(0)->value());
            widgetVec.at(i+1)->setDecimals(5);

            // Add the QStuff to the Grid
            paramGrid->addWidget(label, i+1, 0, 1, 1, Qt::AlignRight);
            paramGrid->addWidget(widgetVec.at(i+1), i+1, 1, 1, 2);

        }

        apply = new QPushButton("&Apply");
        apply->setObjectName("apply");

        paramGrid->addWidget(apply, widgetVec.size(), 0, 1, 3);

        // Sets the layout for the widget to the GridLayout
        setLayout(paramGrid);

    }


    ParamPanel::~ParamPanel()
    {
        delete ui;
    }


    void ParamPanel::load(const rviz_common::Config &config) 
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

    
        RVIZ_COMMON_LOG_INFO("ParamPanel Panel loaded. Robot NS: \"" + robotNs.toStdString() + "\"");
        loaded = true;
    }


    void ParamPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }

    //called when panel is initialized. perform any needed UI connections here
    void ParamPanel::onInitialize()
    {
        connect(apply, &QPushButton::clicked, this, &ParamPanel::buttonPressed);
        connect(widgetVec.at(0), QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ParamPanel::setValue);
    }


    void ParamPanel::buttonPressed()
    {
        for(unsigned int i = 1; i < widgetVec.size(); i++){
            RVIZ_COMMON_LOG_INFO(widgetVec.at(i)->objectName().toStdString());
            RVIZ_COMMON_LOG_INFO(std::to_string(widgetVec.at(i)->value()));
        }
    }

    void ParamPanel::setValue(double val){
        for(unsigned int i = 1; i < widgetVec.size(); i++){
            widgetVec.at(i)->setSingleStep(val);
        }
    }

}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ParamPanel, rviz_common::Panel);
