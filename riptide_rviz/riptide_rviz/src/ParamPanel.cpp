#include "riptide_rviz/ParamPanel.hpp"
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <math.h>
#include <QGridLayout>
#include <QLabel>
#include <QLayoutItem>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QString>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include <QPlainTextEdit>
#include <vector>
#include <limits.h>
#include <QScrollArea>
#include <float.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace riptide_rviz
{
    ParamPanel::ParamPanel(QWidget *parent) : rviz_common::Panel(parent)
    {

        setFocusPolicy(Qt::ClickFocus);

        // Create a combo box to select node to edit
        this->nodes = new QComboBox();

        // Will hold the parameters that are being edited
        this->paramLayout = new QVBoxLayout();
        
        // Adds a button to refresh params
        this->refreshButton = new QPushButton("&Refresh");
        this->refreshButton->setObjectName("refresh");

        // Adds a button to apply params
        this->applyButton = new QPushButton("&Apply");
        this->applyButton->setObjectName("apply");

        // Main Layout that will hold everything
        QVBoxLayout* mainLayout = new QVBoxLayout();

        // Holds the Node selector combo box
        QHBoxLayout* nodeBox = new QHBoxLayout();
        QLabel* nodeLabel = new QLabel("Node:");
        nodeLabel->setMaximumWidth(nodeLabel->fontMetrics().horizontalAdvance("Node:"));
        nodeBox->addWidget(nodeLabel);
        nodeBox->addWidget(this->nodes);

        // Adds nodeBox to the Main Layout
        mainLayout->addLayout(nodeBox);

        // Add paramLayout to main layout
        // Widgets will be added to paramLayout after ros node is loaded
        mainLayout->addLayout(this->paramLayout);

        // Create Layout for buttons
        QHBoxLayout* buttons = new QHBoxLayout();
        buttons->addWidget(this->refreshButton);
        buttons->addWidget(this->applyButton);

        mainLayout->addLayout(buttons);
        
        // Create a layout to hold widget that will be in QScrollArea
        QGridLayout* scrollLayout = new QGridLayout();

        // QScrollArea has to be applied to widget so widget container made
        QWidget* scrollWidget = new QWidget();
        scrollWidget->setLayout(mainLayout);
        
        
        // Create QScrollArea and setWidgetResizable to true so widget fills scroll area
        QScrollArea* scroll = new QScrollArea();
        scroll->setWidget(scrollWidget);
        scroll->setWidgetResizable(true);

        // Add Scroll Widget to Scroll Layout
        scrollLayout->addWidget(scroll, 0, 0);
        
        // Set Main Layout for Widget to scrollLayout
        setLayout(scrollLayout);

    }


    ParamPanel::~ParamPanel()
    {
        this->eraseLayout(this->layout());
        delete this->layout();
        delete this;
    }


    void ParamPanel::load(const rviz_common::Config &config) 
    {
        config.mapGetString("robot_namespace", &robotNs);
        if(robotNs == "")
        {
            robotNs = QString::fromStdString("/talos"); 
            RVIZ_COMMON_LOG_WARNING("FeedforwardPanel: Using /talos as the default value for robot_namespace"); 
        }

        // Create a node to sync parameters to
        this->node = rclcpp::Node::make_shared("param_panel", robotNs.toStdString());
        // Get list of available nodes and sets default values

        RVIZ_COMMON_LOG_INFO("ParamPanel Panel loaded. Robot NS: \"" + robotNs.toStdString() + "\"");
        loaded = true;
        this->getNodes();
    }


    void ParamPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }

    //called when panel is initialized. perform any needed UI connections here
    void ParamPanel::onInitialize()
    {
        connect(this->nodes, &QComboBox::currentTextChanged, this, &ParamPanel::setNode);
        connect(this->refreshButton, &QPushButton::clicked, this, &ParamPanel::refresh);
        connect(this->applyButton, &QPushButton::clicked, this, &ParamPanel::apply);
    }

    void ParamPanel::getNodes(){
        
        this->gettingNodes = 1;
        // Gets current text of Node box
        QString currentText = this->nodes->currentText();

        // Clears node box
        this->nodes->clear();

        std::vector<std::string> nodeList;

        // Get available nodes from param ros node and add names to Node box
        for(std::string str : this->node->get_node_names())
            nodeList.push_back(str);

        std::sort(nodeList.begin(), nodeList.end());

        for(std::string str : nodeList)
            this->nodes->addItem(str.c_str());

        // If the text from begining of method is in current node box then get index
        int ind = this->nodes->findText(currentText);

        this->gettingNodes = 0;
        // If text if found set currentIndex of nodebox to currentText index
        if(ind != -1)
            this->nodes->setCurrentIndex(ind);

    }

    std::vector<rclcpp::Parameter> ParamPanel::getParams(){

        std::vector<rclcpp::Parameter> paramReply;
        
        if(!connected){
            RVIZ_COMMON_LOG_ERROR("Not Connected to Node");
            return paramReply;
        }

        std::vector<std::string> prefixes = {};

        // Try to get parameters from requested node
        // Doesn't always work and not really sure why as documentation of ros sucks
        // Normally doesn't work on nodes outside of robot namespace
        try{
        rcl_interfaces::msg::ListParametersResult msg = param_client->list_parameters(prefixes, 1, 1s);
        paramReply = this->param_client->get_parameters(msg.names, 1s);
        }catch(...){
            RVIZ_COMMON_LOG_ERROR("Can't get paramaters from node");
        }

        return paramReply;

    }

    void ParamPanel::createParams(std::vector<rclcpp::Parameter> params){
        
        
        for(rclcpp::Parameter param : params){

            // Create Horizontal Box for the current parameter
            QHBoxLayout* hBox = new QHBoxLayout();

            // Get name of current parameter and put in QString
            QString name = QString::fromStdString(param.get_name());

            // Generate the Label for Parameter name
            QLabel* nameLabel = new QLabel(QString::fromStdString(param.get_name() + ":"));
            QFontMetrics fontInfo = nameLabel->fontMetrics();

            // Set Max size so label doesn't change size when window changes size
            nameLabel->setMaximumSize(fontInfo.horizontalAdvance(name), fontInfo.height());
            nameLabel->setObjectName(name);
            hBox->addWidget(nameLabel);

            // For every parameter type create a specific box that supports that type 
            switch(param.get_type()){

                case rclcpp::PARAMETER_INTEGER: {
                    QSpinBox* box = new QSpinBox();
                    box->setMinimum(INT_MIN);
                    box->setMaximum(INT_MAX);
                    box->setObjectName(name);
                    box->setValue(param.as_int());
                    hBox->addWidget(box);
                } break;

                case rclcpp::PARAMETER_INTEGER_ARRAY: {

                    QVBoxLayout* arrVBox = new QVBoxLayout();

                    std::vector<int64_t> arr = param.as_integer_array();

                    for(unsigned int i = 0; i < arr.size(); i++){
                        QString num = QString::fromStdString(std::to_string(i) + ":");
                        QLabel* label = new QLabel(num);
                        label->setMaximumWidth(fontInfo.horizontalAdvance(num));
                        
                        QSpinBox* box = new QSpinBox();
                        box->setMinimum(INT_MIN);
                        box->setMaximum(INT_MAX);
                        box->setObjectName(name);
                        box->setValue(arr.at(i));

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        arrHBox->addWidget(label);
                        arrHBox->addWidget(box);

                        arrVBox->addLayout(arrHBox);
                    }

                    hBox->addLayout(arrVBox);

                } break;

                case rclcpp::PARAMETER_DOUBLE: {
                    QDoubleSpinBox* box = new QDoubleSpinBox();
                    box->setMinimum(-DBL_MAX);
                    box->setMaximum(DBL_MAX);
                    box->setObjectName(name);
                    box->setValue(param.as_double());
                    hBox->addWidget(box);
                } break;

                case rclcpp::PARAMETER_DOUBLE_ARRAY: {

                    QVBoxLayout* arrVBox = new QVBoxLayout();

                    std::vector<double> arr = param.as_double_array();

                    for(unsigned int i = 0; i < arr.size(); i++){
                        
                        QString num = QString::fromStdString(std::to_string(i) + ":");
                        QLabel* label = new QLabel(num);
                        label->setMaximumWidth(fontInfo.horizontalAdvance(num));

                        QDoubleSpinBox* box = new QDoubleSpinBox();
                        box->setMinimum(-DBL_MAX);
                        box->setMaximum(DBL_MAX);
                        box->setObjectName(name);
                        box->setValue(arr.at(i));

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        arrHBox->addWidget(label);
                        arrHBox->addWidget(box);

                        arrVBox->addLayout(arrHBox);
                    }

                    hBox->addLayout(arrVBox);

                } break;

                case rclcpp::PARAMETER_BOOL: {

                    QCheckBox* box = new QCheckBox();

                    box->setObjectName(name);
                    box->setChecked(param.as_bool());

                    hBox->addWidget(box);

                } break;

                case rclcpp::PARAMETER_BOOL_ARRAY: {

                    QVBoxLayout* arrVBox = new QVBoxLayout();
                    std::vector<bool> arr = param.as_bool_array();

                    for(unsigned int i = 0; i < arr.size(); i++){

                        QString num = QString::fromStdString(std::to_string(i) + ":");
                        QLabel* label = new QLabel(num);
                        label->setMaximumWidth(fontInfo.horizontalAdvance(num));

                        QCheckBox* box = new QCheckBox();

                        box->setObjectName(name);
                        box->setChecked(arr.at(i));

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        arrHBox->addWidget(label);
                        arrHBox->addWidget(box);

                        arrVBox->addLayout(arrHBox);
                    }

                    hBox->addLayout(arrVBox);

                } break;

                case rclcpp::PARAMETER_STRING: {

                    QTextEdit* box = new QTextEdit();
                    box->setObjectName(name);
                    box->setPlainText(QString::fromStdString(param.as_string()));
                    hBox->addWidget(box);

                } break;

                case rclcpp::PARAMETER_STRING_ARRAY: {

                    QVBoxLayout* arrVBox = new QVBoxLayout();
                    std::vector<std::string> arr = param.as_string_array();
                    
                    for(unsigned int i = 0; i < arr.size(); i++){

                        QString num = QString::fromStdString(std::to_string(i) + ":");
                        QLabel* label = new QLabel(num);
                        label->setMaximumWidth(fontInfo.horizontalAdvance(num));

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        QTextEdit* box = new QTextEdit();
                        box->setObjectName(name);
                        box->setPlainText(QString::fromStdString(arr.at(i)));

                        arrHBox->addWidget(label);
                        arrHBox->addWidget(box);

                        arrVBox->addLayout(arrHBox);
                    }

                    hBox->addLayout(arrVBox);

                } break;

                case rclcpp::PARAMETER_BYTE_ARRAY: {
                    hBox->addWidget(new QLabel("Byte Array Not Supported Yet"));
                } break;

                case rclcpp::PARAMETER_NOT_SET:
                default:
                    hBox->addWidget(new QLabel("Parameter not Set"));    
            }
            
            this->paramLayout->addLayout(hBox);

        } 
    }

    // When the Node Combo Box Changes connect to the new node selected
    void ParamPanel::setNode(const QString &text){
        
        if(this->gettingNodes)
            return;

        // Erase items in paramLayout
        this->eraseLayout(this->paramLayout);

        // Setup sync client for parameters to node given
        this->param_client = std::make_shared<rclcpp::SyncParametersClient>(this->node, text.toStdString());

        // If Node can't connect log error and terminate Else Connect to Node and call Helper Methods
        if(!this->param_client->wait_for_service(200ms)){
            RVIZ_COMMON_LOG_ERROR("Can't Connect to Param Node");
            this->connected = 0;
        }else{
            RVIZ_COMMON_LOG_INFO("Connected to Node");
            this->connected = 1;
            this->createParams(this->getParams());
        }

    }

    void ParamPanel::refresh(){
        this->getNodes();
    }

    // Storing layouts inside of layouts creates a tree this method gets to the bottom
    // Of the tree recursivly and deletes all items from bottom up
    void ParamPanel::eraseLayout(QLayout* layout){
        
        QLayoutItem* child;

        // Iterate through items in layout until there are none left
        while((child = layout->itemAt(0)) != nullptr){
            // If child item is a layout recurrsivly call this method 
            if(child->layout() != nullptr){
                eraseLayout(child->layout());
                delete child->layout();
            }else{
                // If item is widget bottom of branch is reached delete widgets
                delete child->widget();
            }

        }

    }

    void ParamPanel::apply() {

        std::vector<std::string> params;

        // Get list of parameters that are currently in the ParamPanel
        
        for(int i = 0; i < this->paramLayout->count(); i++){
            QLabel* label = dynamic_cast<QLabel*>(this->paramLayout->itemAt(i)->layout()->itemAt(0)->widget());
            params.push_back(label->objectName().toStdString());
        }

        std::vector<rclcpp::ParameterType> paramType = this->param_client->get_parameter_types(params);
        std::vector<rclcpp::Parameter> newParams;
        
        // For each parameter in ParamPanel get the value and create a Parameter to be adde dto newParams
        for(unsigned int i = 0; i < params.size(); i++){

            switch(paramType.at(i)){

                case rclcpp::PARAMETER_INTEGER: {

                    QSpinBox* box = dynamic_cast<QSpinBox*>(this->paramLayout->itemAt(i)->layout()->itemAt(1)->widget());

                    rclcpp::ParameterValue paramVal(box->value());
                    rclcpp::Parameter newParam(params.at(i), paramVal);

                    newParams.push_back(newParam);

                } break;

                case rclcpp::PARAMETER_INTEGER_ARRAY: {

                    int len = this->paramLayout->itemAt(i)->layout()->itemAt(1)->layout()->count();
                    std::vector<int> vals;

                    for(int j = 0; j < len; j++){
                        
                        QSpinBox* box = dynamic_cast<QSpinBox*>(this->paramLayout->itemAt(i)->layout()->itemAt(1)->layout()->itemAt(j)->layout()->itemAt(1)->widget());
                        vals.push_back(box->value());

                    }

                    rclcpp::ParameterValue paramVal(vals);
                    rclcpp::Parameter newParam(params.at(i), paramVal);

                    newParams.push_back(newParam);


                } break;

                case rclcpp::PARAMETER_DOUBLE: {

                    QDoubleSpinBox* box = dynamic_cast<QDoubleSpinBox*>(this->paramLayout->itemAt(i)->layout()->itemAt(1)->widget());

                    rclcpp::ParameterValue paramVal(box->value());
                    rclcpp::Parameter newParam(params.at(i), paramVal);

                    newParams.push_back(newParam);

                } break;

                case rclcpp::PARAMETER_DOUBLE_ARRAY: {

                    int len = this->paramLayout->itemAt(i)->layout()->itemAt(1)->layout()->count();
                    std::vector<double> vals;

                    for(int j = 0; j < len; j++){
                        
                        QDoubleSpinBox* box = dynamic_cast<QDoubleSpinBox*>(this->paramLayout->itemAt(i)->layout()->itemAt(1)->layout()->itemAt(j)->layout()->itemAt(1)->widget());
                        vals.push_back(box->value());

                    }

                    rclcpp::ParameterValue paramVal(vals);
                    rclcpp::Parameter newParam(params.at(i), paramVal);

                    newParams.push_back(newParam);

                } break;

                case rclcpp::PARAMETER_BOOL: {

                    QCheckBox* box = dynamic_cast<QCheckBox*>(this->paramLayout->itemAt(i)->layout()->itemAt(1)->widget());

                    rclcpp::ParameterValue paramVal(box->isChecked());
                    rclcpp::Parameter newParam(params.at(i), paramVal);

                    newParams.push_back(newParam);

                } break;

                case rclcpp::PARAMETER_BOOL_ARRAY: {

                    int len = this->paramLayout->itemAt(i)->layout()->itemAt(1)->layout()->count();
                    std::vector<bool> vals;

                    for(int j = 0; j < len; j++){
                        
                        QCheckBox* box = dynamic_cast<QCheckBox*>(this->paramLayout->itemAt(i)->layout()->itemAt(1)->layout()->itemAt(j)->layout()->itemAt(1)->widget());
                        vals.push_back(box->isChecked());

                    }

                    rclcpp::ParameterValue paramVal(vals);
                    rclcpp::Parameter newParam(params.at(i), paramVal);

                    newParams.push_back(newParam);

                } break;

                case rclcpp::PARAMETER_STRING: {

                    QTextEdit* box = dynamic_cast<QTextEdit*>(this->paramLayout->itemAt(i)->layout()->itemAt(1)->widget());

                    rclcpp::ParameterValue paramVal(box->toPlainText().toStdString());
                    rclcpp::Parameter newParam(params.at(i), paramVal);

                    newParams.push_back(newParam);

                } break;

                case rclcpp::PARAMETER_STRING_ARRAY: {

                    int len = this->paramLayout->itemAt(i)->layout()->itemAt(1)->layout()->count();
                    std::vector<std::string> vals;

                    for(int j = 0; j < len; j++){
                        
                        QTextEdit* box = dynamic_cast<QTextEdit*>(this->paramLayout->itemAt(i)->layout()->itemAt(1)->layout()->itemAt(j)->layout()->itemAt(1)->widget());
                        vals.push_back(box->toPlainText().toStdString());

                    }

                    rclcpp::ParameterValue paramVal(vals);
                    rclcpp::Parameter newParam(params.at(i), paramVal);

                    newParams.push_back(newParam);

                } break;

                case rclcpp::PARAMETER_BYTE_ARRAY:
                case rclcpp::PARAMETER_NOT_SET:
                    RVIZ_COMMON_LOG_WARNING("Type not supported");
            }
            
            // Sets parameters and logs result
            RVIZ_COMMON_LOG_INFO(this->param_client->set_parameters_atomically(newParams).reason);

        }

    }

}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ParamPanel, rviz_common::Panel);
