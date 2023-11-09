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
        QHBoxLayout* tempLayout = new QHBoxLayout();
        tempLayout->addWidget(new QDoubleSpinBox());
        tempLayout->addWidget(new QLabel("Test"));
        this->paramLayout->addLayout(tempLayout);
        
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
        nodeBox->addWidget(new QLabel("Node:"));
        nodeBox->addWidget(this->nodes);

        // Adds nodeBox to the Main Layout
        mainLayout->addLayout(nodeBox);

        mainLayout->addLayout(this->paramLayout);

        // Create Layout for buttons
        QHBoxLayout* buttons = new QHBoxLayout();
        buttons->addWidget(this->refreshButton);
        buttons->addWidget(this->applyButton);

        mainLayout->addLayout(buttons);
        
        QGridLayout* scrollLayout = new QGridLayout();

        

        QWidget* scrollWidget = new QWidget();
        scrollWidget->setLayout(mainLayout);
        
        

        QScrollArea* scroll = new QScrollArea();
        scroll->setWidget(scrollWidget);
        scroll->setWidgetResizable(true);

        scrollLayout->addWidget(scroll, 0, 0);
        
        // Sets the layout for the widget to the mainLayout
        setLayout(scrollLayout);

    }


    ParamPanel::~ParamPanel()
    {
        
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
        this->getNodes();

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
        connect(this->nodes, &QComboBox::currentTextChanged, this, &ParamPanel::setNode);
        connect(this->refreshButton, &QPushButton::clicked, this, &ParamPanel::refresh);
        connect(this->applyButton, &QPushButton::clicked, this, &ParamPanel::apply);
        // connect(widgetVec.at(0), QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ParamPanel::setValue);
    }

    void ParamPanel::getNodes(){
        
        QString currentText = this->nodes->currentText();

        this->nodes->clear();

        for(std::string str : this->node->get_node_names())
            this->nodes->addItem(str.c_str());

        int ind = this->nodes->findText(currentText);

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

            QHBoxLayout* hBox = new QHBoxLayout();
            QString name = QString::fromStdString(param.get_name());

            hBox->addWidget(new QLabel(name));

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
                        QSpinBox* box = new QSpinBox();
                        box->setMinimum(INT_MIN);
                        box->setMaximum(INT_MAX);
                        box->setObjectName(name);
                        box->setValue(arr.at(i));

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        arrHBox->addWidget(new QLabel(QString::fromStdString(std::to_string(i) + ":")));
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
                        QDoubleSpinBox* box = new QDoubleSpinBox();
                        box->setMinimum(-DBL_MAX);
                        box->setMaximum(DBL_MAX);
                        box->setObjectName(name);
                        box->setValue(arr.at(i));

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        arrHBox->addWidget(new QLabel(QString::fromStdString(std::to_string(i) + ":")));
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

                        QCheckBox* box = new QCheckBox();

                        box->setObjectName(name);
                        box->setChecked(arr.at(i));

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        arrHBox->addWidget(new QLabel(QString::fromStdString(std::to_string(i) + ":")));
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

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        QTextEdit* box = new QTextEdit();
                        box->setObjectName(name);
                        box->setPlainText(QString::fromStdString(arr.at(i)));

                        arrHBox->addWidget(new QLabel(QString::fromStdString(std::to_string(i) + ":")));
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
        
        // Erase items in paramLayout
        this->eraseLayout(this->paramLayout);

        // Setup sync client for parameters to node given
        this->param_client = std::make_shared<rclcpp::SyncParametersClient>(this->node, text.toStdString());

        // If Node can't connect log error and terminate Else Connect to Node and call Helper Methods
        if(!this->param_client->wait_for_service(1s)){
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
        this->setNode(this->nodes->currentText());
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

        for(int i = 0; i < this->paramLayout->count(); i++){
            QLabel* label = dynamic_cast<QLabel*>(this->paramLayout->itemAt(i)->layout()->itemAt(0)->widget());
            params.push_back(label->text().toStdString());
        }

        std::vector<rclcpp::ParameterType> paramType = this->param_client->get_parameter_types(params);

        std::vector<rclcpp::Parameter> newParams;

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

                }

            }

            this->param_client->set_parameters(newParams);

        }

    }

}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ParamPanel, rviz_common::Panel);
