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
#include <QTextEdit>
#include <QLineEdit>
#include <vector>
#include <limits.h>
#include <QScrollArea>
#include <float.h>
#include <QEvent>
#include <QMetaObject>
#include <QStatusBar>
#include <QTimer>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace riptide_rviz
{
    ParamPanel::ParamPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        // Create a combo box to select node to edit
        this->nodes = new QComboBox();
        
        // Add search box for parameters
        this->searchBox = new QLineEdit();
        this->searchBox->setPlaceholderText("Search parameters...");
        this->searchBox->setClearButtonEnabled(true);
        
        // Create a container for the search box that we can show/hide
        this->searchContainer = new QWidget();
        QHBoxLayout* searchLayout = new QHBoxLayout(this->searchContainer);
        searchLayout->addWidget(new QLabel("Search:"));
        searchLayout->addWidget(this->searchBox);
        searchLayout->setContentsMargins(0, 0, 0, 0);
        
        // Initially hide the search container
        this->searchContainer->setVisible(false);

        // Will hold the parameters that are being edited
        this->paramLayout = new QVBoxLayout();

        // Create a status bar for messages
        this->statusBar = new QStatusBar();
        this->statusBar->setSizeGripEnabled(false);
        
        // Adds a button to refresh params
        this->refreshButton = new QPushButton("&Refresh");
        this->refreshButton->setObjectName("refresh");

        // Adds a button to apply params
        this->applyButton = new QPushButton("&Apply");
        this->applyButton->setObjectName("apply");

        // Create Layout for buttons that will be fixed at the bottom
        QHBoxLayout* buttons = new QHBoxLayout();
        buttons->addWidget(this->refreshButton);
        buttons->addWidget(this->applyButton);
        
        // Create a widget to hold the buttons and status bar
        QWidget* bottomWidget = new QWidget();
        QVBoxLayout* bottomLayout = new QVBoxLayout(bottomWidget);
        bottomLayout->addLayout(buttons);
        bottomLayout->addWidget(this->statusBar);
        bottomLayout->setContentsMargins(5, 5, 5, 5);
        
        // Main Layout that will hold everything (node selector, search, parameters)
        QVBoxLayout* scrollContentsLayout = new QVBoxLayout();

        // Holds the Node selector combo box
        QHBoxLayout* nodeBox = new QHBoxLayout();
        nodeBox->addWidget(new QLabel("Node:"));
        nodeBox->addWidget(this->nodes);

        // Adds layouts to the content layout
        scrollContentsLayout->addLayout(nodeBox);
        scrollContentsLayout->addWidget(this->searchContainer); // Add search container
        scrollContentsLayout->addLayout(this->paramLayout);
        scrollContentsLayout->addStretch(1); // Add stretch to push everything up
        
        // Create a widget to hold the scrollable content
        QWidget* scrollWidget = new QWidget();
        scrollWidget->setLayout(scrollContentsLayout);
        
        // Create a scroll area for the parameters
        QScrollArea* scroll = new QScrollArea();
        scroll->setWidget(scrollWidget);
        scroll->setWidgetResizable(true);
        
        // Create the main layout that will hold the scroll area and bottom widget
        QVBoxLayout* mainLayout = new QVBoxLayout();
        mainLayout->addWidget(scroll);
        mainLayout->addWidget(bottomWidget);
        mainLayout->setContentsMargins(0, 0, 0, 0);
        mainLayout->setSpacing(0);
        
        // Sets the layout for the panel widget
        setLayout(mainLayout);
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

    // Helper method to show status messages
    void ParamPanel::showStatusMessage(const QString& message, int timeout)
    {
        this->statusBar->showMessage(message, timeout);
    }

    //called when panel is initialized. perform any needed UI connections here
    void ParamPanel::onInitialize()
    {
        connect(this->nodes, &QComboBox::currentTextChanged, this, &ParamPanel::setNode);
        connect(this->refreshButton, &QPushButton::clicked, this, &ParamPanel::refresh);
        connect(this->applyButton, &QPushButton::clicked, this, &ParamPanel::apply);
        connect(this->searchBox, &QLineEdit::textChanged, this, &ParamPanel::filterParameters);
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
            showStatusMessage("Not connected to node", 3000);
            return paramReply;
        }

        std::vector<std::string> prefixes = {};

        try{
            rcl_interfaces::msg::ListParametersResult msg = param_client->list_parameters(prefixes, 1, 1s);
            paramReply = this->param_client->get_parameters(msg.names, 1s);
            showStatusMessage(QString("Loaded %1 parameters").arg(paramReply.size()), 3000);
        } catch(const std::exception& e) {
            RVIZ_COMMON_LOG_ERROR("Can't get parameters from node: " + std::string(e.what()));
            showStatusMessage("Error: Can't get parameters from node", 3000);
        } catch(...) {
            RVIZ_COMMON_LOG_ERROR("Can't get parameters from node");
            showStatusMessage("Error: Can't get parameters from node", 3000);
        }

        return paramReply;
    }

    // Helper method to store the original value of a parameter
    void ParamPanel::storeOriginalValue(QWidget* widget, const QString& name) {
        if (!widget) return;
        
        originalValues[name] = getCurrentValue(widget);
    }

    // Helper method to get the current value from a widget
    QVariant ParamPanel::getCurrentValue(QWidget* widget) {
        if (!widget) return QVariant();
        
        if (auto spinBox = qobject_cast<QSpinBox*>(widget)) {
            return spinBox->value();
        } else if (auto doubleSpinBox = qobject_cast<QDoubleSpinBox*>(widget)) {
            return doubleSpinBox->value();
        } else if (auto checkBox = qobject_cast<QCheckBox*>(widget)) {
            return checkBox->isChecked();
        } else if (auto textEdit = qobject_cast<QTextEdit*>(widget)) {
            return textEdit->toPlainText();
        }
        
        return QVariant();
    }

    // Helper method to highlight widgets when values change
    void ParamPanel::highlightChanges(QWidget* widget, const QString& name) {
        if (!widget || !originalValues.contains(name)) return;
        
        QVariant currentValue = getCurrentValue(widget);
        
        if (currentValue != originalValues[name]) {
            widget->setStyleSheet("background-color: #ffffcc;");
        } else {
            widget->setStyleSheet("");
        }
    }

    // Slot to handle parameter value changes
    void ParamPanel::parameterValueChanged() {
        QObject* senderObj = sender();
        if (!senderObj) return;
        
        QString name = senderObj->objectName();
        highlightChanges(qobject_cast<QWidget*>(senderObj), name);
    }

    void ParamPanel::createParams(std::vector<rclcpp::Parameter> params){ 
        // Update search box visibility based on parameter count
        this->searchContainer->setVisible(!params.empty());
        
        // Clear original values map
        originalValues.clear();
        
        for(rclcpp::Parameter param : params){

            QHBoxLayout* hBox = new QHBoxLayout();
            QString name = QString::fromStdString(param.get_name());

            hBox->addWidget(new QLabel(name));

            // Create a container widget to hold the parameter row
            QWidget* container = new QWidget();
            QWidget* valueWidget = nullptr;

            switch(param.get_type()){

                case rclcpp::PARAMETER_INTEGER: {
                    QSpinBox* box = new QSpinBox();
                    box->setMinimum(INT_MIN);
                    box->setMaximum(INT_MAX);
                    box->setObjectName(name);
                    box->setValue(param.as_int());
                    box->installEventFilter(this);  // Disable wheel scroll here
                    connect(box, QOverload<int>::of(&QSpinBox::valueChanged), this, &ParamPanel::parameterValueChanged);
                    hBox->addWidget(box);
                    valueWidget = box;
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
                        box->setProperty("orig", static_cast<qlonglong>(arr.at(i))); // Store per element in array
                        box->installEventFilter(this);  // Disable wheel scroll here
                        connect(box, QOverload<int>::of(&QSpinBox::valueChanged), this, &ParamPanel::parameterValueChanged);

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        arrHBox->addWidget(new QLabel(QString::fromStdString(std::to_string(i) + ":")));
                        arrHBox->addWidget(box);
                        arrHBox->setStretch(0, 0); // tighten inner label column
                        arrHBox->setStretch(1, 1); // let value take remaining space

                        arrVBox->addLayout(arrHBox);
                    }

                    hBox->addLayout(arrVBox);
                    hBox->setStretch(0, 0); // tighten param name column
                    hBox->setStretch(1, 1); // let array side take remaining space

                } break;

                case rclcpp::PARAMETER_DOUBLE: {
                    QDoubleSpinBox* box = new QDoubleSpinBox();
                    box->setMinimum(-DBL_MAX);
                    box->setMaximum(DBL_MAX);
                    box->setObjectName(name);
                    box->setValue(param.as_double());
                    box->installEventFilter(this);  // Disable wheel scroll here
                    connect(box, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ParamPanel::parameterValueChanged);
                    hBox->addWidget(box);
                    valueWidget = box;
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
                        box->setProperty("orig", arr.at(i)); // Store per element in array
                        box->installEventFilter(this);  // Disable wheel scroll here
                        connect(box, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ParamPanel::parameterValueChanged);

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        arrHBox->addWidget(new QLabel(QString::fromStdString(std::to_string(i) + ":")));
                        arrHBox->addWidget(box);
                        arrHBox->setStretch(0, 0); // tighten inner label column
                        arrHBox->setStretch(1, 1); // let value take remaining space

                        arrVBox->addLayout(arrHBox);
                    }

                    hBox->addLayout(arrVBox);
                    hBox->setStretch(0, 0); // tighten param name column
                    hBox->setStretch(1, 1); // let array side take remaining space

                } break;

                case rclcpp::PARAMETER_BOOL: {

                    QCheckBox* box = new QCheckBox();

                    box->setObjectName(name);
                    box->setChecked(param.as_bool());
                    connect(box, &QCheckBox::stateChanged, this, &ParamPanel::parameterValueChanged);

                    hBox->addWidget(box);
                    valueWidget = box;

                } break;

                case rclcpp::PARAMETER_BOOL_ARRAY: {

                    QVBoxLayout* arrVBox = new QVBoxLayout();
                    std::vector<bool> arr = param.as_bool_array();

                    for(unsigned int i = 0; i < arr.size(); i++){

                        QCheckBox* box = new QCheckBox();

                        box->setObjectName(name);
                        box->setChecked(arr.at(i));
                        box->setProperty("orig", static_cast<bool>(arr.at(i))); // Store per element in array
                        connect(box, &QCheckBox::stateChanged, this, &ParamPanel::parameterValueChanged);

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        arrHBox->addWidget(new QLabel(QString::fromStdString(std::to_string(i) + ":")));
                        arrHBox->addWidget(box);
                        arrHBox->setStretch(0, 0); // tighten inner label column
                        arrHBox->setStretch(1, 1); // let value take remaining space

                        arrVBox->addLayout(arrHBox);
                    }

                    hBox->addLayout(arrVBox);
                    hBox->setStretch(0, 0); // tighten param name column
                    hBox->setStretch(1, 1); // let array side take remaining space

                } break;

                case rclcpp::PARAMETER_STRING: {

                    QTextEdit* box = new QTextEdit();
                    box->setObjectName(name);
                    box->setPlainText(QString::fromStdString(param.as_string()));
                    connect(box, &QTextEdit::textChanged, this, &ParamPanel::parameterValueChanged);
                    hBox->addWidget(box);
                    valueWidget = box;

                } break;

                case rclcpp::PARAMETER_STRING_ARRAY: {

                    QVBoxLayout* arrVBox = new QVBoxLayout();
                    std::vector<std::string> arr = param.as_string_array();
                    
                    for(unsigned int i = 0; i < arr.size(); i++){

                        QHBoxLayout* arrHBox = new QHBoxLayout();

                        QTextEdit* box = new QTextEdit();
                        box->setObjectName(name);
                        box->setPlainText(QString::fromStdString(arr.at(i)));
                        box->setProperty("orig", QString::fromStdString(arr.at(i))); // Store per element in array
                        connect(box, &QTextEdit::textChanged, this, &ParamPanel::parameterValueChanged);

                        arrHBox->addWidget(new QLabel(QString::fromStdString(std::to_string(i) + ":")));
                        arrHBox->addWidget(box);
                        arrHBox->setStretch(0, 0);
                        arrHBox->setStretch(1, 1);

                        arrVBox->addLayout(arrHBox);
                    }

                    hBox->addLayout(arrVBox);
                    hBox->setStretch(0, 0);
                    hBox->setStretch(1, 1);

                } break;

                case rclcpp::PARAMETER_BYTE_ARRAY: {
                    hBox->addWidget(new QLabel("Byte Array Not Supported Yet"));
                } break;

                case rclcpp::PARAMETER_NOT_SET:
                default:
                    hBox->addWidget(new QLabel("Parameter not Set"));    
            }
            
            // Store original value for change highlighting
            if (valueWidget) {
                storeOriginalValue(valueWidget, name);
            }
            
            // Set the layout to the container
            container->setLayout(hBox);
            
            // Add the container to the parameter layout
            this->paramLayout->addWidget(container);
        } 
    }

    // Event filter to block scroll events for spin boxes
    bool ParamPanel::eventFilter(QObject* obj, QEvent* event) {
        if (event->type() == QEvent::Wheel) {
            if (dynamic_cast<QDoubleSpinBox*>(obj) || dynamic_cast<QSpinBox*>(obj)) {
                return true;  // Block the wheel event
            }
        }
        return QObject::eventFilter(obj, event);
    }

    // Filter parameters based on search text
    void ParamPanel::filterParameters(const QString& searchText)
    {
        if (searchText.isEmpty()) {
            // Show all parameters
            for (int i = 0; i < this->paramLayout->count(); i++) {
                QLayoutItem* item = this->paramLayout->itemAt(i);
                if (item->widget()) {
                    item->widget()->setVisible(true);
                }
            }
            return;
        }
        
        // Filter parameters based on search text
        for (int i = 0; i < this->paramLayout->count(); i++) {
            QLayoutItem* item = this->paramLayout->itemAt(i);
            if (item->widget()) {
                QWidget* container = item->widget();
                QLayout* containerLayout = container->layout();
                
                bool shouldShow = false;
                if (containerLayout && containerLayout->count() > 0) {
                    QLabel* label = dynamic_cast<QLabel*>(containerLayout->itemAt(0)->widget());
                    if (label) {
                        QString paramName = label->text();
                        shouldShow = paramName.contains(searchText, Qt::CaseInsensitive);
                    }
                }
                
                container->setVisible(shouldShow);
            }
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
            this->searchContainer->setVisible(false);
            showStatusMessage("Error: Can't connect to node " + text, 5000);
        }else{
            RVIZ_COMMON_LOG_INFO("Connected to Node");
            this->connected = 1;
            showStatusMessage("Connected to node " + text, 3000);
            this->createParams(this->getParams());
        }
        
        // Clear the search box when changing nodes
        this->searchBox->clear();
    }

    void ParamPanel::refresh(){
        showStatusMessage("Refreshing node list and parameters...");
        this->getNodes();
        this->setNode(this->nodes->currentText());
    }

    // Storing layouts inside of layouts creates a tree this method gets to the bottom
    // Of the tree recursivly and deletes all items from bottom up
    void ParamPanel::eraseLayout(QLayout* layout){
        QLayoutItem* child;

        // Iterate through items in layout until there are none left
        while((child = layout->itemAt(0)) != nullptr) {
            // If child item is a layout recursively call this method 
            if(child->layout() != nullptr) {
                eraseLayout(child->layout());
                delete child->layout();
            } else if (child->widget() != nullptr) {
                // If item is widget, delete it
                delete child->widget();
            }
            // Remove the item from the layout
            layout->removeItem(child);
        }
    }

    void ParamPanel::apply() {
        // Show initial status message
        showStatusMessage("Applying modified parameters...");
        
        std::vector<rclcpp::Parameter> modifiedParams;
        std::vector<std::string> arrayParamsChanged; // Trakcing arrays that got changed (they're special)
        
        // Iterate through all visible parameters to find modified ones
        for(int i = 0; i < this->paramLayout->count(); i++) {
            QLayoutItem* item = this->paramLayout->itemAt(i);
            if (!item->widget() || !item->widget()->isVisible()) {
                continue;  // Skip if not visible (filtered out)
            }
            
            QWidget* container = item->widget();
            QLayout* containerLayout = container->layout();
            
            if (!containerLayout || containerLayout->count() < 2) {
                continue;
            }
            
            // Get the parameter name from the label
            QLabel* label = dynamic_cast<QLabel*>(containerLayout->itemAt(0)->widget());
            if (!label) {
                continue;
            }
            
            QString paramName = label->text();
            std::string paramNameStr = paramName.toStdString();
            
            // Get the widget containing the value
            QWidget* valueWidget = nullptr;
            QVBoxLayout* arrLayout = nullptr;
            
            // Check if it's a regular widget or an array layout
            if (QWidget* widget = dynamic_cast<QWidget*>(containerLayout->itemAt(1)->widget())) {
                valueWidget = widget;
                
                // Check if the value has been modified
                if (originalValues.contains(paramName) && 
                    getCurrentValue(valueWidget) != originalValues[paramName]) {
                    
                    // Process based on widget type
                    if (auto spinBox = qobject_cast<QSpinBox*>(valueWidget)) {
                        modifiedParams.push_back(rclcpp::Parameter(paramNameStr, spinBox->value()));
                        originalValues[paramName] = spinBox->value();
                        spinBox->setStyleSheet("");
                    } else if (auto doubleSpinBox = qobject_cast<QDoubleSpinBox*>(valueWidget)) {
                        modifiedParams.push_back(rclcpp::Parameter(paramNameStr, doubleSpinBox->value()));
                        originalValues[paramName] = doubleSpinBox->value();
                        doubleSpinBox->setStyleSheet("");
                    } else if (auto checkBox = qobject_cast<QCheckBox*>(valueWidget)) {
                        modifiedParams.push_back(rclcpp::Parameter(paramNameStr, checkBox->isChecked()));
                        originalValues[paramName] = checkBox->isChecked();
                        checkBox->setStyleSheet("");
                    } else if (auto textEdit = qobject_cast<QTextEdit*>(valueWidget)) {
                        modifiedParams.push_back(rclcpp::Parameter(paramNameStr, textEdit->toPlainText().toStdString()));
                        originalValues[paramName] = textEdit->toPlainText();
                        textEdit->setStyleSheet("");
                    }
                }
            } else if ((arrLayout = dynamic_cast<QVBoxLayout*>(containerLayout->itemAt(1)->layout()))) {
                // Handle arrays
                // We need to get the parameter type first
                auto params = std::vector<std::string>{paramNameStr};
                auto types = this->param_client->get_parameter_types(params, 1s);
                
                if (types.empty()) {
                    continue;
                }
                
                bool isModified = false;
                
                switch(types[0]) {
                    case rclcpp::PARAMETER_INTEGER_ARRAY: {
                        std::vector<int64_t> values;
                        for (int j = 0; j < arrLayout->count(); j++) {
                            QHBoxLayout* arrHBox = dynamic_cast<QHBoxLayout*>(arrLayout->itemAt(j)->layout());
                            if (arrHBox && arrHBox->count() > 1) {
                                QSpinBox* box = dynamic_cast<QSpinBox*>(arrHBox->itemAt(1)->widget());
                                if (box) {
                                    values.push_back(box->value());
                                    QVariant orig = box->property("orig");
                                    if (!orig.isValid() || orig.toLongLong() != static_cast<qlonglong>(box->value())) {
                                        isModified = true;
                                    }
                                }
                            }
                        }
                        if (isModified) {
                            modifiedParams.push_back(rclcpp::Parameter(paramNameStr, values));
                            arrayParamsChanged.push_back(paramNameStr);
                        }
                    } break;
                    
                    case rclcpp::PARAMETER_DOUBLE_ARRAY: {
                        std::vector<double> values;
                        for (int j = 0; j < arrLayout->count(); j++) {
                            QHBoxLayout* arrHBox = dynamic_cast<QHBoxLayout*>(arrLayout->itemAt(j)->layout());
                            if (arrHBox && arrHBox->count() > 1) {
                                QDoubleSpinBox* box = dynamic_cast<QDoubleSpinBox*>(arrHBox->itemAt(1)->widget());
                                if (box) {
                                    values.push_back(box->value());
                                    QVariant orig = box->property("orig");
                                    if (!orig.isValid() || orig.toDouble() != box->value()) {
                                        isModified = true;
                                    }
                                }
                            }
                        }
                        if (isModified) {
                            modifiedParams.push_back(rclcpp::Parameter(paramNameStr, values));
                            arrayParamsChanged.push_back(paramNameStr);
                        }
                    } break;
                    
                    case rclcpp::PARAMETER_BOOL_ARRAY: {
                        std::vector<bool> values;
                        for (int j = 0; j < arrLayout->count(); j++) {
                            QHBoxLayout* arrHBox = dynamic_cast<QHBoxLayout*>(arrLayout->itemAt(j)->layout());
                            if (arrHBox && arrHBox->count() > 1) {
                                QCheckBox* box = dynamic_cast<QCheckBox*>(arrHBox->itemAt(1)->widget());
                                if (box) {
                                    values.push_back(box->isChecked());
                                    QVariant orig = box->property("orig");
                                    if (!orig.isValid() || orig.toBool() != box->isChecked()) {
                                        isModified = true;
                                    }
                                }
                            }
                        }
                        if (isModified) {
                            modifiedParams.push_back(rclcpp::Parameter(paramNameStr, values));
                            arrayParamsChanged.push_back(paramNameStr);
                        }
                    } break;
                    
                    case rclcpp::PARAMETER_STRING_ARRAY: {
                        std::vector<std::string> values;
                        for (int j = 0; j < arrLayout->count(); j++) {
                            QHBoxLayout* arrHBox = dynamic_cast<QHBoxLayout*>(arrLayout->itemAt(j)->layout());
                            if (arrHBox && arrHBox->count() > 1) {
                                QTextEdit* box = dynamic_cast<QTextEdit*>(arrHBox->itemAt(1)->widget());
                                if (box) {
                                    values.push_back(box->toPlainText().toStdString());
                                    QVariant orig = box->property("orig");
                                    if (!orig.isValid() || orig.toString() != box->toPlainText()) {
                                        isModified = true;
                                    }
                                }
                            }
                        }
                        if (isModified) {
                            modifiedParams.push_back(rclcpp::Parameter(paramNameStr, values));
                            arrayParamsChanged.push_back(paramNameStr);
                        }
                    } break;
                    
                    default:
                        break;
                }
            }
        }
    
        // Apply the parameters if we have any modified ones
        if (!modifiedParams.empty()) {
            try {
                // Set parameters and get results
                auto results = this->param_client->set_parameters(modifiedParams, 1s);
                
                // Check for errors
                bool hasErrors = false;
                QString errorMessage = "Failed to set parameters:";
                
                for (size_t i = 0; i < results.size(); i++) {
                    if (!results[i].successful) {
                        hasErrors = true;
                        errorMessage += "\n- " + QString::fromStdString(modifiedParams[i].get_name()) + 
                                        ": " + QString::fromStdString(results[i].reason);
                    }
                }
                
                if (hasErrors) {
                    RVIZ_COMMON_LOG_ERROR(errorMessage.toStdString());
                    showStatusMessage(errorMessage, 5000);
                } else {
                    // Update per-element baselines for arrays we changed
                    if (!arrayParamsChanged.empty()) {
                        // Build a quick lookup
                        std::unordered_set<std::string> changedSet(arrayParamsChanged.begin(), arrayParamsChanged.end());
                        for (int i = 0; i < this->paramLayout->count(); i++) {
                            QLayoutItem* item = this->paramLayout->itemAt(i);
                            if (!item->widget()) continue;
                            QWidget* container = item->widget();
                            QLayout* containerLayout = container->layout();
                            if (!containerLayout || containerLayout->count() < 2) continue;
                            QLabel* label = dynamic_cast<QLabel*>(containerLayout->itemAt(0)->widget());
                            if (!label) continue;
                            std::string name = label->text().toStdString();
                            if (changedSet.find(name) == changedSet.end()) continue;

                            if (auto* arrLayout = dynamic_cast<QVBoxLayout*>(containerLayout->itemAt(1)->layout())) {
                                // Update each element's "orig" to its current value
                                for (int j = 0; j < arrLayout->count(); j++) {
                                    if (auto* arrHBox = dynamic_cast<QHBoxLayout*>(arrLayout->itemAt(j)->layout())) {
                                        if (arrHBox->count() > 1) {
                                            if (auto* sb = dynamic_cast<QSpinBox*>(arrHBox->itemAt(1)->widget())) {
                                                sb->setProperty("orig", static_cast<qlonglong>(sb->value()));
                                            } else if (auto* dsb = dynamic_cast<QDoubleSpinBox*>(arrHBox->itemAt(1)->widget())) {
                                                dsb->setProperty("orig", dsb->value());
                                            } else if (auto* cb = dynamic_cast<QCheckBox*>(arrHBox->itemAt(1)->widget())) {
                                                cb->setProperty("orig", cb->isChecked());
                                            } else if (auto* te = dynamic_cast<QTextEdit*>(arrHBox->itemAt(1)->widget())) {
                                                te->setProperty("orig", te->toPlainText());
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    showStatusMessage(QString("Successfully applied %1 modified parameters").arg(modifiedParams.size()), 3000);
                }
            } catch (const std::exception& e) {
                QString errorMsg = "Error applying parameters: " + QString(e.what());
                RVIZ_COMMON_LOG_ERROR(errorMsg.toStdString());
                showStatusMessage(errorMsg, 5000);
            } catch (...) {
                RVIZ_COMMON_LOG_ERROR("Unknown error applying parameters");
                showStatusMessage("Unknown error applying parameters", 5000);
            }
        } else {
            showStatusMessage("No modified parameters to apply", 3000);
        }
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ParamPanel, rviz_common::Panel);