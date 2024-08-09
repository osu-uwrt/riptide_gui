#include "riptide_rviz/DiagnosticOverlay.hpp"

#include <QFontDatabase>
#include <QMessageBox>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace riptide_rviz
{
    DiagnosticOverlay::DiagnosticOverlay(){
        // init font parameter
        QFontDatabase database;
        fontFamilies = database.families();
        fontProperty = new rviz_common::properties::EnumProperty("font", "DejaVu Sans Mono", "font", this, SLOT(updateFont()));
        for (ssize_t i = 0; i < fontFamilies.size(); i++) {
            fontProperty->addOption(fontFamilies[i], (int) i);
        }

        robotNsProperty = new rviz_common::properties::StringProperty(
            "robot_namespace", "/talos", "Robot namespace to attach to", this, SLOT(updateNS()));

        timeoutProperty = new rviz_common::properties::FloatProperty(
            "diagnostic_timeout", 10.0, "Maximum time between diagnostic packets before default", this);


    }

    DiagnosticOverlay::~DiagnosticOverlay(){
        delete robotNsProperty;
        delete timeoutProperty;
    }

    void DiagnosticOverlay::onInitialize(){
        OverlayDisplay::onInitialize();

        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        // backdate timeouts
        lastDiag = node->get_clock()->now() - 1h;
        lastKill = node->get_clock()->now() - 1h;
        lastZed = node->get_clock()->now() - 1h;
        lastLeak = node->get_clock()->now() - 1h;

        // make the diagnostic subscriber
        diagSub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics_agg", rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlay::diagnosticCallback, this, _1)
        );

        std::string zedTopic = robotNsProperty->getStdString() + "/zed/zed_node/temperature/imu";
        zedSub = node->create_subscription<sensor_msgs::msg::Temperature>(
            zedTopic, rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlay::zedCallback, this, _1)
        );

        std::string leakTopic = robotNsProperty->getStdString() + "/state/leak";
        leakSub = node->create_subscription<std_msgs::msg::Bool>(
            leakTopic, rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlay::leakCallback, this, _1)
        );

        // watchdog timers for handling timeouts
        checkTimer = node->create_wall_timer(0.25s, std::bind(&DiagnosticOverlay::checkTimeout, this));

        // add all of the variable design items
        voltageConfig.text_color_ = QColor(255, 0, 255, 255);
        voltageTextId = addText(voltageConfig);

        diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        diagLedConfigId = addCircle(diagLedConfig);

        killLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        killLedConfigId = addCircle(killLedConfig);

        zedLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        zedLedConfigId = addCircle(zedLedConfig);

        leakLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        leakLedConfigId = addCircle(leakLedConfig);

        // add the static design items
        PaintedTextConfig diagLedLabel = {
            6, 20, 0, 0, "Diag",
            fontName, false, 2, 12,
            QColor(255, 255, 255, 255)
        };
        PaintedTextConfig killLedLabel = {
            50, 20, 0, 0, "Kill",
            fontName, false, 2, 12,
            QColor(255, 255, 255, 255)
        };
        PaintedTextConfig zedLedLabel = {
            87, 20, 0, 0, "Zed",
            fontName, false, 2, 12,
            QColor(255, 255, 255, 255)
        };
        PaintedTextConfig leakLedLabel = {
            6, 70, 0, 0, "Leak",
            fontName, false, 2, 12,
            QColor(255, 255, 255, 255)
        };
        addText(diagLedLabel);
        addText(killLedLabel);
        addText(zedLedLabel);
        addText(leakLedLabel);
    }

    void DiagnosticOverlay::updateNS(){
        RVIZ_COMMON_LOG_INFO_STREAM("DiagnosticOverlay: Robot NS update " << robotNsProperty->getStdString());
        killSub.reset();

        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        killSub = node->create_subscription<std_msgs::msg::Bool>(
            robotNsProperty->getStdString() + "/state/kill", rclcpp::SystemDefaultsQoS(),
            std::bind(&DiagnosticOverlay::killCallback, this, _1)
        );
    }

    void DiagnosticOverlay::diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg){
        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        // write down the timestamp that it was recieved
        lastDiag = node->get_clock()->now();
        diagsTimedOut = false;

        // look for specific packets
        for(auto diagnostic : msg.status){
            // handle robot voltage packet
            if(diagnostic.name == "/Robot Diagnostics/Electronics/Voltages and Currents/V+ Rail Voltage"){
                bool found = false;
                voltageConfig.text_ = "00.00 V";
                voltageConfig.text_color_ = QColor(255, 0, 0, 255);
                
                for(auto pair : diagnostic.values){
                    if(pair.key == "V+ Rail Voltage"){
                        found = true;
                        voltageConfig.text_ = pair.value;
                    }
                }

                if(!found){
                    voltageConfig.text_ = "BAD CONV";
                }

                // now we need to look at the status of the voltage to determine color
                // ok is green, warn is yellow, error is red
                if(diagnostic.level == diagnostic.ERROR){
                    voltageConfig.text_color_ = QColor(255, 0, 0, 255);
                } else if (diagnostic.level == diagnostic.WARN){
                    voltageConfig.text_color_ = QColor(255, 255, 0, 255);
                } else if (diagnostic.level == diagnostic.STALE){
                    diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
                } else {
                    voltageConfig.text_color_ = QColor(0, 255, 0, 255);
                }
                

                // edit the text
                updateText(voltageTextId, voltageConfig);
            }

            // handle general packet for LED
            else if(diagnostic.name == "/Robot Diagnostics"){
                // Determine the LED color to use
                if(diagnostic.level == diagnostic.ERROR){
                    diagLedConfig.inner_color_ = QColor(255, 0, 0, 255);
                } else if (diagnostic.level == diagnostic.WARN){
                    diagLedConfig.inner_color_ = QColor(255, 255, 0, 255);
                } else if (diagnostic.level == diagnostic.STALE){
                    diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
                } else {
                    diagLedConfig.inner_color_ = QColor(0, 255, 0, 255);
                }

                updateCircle(diagLedConfigId, diagLedConfig);
            }
        }
    }

    void DiagnosticOverlay::zedCallback(const sensor_msgs::msg::Temperature& msg) {
        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        zedTimedOut = false;

        zedLedConfig.inner_color_ = QColor(0, 255, 0, 255);
        updateCircle(zedLedConfigId, zedLedConfig);

        lastZed = node->get_clock()->now();
    }

    void DiagnosticOverlay::leakCallback(const std_msgs::msg::Bool& msg) {
        static bool redFlash = true;

        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        leakTimedOut = false;

        // get our local rosnode
        if (msg.data) {
            if (redFlash) {
                leakLedConfig.inner_color_ = QColor(255, 0, 0, 255);  // Flash red
                redFlash = false;
            }
            else {
                leakLedConfig.inner_color_ = QColor(252, 126, 0, 255);  // Flash orange
                redFlash = true;
            }
            
            if (!startedLeaking) {
                // Pop up an annoying box
                QMessageBox msgBox;
                msgBox.setWindowTitle("LEAK");
                msgBox.setIcon(QMessageBox::Warning);
                msgBox.setText(QString::fromStdString("Water was detected in one of the cages."));

                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.setDefaultButton(QMessageBox::Ok);
                msgBox.exec();  
                
                startedLeaking = true;
            }

        }
        else {
            leakLedConfig.inner_color_ = QColor(0, 255, 0, 255);
            startedLeaking = false;
        }

        updateCircle(leakLedConfigId, leakLedConfig);
        lastLeak = node->get_clock()->now();
    }

    void DiagnosticOverlay::killCallback(const std_msgs::msg::Bool & msg){
        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        // write down the time that we recieve the last kill msg
        lastKill = node->get_clock()->now();
        killTimedOut = false;

        if(!msg.data){
            killLedConfig.inner_color_ = QColor(0, 255, 0, 255);
        } else {
            killLedConfig.inner_color_ = QColor(255, 0, 0, 255);
        }
        updateCircle(killLedConfigId, killLedConfig);
    }

    void DiagnosticOverlay::updateFont() {
        int font_index = fontProperty->getOptionInt();
        if (font_index < fontFamilies.size()) {
            fontName = fontFamilies[font_index].toStdString();
        } else {
            RVIZ_COMMON_LOG_ERROR_STREAM("DiagnosticOverlay: Unexpected error at selecting font index " << font_index);
            return;
        }


        require_update_texture_ = true;
    }
    
    void DiagnosticOverlay::onEnable(){
        OverlayDisplay::onEnable();
    }
    
    void DiagnosticOverlay::onDisable(){
        OverlayDisplay::onDisable();
    }
    
    void DiagnosticOverlay::update(float wall_dt, float ros_dt){
        OverlayDisplay::update(wall_dt, ros_dt);
    }

    void DiagnosticOverlay::checkTimeout(){
        // read current timeout property and convert to duration
        auto timeoutDur = std::chrono::duration<double>(timeoutProperty->getFloat());

        // get our local rosnode
        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        // check for diagnostic timeout
        auto duration = node->get_clock()->now() - lastDiag;
        if(duration > timeoutDur){
            if(! diagsTimedOut){
                RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Diagnostics timed out!");
                diagsTimedOut = true;
            }

            // diagnostics timed out, reset them
            voltageConfig.text_color_ = QColor(255, 0, 255, 255);
            updateText(voltageTextId, voltageConfig);

            diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
            updateCircle(diagLedConfigId, diagLedConfig);
        }

        // check for kill timeout
        duration = node->get_clock()->now() - lastKill;
        if(duration > timeoutDur){
            if(! killTimedOut){
                RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Kill switch timed out!");
                killTimedOut = true;
            }

            // diagnostics timed out, reset them
            killLedConfig.inner_color_ = QColor(255, 0, 255, 255);
            updateCircle(killLedConfigId, killLedConfig);
        }

        duration = node->get_clock()->now() - lastZed;
        if (duration > std::chrono::duration<double>(1.0f)) {
            if (!zedTimedOut) {
                RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Zed connection timed out!");
                zedTimedOut = true;
            }

            zedLedConfig.inner_color_ = QColor(255, 0, 0, 255);
            updateCircle(zedLedConfigId, zedLedConfig);
        }

        duration = node->get_clock()->now() - lastLeak;
        if (duration > std::chrono::duration<double>(2.0s)) {
            if (!leakTimedOut) {
                RVIZ_COMMON_LOG_WARNING("DiagnosticsOverlay: Leak sensors timed out!");
                leakTimedOut = true;
            }

            leakLedConfig.inner_color_ = QColor(255, 0, 255, 255);
            updateCircle(leakLedConfigId, leakLedConfig);
        }
    }

    void DiagnosticOverlay::reset(){
        OverlayDisplay::reset();
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(riptide_rviz::DiagnosticOverlay, rviz_common::Display)