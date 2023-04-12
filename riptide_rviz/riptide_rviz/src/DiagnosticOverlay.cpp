#include "riptide_rviz/DiagnosticOverlay.hpp"

#include <QFontDatabase>
#include <rviz_common/logging.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace riptide_rviz
{
    DiagnosticOverlay::DiagnosticOverlay(){
        // make the ROS node
        auto options = rclcpp::NodeOptions().arguments({});
        nodeHandle = std::make_shared<rclcpp::Node>("riptide_rviz_diag_overlay", options);

        // backdate timeouts
        lastDiag = nodeHandle->get_clock()->now() - 1h;
        lastKill = nodeHandle->get_clock()->now() - 1h;

        // init font parameter
        QFontDatabase database;
        fontFamilies = database.families();
        fontProperty = new rviz_common::properties::EnumProperty("font", "DejaVu Sans Mono", "font", this, SLOT(updateFont()));
        for (ssize_t i = 0; i < fontFamilies.size(); i++) {
            fontProperty->addOption(fontFamilies[i], (int) i);
        }

        robotNsProperty = new rviz_common::properties::StringProperty(
            "robot_namespace", "/tempest", "Robot namespace to attach to", this, SLOT(updateNS()));

        timeoutProperty = new rviz_common::properties::FloatProperty(
            "diagnostic_timeout", 10.0, "Maximum time between diagnostic packets before default", this);


    }

    DiagnosticOverlay::~DiagnosticOverlay(){
        delete robotNsProperty;
        delete timeoutProperty;
    }

    void DiagnosticOverlay::onInitialize(){
        OverlayDisplay::onInitialize();

        // make the diagnostic subscriber
        diagSub = nodeHandle->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics_agg", rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlay::diagnosticCallback, this, _1)
        );

        // watchdog timers for handling timeouts
        checkTimer = nodeHandle->create_wall_timer(0.25s, std::bind(&DiagnosticOverlay::checkTimeout, this));

        // add all of the variable design items
        voltageConfig.text_color_ = QColor(255, 0, 255, 255);
        voltageTextId = addText(voltageConfig);

        diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        diagLedConfigId = addCircle(diagLedConfig);

        killLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        killLedConfigId = addCircle(killLedConfig);

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
        addText(diagLedLabel);
        addText(killLedLabel);
    }

    void DiagnosticOverlay::updateNS(){
        RVIZ_COMMON_LOG_INFO_STREAM("Robot NS update " << robotNsProperty->getStdString());
        killSub.reset();
        killSub = nodeHandle->create_subscription<std_msgs::msg::Bool>(
            robotNsProperty->getStdString() + "/state/kill", rclcpp::SystemDefaultsQoS(),
            std::bind(&DiagnosticOverlay::killCallback, this, _1)
        );
    }

    void DiagnosticOverlay::diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg){
        // write down the timestamp that it was recieved
        lastDiag = nodeHandle->get_clock()->now();
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

    void DiagnosticOverlay::killCallback(const std_msgs::msg::Bool & msg){
        // write down the time that we recieve the last kill msg
        lastKill = nodeHandle->get_clock()->now();
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
            RVIZ_COMMON_LOG_ERROR_STREAM("Unexpected error at selecting font index " << font_index);
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
        rclcpp::spin_some(nodeHandle);

        OverlayDisplay::update(wall_dt, ros_dt);
    }

    void DiagnosticOverlay::checkTimeout(){
        // read current timeout property and convert to duration
        auto timeoutDur = std::chrono::duration<double>(timeoutProperty->getFloat());

        // check for diagnostic timeout
        auto duration = nodeHandle->get_clock()->now() - lastDiag;
        if(duration > timeoutDur){
            if(! diagsTimedOut){
                RVIZ_COMMON_LOG_WARNING("Diagnostics timed out!");
                diagsTimedOut = true;
            }

            // diagnostics timed out, reset them
            voltageConfig.text_color_ = QColor(255, 0, 255, 255);
            updateText(voltageTextId, voltageConfig);

            diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
            updateCircle(diagLedConfigId, diagLedConfig);
        }

        // check for kill timeout
        duration = nodeHandle->get_clock()->now() - lastKill;
        if(duration > timeoutDur){
            if(! killTimedOut){
                RVIZ_COMMON_LOG_WARNING("Kill switch timed out!");
                killTimedOut = true;
            }

            // diagnostics timed out, reset them
            killLedConfig.inner_color_ = QColor(255, 0, 255, 255);
            updateCircle(killLedConfigId, killLedConfig);
        }
    }

    void DiagnosticOverlay::reset(){
        OverlayDisplay::reset();
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(riptide_rviz::DiagnosticOverlay, rviz_common::Display)