#pragma once
#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>

#include <QString>
#include <QProcess>
#include <QWidget>

#include <map>
#include <string>
#include <vector>
#include <utility>

#include "ui_ServicePanel.h"

namespace riptide_rviz
{
    // Generic panel for calling any ROS service from RVIZ. The request form is built
    // dynamically from the selected service's type, so the visible fields depend on the
    // service (e.g. a std_srvs/SetBool shows one bool, an AddTwoInts shows two ints).
    class ServicePanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        ServicePanel(QWidget *parent = 0);
        ~ServicePanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

    private:
        // one editable field of the request form
        struct FieldWidget
        {
            QString name;
            QString type;
            QWidget *widget; // QCheckBox for bool, otherwise QLineEdit
        };

        void refreshServices();
        void onServiceSelected();
        void inspectService();
        void buildForm(const QString &interfaceText);
        void clearForm();
        void callService();
        QString buildRequestYaml();
        void setStatus(const QString &status, bool error = false);

        static bool isPrimitiveType(const QString &t);

        Ui_ServicePanel *ui;

        // robot namespace, kept for config save/load parity with the other panels
        QString robotNs;

        // discovered service name -> type as reported by the ROS graph
        std::map<std::string, std::string> serviceTypes;

        // widgets currently making up the request form
        std::vector<FieldWidget> fieldWidgets;

        // true when the request was too complex for a flat form and the raw YAML box is used
        bool yamlMode = false;

        // child processes for the ros2 CLI calls (null when idle)
        QProcess *inspectProc = nullptr;
        QProcess *callProc = nullptr;
    };
} // namespace riptide_rviz
