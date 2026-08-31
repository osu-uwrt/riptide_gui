#include "riptide_rviz/ServicePanel.hpp"

#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPointer>
#include <QPushButton>
#include <QScrollArea>
#include <QStringList>
#include <QTimer>

#include <algorithm>
#include <map>

namespace
{
    // Minimal tree used to reassemble nested YAML from the form's dotted field paths
    // (e.g. "pose.pose.position.x" -> {pose: {pose: {position: {x: ...}}}}).
    struct YamlNode
    {
        std::map<QString, YamlNode> children;
        QString leaf;
        bool isLeaf = false;
    };

    void insertYaml(YamlNode &root, const QStringList &path, const QString &value)
    {
        YamlNode *cur = &root;
        for (const QString &key : path)
        {
            cur = &cur->children[key];
        }
        cur->isLeaf = true;
        cur->leaf = value;
    }

    QString serializeYaml(const YamlNode &node)
    {
        if (node.isLeaf)
        {
            return node.leaf;
        }
        QStringList parts;
        for (const auto &kv : node.children)
        {
            parts << (kv.first + ": " + serializeYaml(kv.second));
        }
        return "{" + parts.join(", ") + "}";
    }
} // namespace

namespace riptide_rviz
{
    // services that hang waiting on an absent server would otherwise lock the panel
    static constexpr int CALL_TIMEOUT_MS = 15000;

    ServicePanel::ServicePanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_ServicePanel();
        ui->setupUi(this);

        // start in flat-form mode with the raw YAML editor hidden
        ui->yamlEdit->setVisible(false);
    }

    ServicePanel::~ServicePanel()
    {
        delete ui;
    }

    void ServicePanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);

        config.mapGetString("robot_namespace", &robotNs);
        if (robotNs == "")
        {
            robotNs = "/talos";
            RVIZ_COMMON_LOG_WARNING("ServicePanel: Using /talos as the default value for robot_namespace");
        }

        // populate the dropdown with whatever services are up right now
        refreshServices();
    }

    void ServicePanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }

    void ServicePanel::onInitialize()
    {
        connect(ui->refreshButton, &QPushButton::clicked, this, [this]() { refreshServices(); });
        connect(ui->inspectButton, &QPushButton::clicked, this, [this]() { inspectService(); });
        connect(ui->callButton, &QPushButton::clicked, this, [this]() { callService(); });

        // picking a service from the dropdown auto-fills its type and rebuilds the form
        connect(ui->serviceCombo, QOverload<int>::of(&QComboBox::activated), this,
                [this](int) { onServiceSelected(); });

        // toggling the filter just repopulates the dropdown
        connect(ui->hideParamServices, &QCheckBox::toggled, this, [this](bool) { refreshServices(); });
    }

    void ServicePanel::refreshServices()
    {
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        const auto namesAndTypes = node->get_service_names_and_types();

        // keep whatever the user has currently typed/selected
        const QString current = ui->serviceCombo->currentText();

        serviceTypes.clear();
        ui->serviceCombo->clear();

        std::vector<std::string> names;
        names.reserve(namesAndTypes.size());
        for (const auto &pair : namesAndTypes)
        {
            if (!pair.second.empty())
            {
                serviceTypes[pair.first] = pair.second.front();
            }
            names.push_back(pair.first);
        }
        std::sort(names.begin(), names.end());

        // optionally hide the per-node parameter/logger services (rcl_interfaces) that
        // otherwise flood the list. We still keep them in serviceTypes so a manually
        // typed name can resolve its type.
        const bool hideParam = ui->hideParamServices->isChecked();
        int shown = 0;
        for (const auto &name : names)
        {
            const auto it = serviceTypes.find(name);
            if (hideParam && it != serviceTypes.end() &&
                QString::fromStdString(it->second).startsWith("rcl_interfaces/"))
            {
                continue;
            }
            ui->serviceCombo->addItem(QString::fromStdString(name));
            shown++;
        }

        ui->serviceCombo->setCurrentText(current);

        if (shown == static_cast<int>(names.size()))
        {
            setStatus(QString("Found %1 service(s).").arg(static_cast<int>(names.size())), false);
        }
        else
        {
            setStatus(QString("Showing %1 of %2 service(s) (%3 rcl_interfaces hidden).")
                          .arg(shown)
                          .arg(static_cast<int>(names.size()))
                          .arg(static_cast<int>(names.size()) - shown),
                      false);
        }
    }

    void ServicePanel::onServiceSelected()
    {
        const std::string name = ui->serviceCombo->currentText().toStdString();
        const auto it = serviceTypes.find(name);
        if (it != serviceTypes.end())
        {
            ui->typeEdit->setText(QString::fromStdString(it->second));
            inspectService();
        }
    }

    void ServicePanel::inspectService()
    {
        QString type = ui->typeEdit->text().trimmed();
        if (type.isEmpty())
        {
            // fall back to the type the graph reported for the chosen service
            const auto it = serviceTypes.find(ui->serviceCombo->currentText().trimmed().toStdString());
            if (it != serviceTypes.end())
            {
                type = QString::fromStdString(it->second);
                ui->typeEdit->setText(type);
            }
        }

        if (type.isEmpty())
        {
            setStatus("No service type to inspect - pick a service or type its type (pkg/srv/Type).", true);
            return;
        }

        if (inspectProc)
        {
            return; // an inspect is already running
        }

        setStatus("Inspecting " + type + " ...", false);

        QProcess *proc = new QProcess(this);
        inspectProc = proc;

        connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
                [this, proc](int, QProcess::ExitStatus) {
                    const QString out = QString::fromLocal8Bit(proc->readAllStandardOutput());
                    const QString err = QString::fromLocal8Bit(proc->readAllStandardError());
                    proc->deleteLater();
                    if (inspectProc == proc)
                    {
                        inspectProc = nullptr;
                    }

                    if (out.trimmed().isEmpty())
                    {
                        setStatus("Could not inspect service type: " + err.trimmed(), true);
                        return;
                    }

                    buildForm(out);
                });

        connect(proc, &QProcess::errorOccurred, this, [this, proc](QProcess::ProcessError e) {
            if (e == QProcess::FailedToStart)
            {
                proc->deleteLater();
                if (inspectProc == proc)
                {
                    inspectProc = nullptr;
                }
                setStatus("Failed to run 'ros2' - is it installed and sourced?", true);
            }
        });

        proc->start("ros2", QStringList{"interface", "show", type});
    }

    void ServicePanel::buildForm(const QString &interfaceText)
    {
        clearForm();
        yamlMode = false;

        // ros2 interface show prints the fully-expanded request tree, one tab of
        // indentation per nesting level. We walk it keeping a stack of ancestor field
        // names so each primitive leaf becomes a box labelled with its dotted path.
        const QStringList lines = interfaceText.split('\n');
        QStringList requestRawLines;
        QStringList prefix;       // ancestor field names for the current depth
        int arraySkipDepth = -1;  // while >=0, skip an array's expanded element template
        bool sawAny = false;      // did the request section have any parseable field

        struct Field
        {
            QString path;
            QString type; // base type for primitives, full "type[..]" for arrays
            bool isArray;
        };
        std::vector<Field> fields;

        for (const QString &rawLine : lines)
        {
            if (rawLine.trimmed() == "---")
            {
                break; // start of the response section
            }
            requestRawLines << rawLine;

            // indentation depth = number of leading tabs
            int depth = 0;
            while (depth < rawLine.size() && rawLine[depth] == '\t')
            {
                depth++;
            }

            // drop trailing comments and surrounding whitespace
            QString line = rawLine;
            const int hash = line.indexOf('#');
            if (hash >= 0)
            {
                line = line.left(hash);
            }
            line = line.trimmed();
            if (line.isEmpty())
            {
                continue;
            }

            // skip the expanded element fields that follow an array declaration
            if (arraySkipDepth >= 0)
            {
                if (depth > arraySkipDepth)
                {
                    continue;
                }
                arraySkipDepth = -1;
            }

            const QStringList toks = line.simplified().split(' ');
            if (toks.size() < 2)
            {
                continue; // not a "type name" field line
            }

            const QString type = toks[0];
            const QString fieldName = toks[1];

            // skip constants e.g. "int8 FOO=1"
            if (fieldName.contains('='))
            {
                continue;
            }
            sawAny = true;

            // ancestors of this field == its depth, trim the stack back to that
            while (prefix.size() > depth)
            {
                prefix.removeLast();
            }
            const QString path = prefix.isEmpty() ? fieldName : (prefix.join('.') + "." + fieldName);

            const bool isArray = type.contains('[');
            QString baseType = type;
            const int bracket = baseType.indexOf('[');
            if (bracket >= 0)
            {
                baseType = baseType.left(bracket);
            }

            if (isArray)
            {
                // one box for the whole array; ignore its expanded element structure
                fields.push_back({path, type, true});
                arraySkipDepth = depth;
            }
            else if (isPrimitiveType(baseType))
            {
                fields.push_back({path, baseType, false});
            }
            else
            {
                // nested message: descend so children get prefixed with this name
                prefix.append(fieldName);
            }
        }

        if (fields.empty())
        {
            if (sawAny)
            {
                // unexpected structure we couldn't break into boxes; fall back to raw YAML
                yamlMode = true;
                ui->formScroll->setVisible(false);
                ui->yamlEdit->setVisible(true);
                ui->yamlEdit->setPlaceholderText("Enter the request as YAML, e.g.:\n\n" +
                                                 requestRawLines.join('\n'));
                setStatus("Could not auto-build a form - enter the request as YAML below.", false);
            }
            else
            {
                ui->yamlEdit->setVisible(false);
                ui->formScroll->setVisible(true);
                setStatus("This service takes no request fields - just press Call Service.", false);
            }
            return;
        }

        ui->yamlEdit->setVisible(false);
        ui->formScroll->setVisible(true);

        for (const auto &f : fields)
        {
            QWidget *w = nullptr;
            if (!f.isArray && f.type == "bool")
            {
                w = new QCheckBox(ui->formContainer);
            }
            else
            {
                QLineEdit *le = new QLineEdit(ui->formContainer);
                le->setPlaceholderText(f.isArray ? (f.type + "  e.g. [1, 2, 3]") : f.type);
                w = le;
            }
            ui->formLayout->addRow(f.path + "  (" + f.type + ")", w);
            fieldWidgets.push_back({f.path, f.type, w});
        }

        setStatus(QString("Ready - %1 field(s). Fill them in and press Call Service.")
                      .arg(static_cast<int>(fields.size())),
                  false);
    }

    void ServicePanel::clearForm()
    {
        // removeRow deletes both the label and the field widget
        while (ui->formLayout->rowCount() > 0)
        {
            ui->formLayout->removeRow(0);
        }
        fieldWidgets.clear();
    }

    void ServicePanel::callService()
    {
        const QString name = ui->serviceCombo->currentText().trimmed();
        const QString type = ui->typeEdit->text().trimmed();

        if (name.isEmpty())
        {
            setStatus("Enter or select a service name first.", true);
            return;
        }
        if (type.isEmpty())
        {
            setStatus("Service type is empty - press Inspect or type it (pkg/srv/Type).", true);
            return;
        }
        if (callProc)
        {
            setStatus("A service call is already in progress.", true);
            return;
        }

        const QString request = buildRequestYaml();

        ui->responseText->setPlainText("$ ros2 service call " + name + " " + type + " \"" + request + "\"\n");
        setStatus("Calling " + name + " ...", false);
        ui->callButton->setEnabled(false);

        QProcess *proc = new QProcess(this);
        proc->setProcessChannelMode(QProcess::MergedChannels);
        callProc = proc;

        connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
                [this, proc, name](int exitCode, QProcess::ExitStatus) {
                    const QString out = QString::fromLocal8Bit(proc->readAll());
                    proc->deleteLater();
                    if (callProc == proc)
                    {
                        callProc = nullptr;
                    }
                    ui->callButton->setEnabled(true);

                    ui->responseText->appendPlainText(out);
                    if (exitCode == 0)
                    {
                        setStatus("Call to " + name + " complete.", false);
                    }
                    else
                    {
                        setStatus("Call to " + name + " failed (exit " + QString::number(exitCode) + ").", true);
                    }
                });

        connect(proc, &QProcess::errorOccurred, this, [this, proc](QProcess::ProcessError e) {
            if (e == QProcess::FailedToStart)
            {
                proc->deleteLater();
                if (callProc == proc)
                {
                    callProc = nullptr;
                }
                ui->callButton->setEnabled(true);
                setStatus("Failed to run 'ros2' - is it installed and sourced?", true);
            }
        });

        proc->start("ros2", QStringList{"service", "call", name, type, request});

        // guard against a call that never returns (e.g. server never comes up)
        QPointer<QProcess> guard(proc);
        QTimer::singleShot(CALL_TIMEOUT_MS, this, [this, guard, proc]() {
            if (guard && callProc == proc && proc->state() != QProcess::NotRunning)
            {
                ui->responseText->appendPlainText("\n[timed out, killing call]");
                proc->kill(); // triggers finished -> cleanup
            }
        });
    }

    QString ServicePanel::buildRequestYaml()
    {
        if (yamlMode)
        {
            const QString y = ui->yamlEdit->toPlainText().trimmed();
            return y.isEmpty() ? "{}" : y;
        }

        if (fieldWidgets.empty())
        {
            return "{}";
        }

        // rebuild the nested request from each field's dotted path
        YamlNode root;
        for (const auto &f : fieldWidgets)
        {
            const bool isArray = f.type.contains('[');
            QString valueStr;
            bool include = true;

            if (!isArray && f.type == "bool")
            {
                const QCheckBox *cb = qobject_cast<QCheckBox *>(f.widget);
                valueStr = (cb && cb->isChecked()) ? "true" : "false";
            }
            else
            {
                const QLineEdit *le = qobject_cast<QLineEdit *>(f.widget);
                const QString text = le ? le->text().trimmed() : QString();
                if (isArray)
                {
                    // can't guess a correct-length array, so omit empties (use defaults)
                    if (text.isEmpty())
                    {
                        include = false;
                    }
                    else
                    {
                        valueStr = text; // user-entered YAML list, verbatim
                    }
                }
                else if (f.type == "string" || f.type == "wstring")
                {
                    QString escaped = le ? le->text() : QString();
                    escaped.replace("'", "''"); // single-quote escaping for YAML
                    valueStr = "'" + escaped + "'";
                }
                else
                {
                    // numeric: empty defaults to 0, otherwise pass through as typed
                    valueStr = text.isEmpty() ? "0" : text;
                }
            }

            if (include)
            {
                insertYaml(root, f.name.split('.'), valueStr);
            }
        }
        return serializeYaml(root);
    }

    void ServicePanel::setStatus(const QString &status, bool error)
    {
        ui->statusLabel->setText(status);
        ui->statusLabel->setStyleSheet(error ? "QLabel { color: red; }" : "");
    }

    bool ServicePanel::isPrimitiveType(const QString &t)
    {
        static const QStringList prims = {
            "bool", "byte", "char",
            "float32", "float64",
            "int8", "uint8", "int16", "uint16",
            "int32", "uint32", "int64", "uint64",
            "string", "wstring"};
        return prims.contains(t);
    }
} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ServicePanel, rviz_common::Panel);
