#include "riptide_rviz/RosbagPanel.hpp"

#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStringList>

#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <set>
#include <vector>

namespace riptide_rviz
{
    RosbagPanel::RosbagPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_RosbagPanel();
        ui->setupUi(this);

        // default the output directory to the user's home
        const char *home = std::getenv("HOME");
        ui->outputDirEdit->setText(home ? QString::fromLocal8Bit(home) : ".");

        // record button starts in the "ready to record" (green) state
        ui->recordButton->setStyleSheet("QPushButton{color:white; background: green;}");

        elapsedTimer = new QTimer(this);
        elapsedTimer->setInterval(1000);
        connect(elapsedTimer, &QTimer::timeout, this, [this]() { updateElapsed(); });
    }

    RosbagPanel::~RosbagPanel()
    {
        // finalize a running recording so the bag isn't left corrupt
        if (isRecording())
        {
            ::kill(static_cast<pid_t>(recordProc->processId()), SIGINT);
            recordProc->waitForFinished(3000);
        }
        delete ui;
    }

    void RosbagPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);

        config.mapGetString("robot_namespace", &robotNs);
        if (robotNs == "")
        {
            robotNs = "/talos";
            RVIZ_COMMON_LOG_WARNING("RosbagPanel: Using /talos as the default value for robot_namespace");
        }

        // presets resolve relative topics against robotNs, so load them after it is known
        loadPresets();
        refreshTopics();
    }

    void RosbagPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("robot_namespace", robotNs);
    }

    void RosbagPanel::onInitialize()
    {
        // Refresh re-reads both the live topics and the presets file, so edits to
        // rosbag_presets.yaml show up without restarting RViz
        connect(ui->refreshButton, &QPushButton::clicked, this, [this]() {
            loadPresets();
            refreshTopics();
        });
        connect(ui->recordButton, &QPushButton::clicked, this, [this]() { toggleRecording(); });
        connect(ui->clearButton, &QPushButton::clicked, this, [this]() {
            clearTopicSelection();
            ui->presetCombo->setCurrentIndex(0);
        });
        connect(ui->filterEdit, &QLineEdit::textChanged, this, [this](const QString &) { applyTopicFilter(); });
        connect(ui->recordAllCheck, &QCheckBox::toggled, this, [this](bool checked) {
            // when recording everything the explicit topic list/preset are irrelevant
            ui->topicList->setEnabled(!checked);
            ui->filterEdit->setEnabled(!checked);
            ui->presetCombo->setEnabled(!checked);
        });
        connect(ui->presetCombo, QOverload<int>::of(&QComboBox::activated), this,
                [this](int index) { applyPreset(index); });
    }

    void RosbagPanel::loadPresets()
    {
        presets.clear();
        ui->presetCombo->clear();
        ui->presetCombo->addItem("Custom / None");

        std::string path;
        try
        {
            path = ament_index_cpp::get_package_share_directory("riptide_rviz") + "/config/rosbag_presets.yaml";
        }
        catch (const std::exception &e)
        {
            RVIZ_COMMON_LOG_WARNING("RosbagPanel: could not locate package share dir for presets");
            return;
        }

        try
        {
            const YAML::Node root = YAML::LoadFile(path);
            const YAML::Node node = root["presets"];
            if (!node || !node.IsMap())
            {
                RVIZ_COMMON_LOG_WARNING("RosbagPanel: no 'presets' map found in " + path);
                return;
            }

            for (auto it = node.begin(); it != node.end(); ++it)
            {
                const QString presetName = QString::fromStdString(it->first.as<std::string>());
                QStringList topics;
                for (const auto &t : it->second)
                {
                    topics << QString::fromStdString(resolveTopic(t.as<std::string>()));
                }
                presets.push_back({presetName, topics});
                ui->presetCombo->addItem(presetName);
            }
        }
        catch (const std::exception &e)
        {
            RVIZ_COMMON_LOG_WARNING(std::string("RosbagPanel: failed to load presets: ") + e.what());
        }
    }

    std::string RosbagPanel::resolveTopic(const std::string &raw) const
    {
        if (!raw.empty() && raw.front() == '/')
        {
            return raw; // absolute, use verbatim
        }
        return robotNs.toStdString() + "/" + raw;
    }

    QListWidgetItem *RosbagPanel::ensureTopicItem(const QString &name)
    {
        const QList<QListWidgetItem *> matches = ui->topicList->findItems(name, Qt::MatchExactly);
        if (!matches.isEmpty())
        {
            return matches.first();
        }

        // preset/manual topic that isn't currently published - add it so it still records
        QListWidgetItem *item = new QListWidgetItem(name, ui->topicList);
        item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
        item->setCheckState(Qt::Unchecked);
        return item;
    }

    void RosbagPanel::applyPreset(int comboIndex)
    {
        // index 0 is the "Custom / None" placeholder
        if (comboIndex <= 0 || comboIndex > static_cast<int>(presets.size()))
        {
            return;
        }

        // a specific topic set implies we are not in record-all mode
        if (ui->recordAllCheck->isChecked())
        {
            ui->recordAllCheck->setChecked(false);
        }

        // start clean so the preset is exactly what gets recorded (still editable after)
        clearTopicSelection();

        const QStringList &topics = presets[comboIndex - 1].second;
        for (const QString &t : topics)
        {
            ensureTopicItem(t)->setCheckState(Qt::Checked);
        }

        applyTopicFilter();
        setStatus(QString("Applied preset '%1' (%2 topic(s)). You can still tweak the list.")
                      .arg(presets[comboIndex - 1].first)
                      .arg(topics.size()),
                  false);
    }

    void RosbagPanel::clearTopicSelection()
    {
        for (int i = 0; i < ui->topicList->count(); i++)
        {
            ui->topicList->item(i)->setCheckState(Qt::Unchecked);
        }
    }

    void RosbagPanel::refreshTopics()
    {
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        const auto topicsAndTypes = node->get_topic_names_and_types();

        // preserve the user's current selection across a refresh
        std::set<QString> checked;
        for (int i = 0; i < ui->topicList->count(); i++)
        {
            QListWidgetItem *item = ui->topicList->item(i);
            if (item->checkState() == Qt::Checked)
            {
                checked.insert(item->text());
            }
        }

        ui->topicList->clear();

        // union of live topics and anything currently checked, so selected-but-offline
        // topics (e.g. from a preset) survive a refresh and still get recorded
        std::set<std::string> names;
        for (const auto &pair : topicsAndTypes)
        {
            names.insert(pair.first);
        }
        for (const QString &c : checked)
        {
            names.insert(c.toStdString());
        }

        for (const auto &name : names)
        {
            const QString qname = QString::fromStdString(name);
            QListWidgetItem *item = new QListWidgetItem(qname, ui->topicList);
            item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
            item->setCheckState(checked.count(qname) ? Qt::Checked : Qt::Unchecked);
        }

        applyTopicFilter();
        setStatus(QString("Found %1 topic(s).").arg(static_cast<int>(topicsAndTypes.size())), false);
    }

    void RosbagPanel::applyTopicFilter()
    {
        const QString filter = ui->filterEdit->text();
        for (int i = 0; i < ui->topicList->count(); i++)
        {
            QListWidgetItem *item = ui->topicList->item(i);
            item->setHidden(!filter.isEmpty() && !item->text().contains(filter, Qt::CaseInsensitive));
        }
    }

    void RosbagPanel::toggleRecording()
    {
        if (isRecording())
        {
            stopRecording();
        }
        else
        {
            startRecording();
        }
    }

    void RosbagPanel::startRecording()
    {
        const bool all = ui->recordAllCheck->isChecked();

        // collect the checked topics if not recording everything
        QStringList topics;
        if (!all)
        {
            for (int i = 0; i < ui->topicList->count(); i++)
            {
                QListWidgetItem *item = ui->topicList->item(i);
                if (item->checkState() == Qt::Checked)
                {
                    topics << item->text();
                }
            }
            if (topics.isEmpty())
            {
                setStatus("Select at least one topic, or check 'Record all topics'.", true);
                return;
            }
        }

        QString dir = ui->outputDirEdit->text().trimmed();
        if (dir.isEmpty())
        {
            const char *home = std::getenv("HOME");
            dir = home ? QString::fromLocal8Bit(home) : ".";
        }

        // build: ros2 bag record [-o name] (-a | topic...)
        QStringList args;
        args << "bag" << "record";

        const QString name = ui->bagNameEdit->text().trimmed();
        if (!name.isEmpty())
        {
            args << "-o" << name;
        }

        if (all)
        {
            args << "-a";
        }
        else
        {
            args << topics;
        }

        recordProc = new QProcess(this);
        recordProc->setProcessChannelMode(QProcess::MergedChannels);
        recordProc->setWorkingDirectory(dir);

        connect(recordProc, &QProcess::readyReadStandardOutput, this, [this]() {
            if (recordProc)
            {
                ui->outputText->appendPlainText(QString::fromLocal8Bit(recordProc->readAll()).trimmed());
            }
        });

        connect(recordProc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
                [this, dir](int exitCode, QProcess::ExitStatus) {
                    recordProc->deleteLater();
                    recordProc = nullptr;

                    elapsedTimer->stop();
                    setInputsEnabled(true);
                    ui->recordButton->setEnabled(true);
                    ui->recordButton->setText("Record");
                    ui->recordButton->setStyleSheet("QPushButton{color:white; background: green;}");

                    if (exitCode == 0)
                    {
                        setStatus("Recording stopped. Bag saved under " + dir, false);
                    }
                    else
                    {
                        setStatus("ros2 bag record exited with code " + QString::number(exitCode) + ".", true);
                    }
                });

        connect(recordProc, &QProcess::errorOccurred, this, [this](QProcess::ProcessError e) {
            if (e == QProcess::FailedToStart)
            {
                recordProc->deleteLater();
                recordProc = nullptr;
                elapsedTimer->stop();
                setInputsEnabled(true);
                ui->recordButton->setEnabled(true);
                ui->recordButton->setText("Record");
                ui->recordButton->setStyleSheet("QPushButton{color:white; background: green;}");
                setStatus("Failed to run 'ros2' - is it installed and sourced?", true);
            }
        });

        ui->outputText->clear();
        ui->outputText->appendPlainText("$ ros2 " + args.join(' '));
        recordProc->start("ros2", args);

        recordStartMs = QDateTime::currentMSecsSinceEpoch();
        elapsedTimer->start();

        // lock down the config inputs and flip the button to a stop control
        setInputsEnabled(false);
        ui->recordButton->setText("Stop Recording");
        ui->recordButton->setStyleSheet("QPushButton{color:black; background: red;}");
        setStatus("Recording...", false);
    }

    void RosbagPanel::stopRecording()
    {
        if (!isRecording())
        {
            return;
        }

        setStatus("Stopping (finalizing bag)...", false);

        // SIGINT lets ros2 bag record shut down cleanly and close the bag, like Ctrl-C.
        // The finished handler re-enables the UI once it actually exits.
        ui->recordButton->setEnabled(false);
        ::kill(static_cast<pid_t>(recordProc->processId()), SIGINT);
    }

    void RosbagPanel::updateElapsed()
    {
        if (!isRecording())
        {
            return;
        }

        const int64_t elapsed = (QDateTime::currentMSecsSinceEpoch() - recordStartMs) / 1000;
        const int64_t h = elapsed / 3600;
        const int64_t m = (elapsed % 3600) / 60;
        const int64_t s = elapsed % 60;

        const QString t = QString("%1:%2:%3")
                              .arg(h, 2, 10, QChar('0'))
                              .arg(m, 2, 10, QChar('0'))
                              .arg(s, 2, 10, QChar('0'));
        setStatus("Recording  " + t, false);
    }

    void RosbagPanel::setInputsEnabled(bool enabled)
    {
        ui->outputDirEdit->setEnabled(enabled);
        ui->bagNameEdit->setEnabled(enabled);
        ui->recordAllCheck->setEnabled(enabled);
        ui->refreshButton->setEnabled(enabled);
        // topic list/filter also respect the record-all toggle
        const bool listEnabled = enabled && !ui->recordAllCheck->isChecked();
        ui->topicList->setEnabled(listEnabled);
        ui->filterEdit->setEnabled(listEnabled);
    }

    void RosbagPanel::setStatus(const QString &status, bool error)
    {
        ui->statusLabel->setText(status);
        ui->statusLabel->setStyleSheet(error ? "QLabel { color: red; }" : "");
    }

    bool RosbagPanel::isRecording() const
    {
        return recordProc != nullptr && recordProc->state() != QProcess::NotRunning;
    }
} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::RosbagPanel, rviz_common::Panel);
