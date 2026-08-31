#pragma once
#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>

#include <QString>
#include <QStringList>
#include <QProcess>
#include <QTimer>
#include <QWidget>
#include <QListWidgetItem>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "ui_RosbagPanel.h"

namespace riptide_rviz
{
    // Simple panel for recording a rosbag from RVIZ. Wraps "ros2 bag record" in a child
    // process so the bag is finalized cleanly on stop (via SIGINT, like a Ctrl-C).
    class RosbagPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        RosbagPanel(QWidget *parent = 0);
        ~RosbagPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;

    private:
        void refreshTopics();
        void applyTopicFilter();
        void toggleRecording();
        void startRecording();
        void stopRecording();
        void updateElapsed();
        void setInputsEnabled(bool enabled);
        void setStatus(const QString &status, bool error = false);
        bool isRecording() const;

        // preset handling
        void loadPresets();
        void applyPreset(int comboIndex);
        void clearTopicSelection();
        QListWidgetItem *ensureTopicItem(const QString &name);
        std::string resolveTopic(const std::string &raw) const;

        Ui_RosbagPanel *ui;

        // robot namespace, used to resolve relative preset topics and for config save/load
        QString robotNs;

        // recording presets loaded from config/rosbag_presets.yaml: (name, resolved topics)
        std::vector<std::pair<QString, QStringList>> presets;

        // the running "ros2 bag record" process, or null when not recording
        QProcess *recordProc = nullptr;

        // ticks once a second to update the elapsed recording time
        QTimer *elapsedTimer = nullptr;
        int64_t recordStartMs = 0;
    };
} // namespace riptide_rviz
