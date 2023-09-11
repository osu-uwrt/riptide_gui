#include "riptide_rviz/BringupWebview.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>
#include <string>
#include <cstring>
#include <system_error>
#include <unistd.h>
#include <sys/wait.h>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QTimer>
#include "QtWebEngine"

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace riptide_rviz
{
    BringupWebview::BringupWebview(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

    }

    void BringupWebview::onInitialize()
    {
        RVIZ_COMMON_LOG_INFO("BringupWebview: Initializing");
        uiPanel = new Ui_BringupWebviewPanel();
        uiPanel->setupUi(this);
        QWebEngineView *webEngineView = uiPanel->webEngineView;
        webEngineView->setUrl(QUrl("http://" + this->host + ":" + this->port));
    }

    void BringupWebview::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }

    void BringupWebview::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
    }

    bool BringupWebview::event(QEvent *event)
    {
        return false;
    }

    BringupWebview::~BringupWebview()
    {
        // master window control removal

    }

} // namespace riptide_rviz
#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::BringupWebview, rviz_common::Panel);