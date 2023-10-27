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
#include <OgreException.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace riptide_rviz
{
    std::string BringupWebview::getFromConfig(const rviz_common::Config &config, const QString& key, const QString& defaultValue)
    {
        QString str;
        config.mapGetString(key, &str);
        if(str == "")
        { 
            str = defaultValue;
            RVIZ_COMMON_LOG_WARNING("BringupWebview: Using " + defaultValue.toStdString() + " as the default value for " + key.toStdString()); 
        }

        return str.toStdString();
    }

    BringupWebview::BringupWebview(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);
    }

    void BringupWebview::onInitialize()
    {
        RVIZ_COMMON_LOG_INFO("BringupWebview: Initializing");
        vLayout = new QVBoxLayout(this);
    }

    void BringupWebview::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);

        this->host = QString::fromStdString(getFromConfig(config, "host", "127.0.0.1"));
        this->port = QString::fromStdString(getFromConfig(config, "port", "8080"));

        delayTimer = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->create_wall_timer
            (50ms, std::bind(&BringupWebview::delayedLoadUi, this));
    }

    void BringupWebview::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        config.mapSetValue("host", this->host);
        config.mapSetValue("port", this->port);
    }

    bool BringupWebview::event(QEvent *event)
    {
        return false;
    }

    BringupWebview::~BringupWebview()
    {
        // master window control removal
        delete vLayout;
    }

    void BringupWebview::delayedLoadUi()
    {
        try
        {
            webEngineView = new QWebEngineView(this);
            webEngineView->setZoomFactor(0.8);
            vLayout->addWidget(webEngineView);
            webEngineView->setUrl(QUrl("http://" + this->host + ":" + this->port));
            delayTimer->cancel();
        } catch(Ogre::InternalErrorException&)
        { }
    }

} // namespace riptide_rviz
#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::BringupWebview, rviz_common::Panel);
