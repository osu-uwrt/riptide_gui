#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

#include "riptide_rviz/SimulatorPanel.hpp"

namespace riptide_rviz {
    SimulatorPanel::SimulatorPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_SimulatorPanel();
        uiPanel->setupUi(this);
    }

    SimulatorPanel::~SimulatorPanel() {
        delete uiPanel;
    }

    void SimulatorPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }

    void SimulatorPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
    }

    void SimulatorPanel::onInitialize()
    {
        RVIZ_COMMON_LOG_INFO("SimulatorPanel: Initialzing");
    }

    bool SimulatorPanel::event(QEvent *event) {
        return false;
    }

}