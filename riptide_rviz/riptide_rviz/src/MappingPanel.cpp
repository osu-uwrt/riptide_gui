#include "riptide_rviz/MappingPanel.hpp"

namespace riptide_rviz
{
    MappingPanel::MappingPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        ui = new Ui_MappingPanel();
        ui->setupUi(this);

        ui->calibStatus->setText("");
    }

    MappingPanel::~MappingPanel()
    {
        delete ui;
    }
}


#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::MappingPanel, rviz_common::Panel);
