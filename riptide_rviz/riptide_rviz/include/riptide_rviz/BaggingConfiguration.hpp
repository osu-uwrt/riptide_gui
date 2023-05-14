#pragma once

#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <QTreeView>
#include "tinyxml2.h"

#include "riptide_rviz/bringup_recipe.hpp"

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

namespace riptide_rviz {

#define __unused __attribute__((unused))

class BaggingTopicModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    BaggingTopicModel(const std::vector<RecipeTopicData> & topics, QObject *parent = nullptr);
    ~BaggingTopicModel();

    QVariant data(const QModelIndex &index, int role) const override;
    QVariant headerData(__unused int section, __unused Qt::Orientation orientation,
                        __unused int role = Qt::DisplayRole) const {return QVariant();}
    bool setData(const QModelIndex &index,
                              const QVariant &value, int role);

    int rowCount(__unused const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(__unused const QModelIndex &parent = QModelIndex()) const override {return 3;}

    // bool insertRows(int position, int rows, const QModelIndex &index = QModelIndex());
    // bool removeRows(int position, int rows, const QModelIndex &index = QModelIndex());

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    const std::vector<RecipeTopicData> getTopicConfig();

private:
    std::vector<RecipeTopicData> topics;
};

}