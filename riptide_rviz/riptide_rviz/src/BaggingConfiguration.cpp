#include "riptide_rviz/BaggingConfiguration.hpp"

#include <QtWidgets>

#include <launch_msgs/msg/topic_data.hpp>

namespace riptide_rviz {

    // ========================================
    // Bagging Topic Model
    // ========================================

    BaggingTopicModel::BaggingTopicModel(const std::vector<RecipeTopicData> & topics, QObject *parent): 
        QAbstractTableModel(parent){
        
        this->topics = topics;
    }

    BaggingTopicModel::~BaggingTopicModel() {
        
    }

    QVariant BaggingTopicModel::data(const QModelIndex &index, int role) const {
        if (!index.isValid())
            return QVariant();

        if (role != Qt::DisplayRole && role != Qt::EditRole)
            return QVariant();

        auto rowItem = topics.at(index.row());

        // detemine the column index
        int colIndex = index.column();
        if(colIndex == 0) {
            return QString::fromStdString(rowItem.name);
        } else if (colIndex == 1){
            return QString::fromStdString(rowItem.type_name);
        } else if (colIndex == 2){ 
            return QString::fromStdString(rowItem.qos_type);
        } else {
            return QVariant();
        }
    }

    int BaggingTopicModel::rowCount(const QModelIndex &index) const {
        return topics.size();
    }

    bool BaggingTopicModel::setData(const QModelIndex &index,
                                const QVariant &value, int role)
    {
        if (index.isValid() && role == Qt::EditRole) {

            // determine our information at the location 
            auto oldRowItem = topics.at(index.row());
            int colIndex = index.column();

            if(colIndex == 0) {
                oldRowItem.name = value.toString().toStdString();
            } else if (colIndex == 1){
                oldRowItem.type_name = value.toString().toStdString();
            } else if (colIndex == 2){ 
                oldRowItem.qos_type = value.toString().toStdString();
            } else {
                return false;
            }

            // put the new row item back in the list
            topics[index.row()] = oldRowItem;
            
            emit dataChanged(index, index);
            return true;
        }
        return false;
    }

    Qt::ItemFlags BaggingTopicModel::flags(const QModelIndex &index) const {
        if (!index.isValid())
            return Qt::NoItemFlags;

        Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsEditable;

        return flags;
    }

    const std::vector<RecipeTopicData> BaggingTopicModel::getTopicConfig(){
        return topics;
    }

};
