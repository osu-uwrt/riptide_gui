#pragma once
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <QTreeView>
#include "tinyxml2.h"

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

namespace riptide_rviz {

class BaggingSelectionItem {
    public:
        explicit BaggingSelectionItem(BaggingSelectionItem *parent);
        virtual ~BaggingSelectionItem();

        int childCount() const {return children.size();}
        QVariant data() const {return fieldData;}
        BaggingSelectionItem *parent() {return parentItem;};

        BaggingSelectionItem *child(unsigned int number);
        int childNumber() const;

        virtual void getTopicsToRecord(std::vector<std::string> &topicsOut);
        virtual Qt::CheckState getCheckState() const;
        virtual void setChecked(bool set);

    protected:
        void addChild(BaggingSelectionItem* child);
        void setName(const QVariant &data) {fieldData = data;}

    private:
        QVariant fieldData;
        std::vector<BaggingSelectionItem *> children;
        BaggingSelectionItem *parentItem;
};

class BaggingTopicEntry: public BaggingSelectionItem {
    public:
        BaggingTopicEntry(const char *topicName, bool enabledByDefault, BaggingSelectionItem* parent):
                BaggingSelectionItem(parent), topicName(topicName), enabled(enabledByDefault) {
            setName(topicName);
        }

        void getTopicsToRecord(std::vector<std::string> &topicsOut) {if (enabled) topicsOut.push_back(topicName);}
        Qt::CheckState getCheckState() const {return (enabled ? Qt::Checked : Qt::Unchecked);}
        void setChecked(bool set) {enabled = set;}

    private:
        std::string topicName;
        bool enabled;
};

class BaggingTopicGroup: public BaggingSelectionItem {
    public:
        class ParseError: std::runtime_error {
            public:
                ParseError(std::string message, int line): std::runtime_error(message + " [Line " + std::to_string(line) + "]") {}
                ParseError(tinyxml2::XMLError error, int line):
                    std::runtime_error(tinyxml2::XMLDocument::ErrorIDToName(error) + std::string(" [Line ") + std::to_string(line) + "]") {}
        };

        BaggingTopicGroup(const char *filename);

    private:
        BaggingTopicGroup(const tinyxml2::XMLElement &element, bool defaultEnabled, BaggingSelectionItem *parent);
        void parseTopicElement(const tinyxml2::XMLElement &element, bool defaultEnabled);
        void populateTopicGroup(const tinyxml2::XMLElement &element, bool defaultEnabled);
};


#define __unused __attribute__((unused))

class BaggingTopicModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    BaggingTopicModel(const char *filename, QObject *parent = nullptr);
    ~BaggingTopicModel();

    QVariant data(const QModelIndex &index, int role) const override;
    QVariant headerData(__unused int section, __unused Qt::Orientation orientation,
                        __unused int role = Qt::DisplayRole) const {return QVariant();}

    QModelIndex index(int row, int column,
                      const QModelIndex &parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex &index) const override;

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(__unused const QModelIndex &parent = QModelIndex()) const override {return 1;}

    Qt::ItemFlags flags(const QModelIndex &index) const override;

private:
    BaggingSelectionItem *getItem(const QModelIndex &index) const;
    BaggingSelectionItem *rootItem;
};

}