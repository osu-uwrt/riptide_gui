#include "riptide_rviz/BaggingConfiguration.hpp"
#include <QtWidgets>

namespace riptide_rviz {

// ========================================
// Bagging Selection Item
// ========================================

BaggingSelectionItem::BaggingSelectionItem(BaggingSelectionItem *parent): parentItem(parent) {}

BaggingSelectionItem::~BaggingSelectionItem() {
    for (auto child : children) {
        delete child;
    }
}

BaggingSelectionItem *BaggingSelectionItem::child(unsigned int number) {
    if (number >= children.size())
        return nullptr;
    return children.at(number);
}

int BaggingSelectionItem::childNumber() const {
    if (parentItem) {
        auto it = std::find(parentItem->children.begin(), parentItem->children.end(), const_cast<BaggingSelectionItem*>(this));
        return (it == children.end() ? 0 : it - children.begin());
    }
    return 0;
}

void BaggingSelectionItem::getTopicsToRecord(std::vector<std::string> &topicsOut) {
    for (auto child : children) {
        child->getTopicsToRecord(topicsOut);
    }
}

Qt::CheckState BaggingSelectionItem::getCheckState() const {
    bool checkStateLoaded = false;
    Qt::CheckState state = Qt::Unchecked;

    for (auto child : children) {
        auto childState = child->getCheckState();
        if (!checkStateLoaded) {
            state = childState;
            checkStateLoaded = true;
        }
        else if (state != childState) {
            state = Qt::PartiallyChecked;
        }
    }

    return state;
}

void BaggingSelectionItem::setChecked(bool set) {
    for (auto child : children) {
        child->setChecked(set);
    }
}

void BaggingSelectionItem::addChild(BaggingSelectionItem* child) {
    children.push_back(child);
}

// ========================================
// Bagging Topic Group
// ========================================

BaggingTopicGroup::BaggingTopicGroup(const char *filename): BaggingSelectionItem(nullptr) {
    using namespace tinyxml2;

    XMLDocument doc;
    XMLError err;

    // Open Recipe document
    err = doc.LoadFile(filename);
    if (err != XML_SUCCESS) {
        // Document did not load. Return with error
        throw ParseError(err, doc.ErrorLineNum());
    }

    const XMLElement *root = doc.RootElement();
    populateTopicGroup(*root, false);
}

BaggingTopicGroup::BaggingTopicGroup(const tinyxml2::XMLElement &element, bool defaultEnabled, BaggingSelectionItem *parent):
        BaggingSelectionItem(parent) {
    populateTopicGroup(element, defaultEnabled);
}

void BaggingTopicGroup::parseTopicElement(const tinyxml2::XMLElement &element, bool defaultEnabled) {
    using namespace tinyxml2;

    // Check if the tag is a "topic" tag
    // This check should really only apply to the root element
    if (strcmp(element.Name(), "topic") != 0) {
        throw ParseError("Expected <topic> tag", element.GetLineNum());
    }

    // Grab the name attribute
    const char *name = element.Attribute("name");
    if (name == nullptr) {
        throw ParseError("Topic requires \"name\" attribute", element.GetLineNum());
    }

    // Update default enabled if attribute set
    const XMLAttribute *defaultEnabledAttr = element.FindAttribute("default_enabled");
    if (defaultEnabledAttr != nullptr) {
        XMLError err = defaultEnabledAttr->QueryBoolValue(&defaultEnabled);
        if (err != XML_SUCCESS) {
            throw ParseError(err, defaultEnabledAttr->GetLineNum());
        }
    }

    addChild(new BaggingTopicEntry(name, defaultEnabled, static_cast<BaggingSelectionItem*>(this)));
}

void BaggingTopicGroup::populateTopicGroup(const tinyxml2::XMLElement &element, bool defaultEnabled) {
    using namespace tinyxml2;

    // Check if the tag is a "group" tag
    // This check should really only apply to the root element
    if (strcmp(element.Name(), "group") != 0) {
        throw ParseError("Expected <group> tag", element.GetLineNum());
    }

        // Grab the name attribute
    const char *title = element.Attribute("title");
    if (title == nullptr) {
        throw ParseError("Group requires \"title\" attribute", element.GetLineNum());
    }
    setName(title);

    // Update default enabled if attribute set
    const XMLAttribute *defaultEnabledAttr = element.FindAttribute("default_enabled");
    if (defaultEnabledAttr != nullptr) {
        XMLError err = defaultEnabledAttr->QueryBoolValue(&defaultEnabled);
        if (err != XML_SUCCESS) {
            throw ParseError(err, defaultEnabledAttr->GetLineNum());
        }
    }

    // Check that there are children elements
    if (element.FirstChildElement() == nullptr) {
        throw ParseError("Group requires at least one child group/topic", element.GetLineNum());
    }

    // Parse all children elements
    for (const XMLElement *child = element.FirstChildElement(); child != nullptr; child = child->NextSiblingElement()) {
        if (strcmp(child->Name(), "group") == 0) {
            addChild(new BaggingTopicGroup(*child, defaultEnabled, static_cast<BaggingSelectionItem*>(this)));
        }
        else if (strcmp(child->Name(), "topic") == 0) {
            parseTopicElement(*child, defaultEnabled);
        }
        else {
            throw ParseError("Invalid Element: " + std::string(child->Name()), element.GetLineNum());
        }
    }
}

// ========================================
// Bagging Topic Model
// ========================================

BaggingTopicModel::BaggingTopicModel(const char *filename, QObject *parent): QAbstractItemModel(parent){
    rootItem = new BaggingTopicGroup(filename);
}

BaggingTopicModel::~BaggingTopicModel() {
    delete rootItem;
}

QVariant BaggingTopicModel::data(const QModelIndex &index, int role) const {
    if (!index.isValid())
        return QVariant();

    BaggingSelectionItem *item = getItem(index);

    if (role == Qt::CheckStateRole && index.column() == 0) {
        return item->getCheckState();
    }

    if (role != Qt::DisplayRole)
        return QVariant();

    return item->data();
}

QModelIndex BaggingTopicModel::index(int row, int column, const QModelIndex &parent) const {
    if (parent.isValid() && parent.column() != 0)
        return QModelIndex();

    BaggingSelectionItem *parentItem = getItem(parent);
    if (!parentItem)
        return QModelIndex();

    BaggingSelectionItem *childItem = parentItem->child(row);
    if (childItem)
        return createIndex(row, column, childItem);
    return QModelIndex();
}

QModelIndex BaggingTopicModel::parent(const QModelIndex &index) const {
    if (!index.isValid())
        return QModelIndex();

    BaggingSelectionItem *childItem = getItem(index);
    BaggingSelectionItem *parentItem = childItem ? childItem->parent() : nullptr;

    if (parentItem == rootItem || !parentItem)
        return QModelIndex();

    return createIndex(parentItem->childNumber(), 0, parentItem);
}

int BaggingTopicModel::rowCount(const QModelIndex &parent) const {
    if (parent.isValid() && parent.column() > 0)
        return 0;

    const BaggingSelectionItem *parentItem = getItem(parent);

    return parentItem ? parentItem->childCount() : 0;
}

Qt::ItemFlags BaggingTopicModel::flags(const QModelIndex &index) const {
    if (!index.isValid())
        return Qt::NoItemFlags;

    Qt::ItemFlags flags = Qt::ItemIsEnabled;

    if ( index.column() == 0 )
        flags |= Qt::ItemIsUserCheckable;

    return flags;
}

BaggingSelectionItem *BaggingTopicModel::getItem(const QModelIndex &index) const {
    if (index.isValid()) {
        BaggingSelectionItem *item = static_cast<BaggingSelectionItem*>(index.internalPointer());
        if (item)
            return item;
    }
    return rootItem;
}

};
