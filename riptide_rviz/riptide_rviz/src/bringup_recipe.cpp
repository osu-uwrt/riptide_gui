#include <sstream>
#include "tinyxml2.h"

#include "riptide_rviz/bringup_recipe.hpp"

namespace riptide_rviz
{

    std::string getRecipeXMLErrorMessage(RecipeXMLError err) {
        std::ostringstream oss;
        if (err.errorCode == RecipeXMLErrorCode::SUCCESS) {
            return "No error has occurred";
        }

        if (err.lineNumber == 0) {
            oss << "Recipe parsing error at unknown location: ";
        } else {
            oss << "Recipe parsing error at line " << err.lineNumber << " in the \".xml\": ";
        }

        switch(err.errorCode) {
        case RecipeXMLErrorCode::XML_NO_ATTRIBUTE:                      [[fallthrough]];
        case RecipeXMLErrorCode::XML_WRONG_ATTRIBUTE_TYPE:              [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_FILE_NOT_FOUND:              [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_FILE_COULD_NOT_BE_OPENED:    [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_FILE_READ_ERROR:             [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_PARSING_ELEMENT:             [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_PARSING_ATTRIBUTE:           [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_PARSING_TEXT:                [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_PARSING_CDATA:               [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_PARSING_COMMENT:             [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_PARSING_DECLARATION:         [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_PARSING_UNKNOWN:             [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_EMPTY_DOCUMENT:              [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_MISMATCHED_ELEMENT:          [[fallthrough]];
        case RecipeXMLErrorCode::XML_ERROR_PARSING:                     [[fallthrough]];
        case RecipeXMLErrorCode::XML_CAN_NOT_CONVERT_TEXT:              [[fallthrough]];
        case RecipeXMLErrorCode::XML_NO_TEXT_NODE:                      [[fallthrough]];
	    case RecipeXMLErrorCode::XML_ELEMENT_DEPTH_EXCEEDED:            [[fallthrough]];
	    case RecipeXMLErrorCode::XML_ERROR_COUNT:
            oss << "tinyxml2 could not parse the document. This recipe is malformed in some way";
            break;

        case RecipeXMLErrorCode::NO_LAUNCHES_TAG:
            oss << "The recipe parser requires there exist a <launches> tag. Please see the example in the recipies directory.";
            break;
        case RecipeXMLErrorCode::NON_STAGE_TAG:
            oss << "The parser only allows <stage> tags to be children of the <launches> tag.";
            break;
        case RecipeXMLErrorCode::UNKNOWN_TAG_TYPE:
            oss << "The parser could not identify this tag type, or the tag you are using is not in the correct place. Please see the example in the recipies directory.";
            break;
        case RecipeXMLErrorCode::MISSING_ID_ATTRIBUTE:
            oss << "This tag is missing the 'id' attribute. Please see the example in the recipies directory.";
            break;
        case RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE:
            oss << "This tag is missing the 'name' attribute. Please see the example in the recipies directory.";
            break;
        case RecipeXMLErrorCode::MISSING_PACKAGE_ATTRIBUTE:
            oss << "This tag is missing the 'package' attribute. Please see the example in the recipies directory.";
            break;
        case RecipeXMLErrorCode::MISSING_TYPE_ATTRIBUTE:
            oss << "This tag is missing the 'type' attribute. Please see the example in the recipies directory.";
            break;
        case RecipeXMLErrorCode::MISSING_QOS_ATTRIBUTE:
            oss << "This tag is missing the 'qos' attribute. Please see the example in the recipies directory.";
            break;
        case RecipeXMLErrorCode::INVALID_QOS_TYPE:
            oss << "This topic's QOS type is invalid. Note that the qos attribute can only be set to \"system_default\" or \"sensor_data\" (case-sensitive).";
            break;
        case RecipeXMLErrorCode::EMPTY_RECIPE:
            oss << "This recipe is empty.";
            break;
        case RecipeXMLErrorCode::DUPLICATE_STAGE_IDS:
            oss << "This stage's id is a duplicate of a previous stage. All stages must have unique id's";
            break;
        case RecipeXMLErrorCode::DUPLICATE_LAUNCH_NAMES:
            oss << "This launch's name is a duplicate of a previous launch. All launches must have unique id's";
            break;
        case RecipeXMLErrorCode::DUPLICATE_TOPIC:
            oss << "This topic's name is a duplicate of a previous topic. All topics within a launch must have unique names";
            break;
        case RecipeXMLErrorCode::STAGE_WITH_NO_LAUNCH:
            oss << "This stage contains no launches. All stages must have launches.";
            break;
        default:
            oss << "Somehow an unknown error occurred. Please yell at Hunter and tell him to fix his flippin' parser.";
            break;
        }

        return oss.str();
    }

    /*
     * Initializes the current Recipe object from an XML file.
     */
    RecipeXMLError Recipe::loadXml(std::string const& recipePath) {
        using namespace tinyxml2;

        XMLDocument doc;
        XMLError err;
        
        // Open Recipe document
        err = doc.LoadFile(recipePath.c_str());
        if (err != XML_SUCCESS) {
            // Document did not load. Return with error
            return RecipeXMLError {
                static_cast<RecipeXMLErrorCode>(err),
                doc.ErrorLineNum()
            };
        }

        const XMLElement *root = doc.RootElement();

        // Check if the root tag is a "launches" tag
        if (strcmp(root->Name(),"launches") != 0) {
            // Root tag is not a "launches" tag. Return with error
            return RecipeXMLError {
                RecipeXMLErrorCode::NO_LAUNCHES_TAG,
                root->GetLineNum()
            };
        }
        
        // Check that there are launches
        if (root->FirstChildElement() == nullptr) {
            // No stages found. Return with error
            return RecipeXMLError {
                RecipeXMLErrorCode::EMPTY_RECIPE,
                root->GetLineNum()
            };
        }

        for (const XMLElement *stageXML = root->FirstChildElement(); stageXML != nullptr; stageXML = stageXML->NextSiblingElement()) {
            RecipeStage stage;

            // Parse this stage tag, and the launches within it    
            RecipeXMLError err = parseStageTag(stageXML, stage);

            if (err.errorCode != RecipeXMLErrorCode::SUCCESS) {
                return err;
            }

            stages.emplace_back(stage);
        }

        return RecipeXMLError {
            RecipeXMLErrorCode::SUCCESS,
            -1
        }; 
    }

    RecipeXMLError Recipe::parseStageTag(const tinyxml2::XMLElement *stageXML, RecipeStage &stage) {
        using namespace tinyxml2;

        // Check this tag is a stage tag
        if (strcmp(stageXML->Name(), "stage") != 0) {
            return RecipeXMLError {
                RecipeXMLErrorCode::NON_STAGE_TAG,
                stageXML->GetLineNum()
            };
        }

        // Check this stage an id
        const char * stageID = stageXML->Attribute("id");
        if (stageID == nullptr) {
            return RecipeXMLError {
                RecipeXMLErrorCode::MISSING_ID_ATTRIBUTE,
                stageXML->GetLineNum()
            };
        }

        // Check this id isn't used by other stages
        if (stageExists(stageID)) {
            return RecipeXMLError {
                RecipeXMLErrorCode::DUPLICATE_STAGE_IDS,
                stageXML->GetLineNum()
            };
        }

        // Iterate through each tag within the stage tag. Note these should only
        // be either a "dependency" tag or a "launch" tag
        for (const XMLElement *tag = stageXML->FirstChildElement(); tag != nullptr; tag = tag->NextSiblingElement()) {
            RecipeXMLError err;

            // Get tag name
            const char * tagName = tag->Name();
            if (strcmp(tagName, "dependency") == 0) {
                // Parse dependency tag
                err = parseDependencyTag(tag, stage);
            } else if (strcmp(tagName, "launch") == 0) {
                // parse Launch Tag
                RecipeLaunch launch;
                err = parseLaunchTag(tag, stageID, launch);
                stage.launches.emplace_back(launch);
            } else {
                // Tag is neither a dependency or a launch.
                err = RecipeXMLError {
                    RecipeXMLErrorCode::UNKNOWN_TAG_TYPE,
                    tag->GetLineNum()
                };
            }

            if (err.errorCode != RecipeXMLErrorCode::SUCCESS) {
                // Return with whatever error "err" was set to
                return err;
            }
        }

        if (stage.launches.size() == 0) {
            // There are no launches found in this stage. Return with error
            return RecipeXMLError {
                RecipeXMLErrorCode::STAGE_WITH_NO_LAUNCH,
                stageXML->GetLineNum()
            };
        }

        return RecipeXMLError {
            RecipeXMLErrorCode::SUCCESS,
            -1
        }; 
    }

    RecipeXMLError Recipe::parseDependencyTag(const tinyxml2::XMLElement *dependsXML, RecipeStage &stage) {
        // Get dependency id
        const char * id = dependsXML->Attribute("id");
        if (id == nullptr) {
            // Dependency tag doesn't have an id. Return with error
            return RecipeXMLError {
                RecipeXMLErrorCode::MISSING_ID_ATTRIBUTE,
                dependsXML->GetLineNum()
            };
        }

        stage.outstandingDependencyIds.emplace_back(id);

        return RecipeXMLError {
            RecipeXMLErrorCode::SUCCESS,
            -1
        }; 
    }

    bool Recipe::stageExists(const char * stageID) {
        for (auto i : stages) {
            if (i.id == stageID) {
                return true;
            }
        }

        return false;
    }

    RecipeXMLError Recipe::parseLaunchTag(const tinyxml2::XMLElement *launchXML, const char * stageID, RecipeLaunch &launch) {
        using namespace tinyxml2;

        // Get launch name
        const char *name = launchXML->Attribute("name");
        if (name == nullptr) {
            // launch does not have a name attribute. Return with error
            return RecipeXMLError {
                RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE,
                launchXML->GetLineNum()
            };
        }

        // Get launch package
        const char *package = launchXML->Attribute("package");
        if (package == nullptr) {
            // launch does not have a package attribute. Return with error
            return RecipeXMLError {
                RecipeXMLErrorCode::MISSING_PACKAGE_ATTRIBUTE,
                launchXML->GetLineNum()
            };
        }

        // Check if this is a duplicate launch name
        if (launchExists(name)) {
            // There is already a launch with this name. Return with error.
            return RecipeXMLError {
                RecipeXMLErrorCode::DUPLICATE_LAUNCH_NAMES,
                launchXML->GetLineNum()
            };
        }

        launch.name = name;
        launch.package = package;
        launch.launchStatus = RecipeLaunchStatus::NOT_STARTED;
        launch.pid = -1;
        launch.stageID = stageID;
        
        for (const XMLElement *topicXML = launchXML->FirstChildElement(); topicXML != nullptr; topicXML = topicXML->NextSiblingElement()) {
            // Check this tag is a stage tag
            if (strcmp(topicXML->Name(), "topic") != 0) {
                return RecipeXMLError {
                    RecipeXMLErrorCode::UNKNOWN_TAG_TYPE,
                    topicXML->GetLineNum()
                };
            }

            // Check this stage an id
            const char * topicName = topicXML->Attribute("name");
            if (topicName == nullptr) {
                return RecipeXMLError {
                    RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE,
                    topicXML->GetLineNum()
                };
            }

            // Check this id isn't used by other stages
            if (launch.topicExists(topicName)) {
                return RecipeXMLError {
                    RecipeXMLErrorCode::DUPLICATE_TOPIC,
                    topicXML->GetLineNum()
                };
            }

            // Get the type information
            const char * topicType = topicXML->Attribute("type");
            if (topicName == nullptr) {
                // No type info provided. Return with error.
                return RecipeXMLError {
                    RecipeXMLErrorCode::MISSING_TYPE_ATTRIBUTE,
                    topicXML->GetLineNum()
                };
            }

            // Get the topic quality of service
            const char * topicQOS = topicXML->Attribute("qos");
            if (topicName == nullptr) {
                // No QOS provided. Return with error
                return RecipeXMLError {
                    RecipeXMLErrorCode::MISSING_QOS_ATTRIBUTE,
                    topicXML->GetLineNum()
                };
            }

            RecipeTopicData topic;
            topic.name = topicName;
            topic.type_name = topicType;
            // Ensure topic qos is only 
            if (strcmp(topicQOS, "system_default") == 0 || strcmp(topicQOS, "sensor_default") == 0) {
                topic.qos_type = topicQOS;
            } else {
                // The qos is not a valid type. Retunr with error.
                return RecipeXMLError {
                    RecipeXMLErrorCode::INVALID_QOS_TYPE,
                    topicXML->GetLineNum()
                };
            }

            launch.topicList.emplace_back(topic);
        }

        return RecipeXMLError {
            RecipeXMLErrorCode::SUCCESS,
            -1
        };
    }

    bool RecipeLaunch::topicExists(const char *topicName) {
        for (auto i : topicList) {
            if (i.name == topicName) {
                return true;
            }
        }

        return false;
    }

    bool Recipe::launchExists(const char * launchName) {
        for (auto stage : stages) {
            for (auto launch : stage.launches) {
                if (launch.name == launchName) {
                    return true;
                }
            }
        }

        return false;
    }

    void Recipe::setLaunchStatus(int64_t pid, RecipeLaunchStatus status) {
        
    }

    RecipeLaunch Recipe::getLaunchInformation(int64_t pid) {
        return RecipeLaunch();
    }
} // namespace riptide_rviz