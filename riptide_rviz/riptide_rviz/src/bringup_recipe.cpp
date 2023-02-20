#include <sstream>
#include <set>
#include <memory>
#include <unordered_map>

#include "tinyxml2.h"

#include "riptide_rviz/bringup_recipe.hpp"

namespace riptide_rviz
{

    bool RecipeTopicData::operator==(const RecipeTopicData& lhs) {
        return name == lhs.name 
            && type_name == lhs.type_name 
            && qos_type == lhs.qos_type; 
    }

    bool RecipeTopicData::operator!=(const RecipeTopicData& lhs) {
        return !(operator==(lhs));
    }

    bool RecipeLaunch::operator==(const RecipeLaunch& lhs) {
        if (name != lhs.name) {
            return false;
        }

        if (stageID != lhs.stageID) {
            return false;
        }

        if (package != lhs.package) {
            return false;
        }

        if (topicList.size() != lhs.topicList.size()) {
            return false;
        }

        for (size_t i = 0; i < topicList.size(); ++i) {
            if (topicList[i] != lhs.topicList[i]) {
                return false;
            }
        }

        if (arguments.size() != lhs.arguments.size()) {
            return false;
        }

        for (auto i : arguments) {
            if (lhs.arguments.find(i.first) == lhs.arguments.end()) {
                return false;
            }

            if (i.second != lhs.arguments.at(i.first)) {
                return false;
            }
        }

        return true;
    }

    bool RecipeLaunch::operator!=(const RecipeLaunch& lhs) {
        return !(operator==(lhs));
    }

    bool RecipeStage::operator==(const RecipeStage& lhs) {
        if (id != lhs.id) {
            return false;
        }

        if (dependencies.size() != lhs.dependencies.size()) {
            return false;
        }

        for (size_t i = 0; i < dependencies.size(); ++i) {
            if (dependencies[i] != lhs.dependencies[i]) {
                return false;
            }
        }

        if (launchIndicies.size() != lhs.launchIndicies.size()) {
            return false;
        }

        for (size_t i = 0; i < launchIndicies.size(); ++i) {
            if (launchIndicies[i] != lhs.launchIndicies[i]) {
                return false;
            }
        }

        return true;
    }
    
    bool RecipeStage::operator!=(const RecipeStage& lhs) {
        return !(operator==(lhs));
    }

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
        case RecipeXMLErrorCode::MISSING_VALUE_ATTRIBUTE:
            oss << "This tag is missing the 'value' attribute. Please see the example in the recipies directory.";
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
        case RecipeXMLErrorCode::NON_EXISTANT_DEPENDENCY:
            oss << "This stage has a dependency that doesn't exist.";
            break;
        case RecipeXMLErrorCode::DEPENDENCY_CYCLE:
            oss << "This stage is contained within a dependency cycle (i.e. the stage, either directly or indirectly, depends on itself).";
            break;
        default:
            oss << "Somehow an unknown error occurred. Please yell at Hunter and tell him to fix his flippin' parser.";
            break;
        }

        return oss.str();
    }

    bool Recipe::operator==(const Recipe& lhs) {
        if (stages.size() != lhs.stages.size()) {
            return false;
        }

        for (auto i : stages) {
            if (lhs.stages.find(i.first) == lhs.stages.end()) {
                return false;
            }

            if (i.second != lhs.stages.at(i.first)) {
                return false;
            }
        }

        for (size_t i = 0; i < launches.size(); ++i) {
            if (*(launches[i]) != *(lhs.launches[i])) {
                return false;
            }
        }

        return true;
    }

    bool Recipe::operator!=(const Recipe& lhs) {
        return !(operator==(lhs));
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

        // Maps that contain the XML line numbers for different elements.
        std::unordered_map<std::string, int> depLineNums;
        std::unordered_map<std::string, int> stageLineNums;

        for (const XMLElement *stageXML = root->FirstChildElement(); stageXML != nullptr; stageXML = stageXML->NextSiblingElement()) {
            RecipeStage stage;

            // Parse this stage tag, and the launches within it    
            RecipeXMLError err = parseStageTag(stageXML, stage);

            if (err.errorCode != RecipeXMLErrorCode::SUCCESS) {
                return err;
            }

            stages[stage.id] = stage;
            for (auto dep : stage.dependencies) {
                depLineNums[dep] = stageXML->GetLineNum();
            }

            stageLineNums[stage.id] = stageXML->GetLineNum();
        }

        for (auto pair : depLineNums) {
            if (!stageExists(pair.first.c_str())) {
                return RecipeXMLError {
                    RecipeXMLErrorCode::NON_EXISTANT_DEPENDENCY,
                    pair.second // Line number for the stage
                };
            }
        }

        // Walk through the dependencies of each stage and check for dependency cycles
        std::set<std::string> dependencyWalkResults;
        for (auto pair : stages) {
            walkDependencyTree(pair.first, dependencyWalkResults);

            // If the dependency results contain the current stage id, fail
            if (dependencyWalkResults.find(pair.first) != dependencyWalkResults.end()) {
                return RecipeXMLError {
                    RecipeXMLErrorCode::DEPENDENCY_CYCLE,
                    stageLineNums[pair.first]
                }; 
            }

            dependencyWalkResults.clear();
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

        stage.id = stageID;
        
        // This is used to check if there are duplicate launches in the recipe
        std::set<std::string> existingLaunches;

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

                // Check this tag doesn't exist
                if (existingLaunches.find(launch.name) != existingLaunches.end()) {
                    // This launch already exists
                    return RecipeXMLError {
                        RecipeXMLErrorCode::DUPLICATE_LAUNCH_NAMES,
                        tag->GetLineNum()
                    };
                }

                existingLaunches.emplace(launch.name); 

                launches.push_back(std::make_shared<RecipeLaunch>(launch));
                stage.launchIndicies.push_back(launches.size() - 1);
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

        if (stage.launchIndicies.size() == 0) {
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

        stage.dependencies.emplace_back(id);

        return RecipeXMLError {
            RecipeXMLErrorCode::SUCCESS,
            -1
        }; 
    }

    bool Recipe::stageExists(const char * stageID) {
        for (auto pair : stages) {
            if (pair.first == stageID) {
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

        launch.name = name;
        launch.package = package;
        launch.stageID = stageID;
        
        for (const XMLElement *tag = launchXML->FirstChildElement(); tag != nullptr; tag = tag->NextSiblingElement()) {
            if (strcmp(tag->Name(), "topic") == 0) {
                // Check this stage has an id
                const char * topicName = tag->Attribute("name");
                if (topicName == nullptr) {
                    return RecipeXMLError {
                        RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE,
                        tag->GetLineNum()
                    };
                }

                // Check this id isn't used by other stages
                if (launch.topicExists(topicName)) {
                    return RecipeXMLError {
                        RecipeXMLErrorCode::DUPLICATE_TOPIC,
                        tag->GetLineNum()
                    };
                }

                // Get the type information
                const char * topicType = tag->Attribute("type");
                if (topicType == nullptr) {
                    // No type info provided. Return with error.
                    return RecipeXMLError {
                        RecipeXMLErrorCode::MISSING_TYPE_ATTRIBUTE,
                        tag->GetLineNum()
                    };
                }

                // Get the topic quality of service
                const char * topicQOS = tag->Attribute("qos");
                if (topicQOS == nullptr) {
                    // No QOS provided. Return with error
                    return RecipeXMLError {
                        RecipeXMLErrorCode::MISSING_QOS_ATTRIBUTE,
                        tag->GetLineNum()
                    };
                }

                RecipeTopicData topic;
                topic.name = topicName;
                topic.type_name = topicType;
                // Ensure topic qos is only 
                if (strcmp(topicQOS, "system_default") == 0 || strcmp(topicQOS, "sensor_data") == 0) {
                    topic.qos_type = topicQOS;
                } else {
                    // The qos is not a valid type. Retunr with error.
                    return RecipeXMLError {
                        RecipeXMLErrorCode::INVALID_QOS_TYPE,
                        tag->GetLineNum()
                    };
                }

                launch.topicList.emplace_back(topic);
            } else if (strcmp(tag->Name(), "arg") == 0) {
                // Check this stage has an id
                const char * argName = tag->Attribute("name");
                if (argName == nullptr) {
                    return RecipeXMLError {
                        RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE,
                        tag->GetLineNum()
                    };
                }

                const char * argValue = tag->Attribute("value");
                if (argValue == nullptr) {
                    return RecipeXMLError {
                        RecipeXMLErrorCode::MISSING_VALUE_ATTRIBUTE,
                        tag->GetLineNum()
                    };
                }

                launch.arguments[argName] = argValue;
            } else {
                return RecipeXMLError {
                    RecipeXMLErrorCode::UNKNOWN_TAG_TYPE,
                    tag->GetLineNum()
                };
            }
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

    void Recipe::walkDependencyTree(const std::string &stageID, std::set<std::string> &dependencyWalkResults) {
        RecipeStage stage = stages[stageID];

        if (stage.dependencies.size() == 0) {
            return;
        }

        for (auto i : stage.dependencies) {
            if (dependencyWalkResults.find(i) != dependencyWalkResults.end()) {
                return;
            }

            dependencyWalkResults.emplace(i);
            walkDependencyTree(i, dependencyWalkResults);
        }
    }

    std::vector<std::shared_ptr<RecipeLaunch>> Recipe::getAllLaunches() {
        return launches;
    }

    std::vector<std::vector<int>> Recipe::getLaunchOrder() {
        std::vector<std::vector<int>> launchOrder;

        std::set<std::string> handledStages;

        while (handledStages.size() < stages.size()) {
            // This is a group of launches that can all be launched at the same time
            // This is different than a stage, in that it can contain launches from
            // multiple stages
            //
            // Ex: if A recipe has a bunch of stages with no dependencies, all of the
            // launches in those stages would be a part of the same group, since they
            // are all launched at the same time.
            std::vector<int> launchGroup;
            std::vector<std::string> handledStagesThisIteration;

            for (auto pair : stages) {
                // if this stage was handled, skip it
                if (handledStages.find(pair.first) != handledStages.end()) {
                    continue;
                } 

                bool dependenciesHandled = true;
                for (auto dep : pair.second.dependencies) {
                    // if this dependency hasn't been handled, set a flag
                    if (handledStages.find(dep) == handledStages.end()) {
                        dependenciesHandled = false;
                    }
                }

                // If all of the dependencies have been inserted into the launchOrder
                // vector, then
                if (dependenciesHandled) {
                    handledStagesThisIteration.push_back(pair.first);
                    launchGroup.reserve(launchGroup.size() + pair.second.launchIndicies.size());
                    for (auto i : pair.second.launchIndicies) {
                        launchGroup.push_back(i);
                    }
                }
            }

            for (auto stage : handledStagesThisIteration) {
                handledStages.emplace(stage);
            }
            launchOrder.push_back(launchGroup);
        }
        
        return launchOrder;
    }

} // namespace riptide_rviz