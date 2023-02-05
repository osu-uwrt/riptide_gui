#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include "tinyxml2.h"

#include "riptide_rviz/Bringup.hpp"
#include "riptide_rviz/bringup_recipe.hpp"

namespace riptide_rviz
{

    /*
     * Initializes the current Recipe object from an XML file.
     */
    RecipeXMLError Recipe::loadXml(std::string const& recipeFile) {
        using namespace tinyxml2;
        
        std::string recipeFilePath = ament_index_cpp::get_package_share_directory(RVIZ_PKG) 
            + "/recipies/" + recipeFile;
        XMLDocument doc;
        XMLError err;
        
        // Open Recipe document
        err = doc.LoadFile(recipeFilePath.c_str());
        if (err != XML_SUCCESS) {
            // Document did not load. Return with error
            return (RecipeXMLError) {
                .errorCode = static_cast<RecipeXMLErrorCode>(err),
                .lineNumber = doc.ErrorLineNum()
            };
        }

        const XMLElement *root = doc.RootElement();

        // Check if the root tag is a "launches" tag
        if (strcmp(root->Name(),"launches") != 0) {
            // Root tag is not a "launches" tag. Return with error
            return (RecipeXMLError) {
                .errorCode = RecipeXMLErrorCode::NO_LAUNCHES_TAG,
                .lineNumber = root->GetLineNum()
            };
        }
        
        // Check that there are launches
        if (root->FirstChildElement() == nullptr) {
            // No stages found. Return with error
            return (RecipeXMLError) {
                .errorCode = RecipeXMLErrorCode::EMPTY_RECIPE,
                .lineNumber = root->GetLineNum()
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

        return (RecipeXMLError) {
            .errorCode = RecipeXMLErrorCode::SUCCESS,
            .lineNumber = -1
        }; 
    }

    RecipeXMLError Recipe::parseStageTag(const tinyxml2::XMLElement *stageXML, RecipeStage &stage) {
        using namespace tinyxml2;

        // Check this tag is a stage tag
        if (strcmp(stageXML->Name(), "stage") != 0) {
            return (RecipeXMLError) {
                .errorCode = RecipeXMLErrorCode::NON_STAGE_TAG,
                .lineNumber = stageXML->GetLineNum()
            };
        }

        // Check this stage an id
        const char * stageID = stageXML->Attribute("id");
        if (stageID == nullptr) {
            return (RecipeXMLError) {
                .errorCode = RecipeXMLErrorCode::MISSING_ID_ATTRIBUTE,
                .lineNumber = stageXML->GetLineNum()
            };
        }

        // Check this id isn't used by other stages
        if (stageExists(stageID)) {
            return (RecipeXMLError) {
                .errorCode = RecipeXMLErrorCode::DUPLICATE_STAGE_IDS,
                .lineNumber = stageXML->GetLineNum()
            };
        }

        for (const XMLElement *tag = stageXML->FirstChildElement(); tag != nullptr; tag = tag->NextSiblingElement()) {
            RecipeXMLError err;

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
                err = (RecipeXMLError){
                    .errorCode = RecipeXMLErrorCode::UNKNOWN_TAG_TYPE,
                    .lineNumber = tag->GetLineNum()
                };
            }

            if (err.errorCode != RecipeXMLErrorCode::SUCCESS) {
                return err;
            }
        }

        if (stage.launches.size() == 0) {
            return (RecipeXMLError){
                .errorCode = RecipeXMLErrorCode::STAGE_WITH_NO_LAUNCH,
                .lineNumber = stageXML->GetLineNum()
            };
        }

        return (RecipeXMLError){
            .errorCode = RecipeXMLErrorCode::SUCCESS,
            .lineNumber = -1
        }; 
    }

    RecipeXMLError Recipe::parseDependencyTag(const tinyxml2::XMLElement *dependsXML, RecipeStage &stage) {
        const char * id = dependsXML->Attribute("id");
        if (id == nullptr) {
            return (RecipeXMLError){
                .errorCode = RecipeXMLErrorCode::MISSING_ID_ATTRIBUTE,
                .lineNumber = dependsXML->GetLineNum()
            };
        }

        stage.outstandingDependencyIds.emplace_back(id);

        return (RecipeXMLError){
            .errorCode = RecipeXMLErrorCode::SUCCESS,
            .lineNumber = -1
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

        const char *name = launchXML->Attribute("name");
        if (name == nullptr) {
            return (RecipeXMLError){
                .errorCode = RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE,
                .lineNumber = launchXML->GetLineNum()
            };
        }

        const char *package = launchXML->Attribute("package");
        if (package == nullptr) {
            return (RecipeXMLError){
                .errorCode = RecipeXMLErrorCode::MISSING_PACKAGE_ATTRIBUTE,
                .lineNumber = launchXML->GetLineNum()
            };
        }

        if (launchExists(name)) {
            return (RecipeXMLError){
                .errorCode = RecipeXMLErrorCode::DUPLICATE_LAUNCH_NAMES,
                .lineNumber = launchXML->GetLineNum()
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
                return (RecipeXMLError) {
                    .errorCode = RecipeXMLErrorCode::UNKNOWN_TAG_TYPE,
                    .lineNumber = topicXML->GetLineNum()
                };
            }

            // Check this stage an id
            const char * topicName = topicXML->Attribute("name");
            if (topicName == nullptr) {
                return (RecipeXMLError) {
                    .errorCode = RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE,
                    .lineNumber = topicXML->GetLineNum()
                };
            }

            // Check this id isn't used by other stages
            if (launch.topicExists(topicName)) {
                return (RecipeXMLError) {
                    .errorCode = RecipeXMLErrorCode::DUPLICATE_TOPIC,
                    .lineNumber = topicXML->GetLineNum()
                };
            }

            const char * topicType = topicXML->Attribute("type");
            if (topicName == nullptr) {
                return (RecipeXMLError) {
                    .errorCode = RecipeXMLErrorCode::MISSING_TYPE_ATTRIBUTE,
                    .lineNumber = topicXML->GetLineNum()
                };
            }

            const char * topicQOS = topicXML->Attribute("qos");
            if (topicName == nullptr) {
                return (RecipeXMLError) {
                    .errorCode = RecipeXMLErrorCode::MISSING_QOS_ATTRIBUTE,
                    .lineNumber = topicXML->GetLineNum()
                };
            }

            launch_msgs::msg::TopicData topic;
            topic.name = topicName;
            topic.type_name = topicType;
            if (strcmp(topicQOS, "system_default")) {
                topic.qos_type = launch_msgs::msg::TopicData::QOS_SYSTEM_DEFAULT;
            } else if (strcmp(topicQOS, "sensor_data")){
                topic.qos_type = launch_msgs::msg::TopicData::QOS_SYSTEM_DEFAULT;
            } else {
                return (RecipeXMLError) {
                    .errorCode = RecipeXMLErrorCode::INVALID_QOS_TYPE,
                    .lineNumber = topicXML->GetLineNum()
                };
            }

            launch.topicList.emplace_back(topic);
        }
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