#pragma once
#include <vector>

#include "tinyxml2.h"

namespace riptide_rviz
{

    struct RecipeTopicData {
        std::string name;
        std::string type_name;
        std::string qos_type;

        bool operator==(const RecipeTopicData&);
        bool operator!=(const RecipeTopicData&);
    };

    enum class RecipeLaunchStatus {
        NOT_STARTED,
        STARTING,
        RUNNING,
        ABORTED,
        STOPPED
    };

    /*
     *  Structure representing the <launch> tag in a recipe. This should share
     *  many similarities with the BringupStart action
     */
    class RecipeLaunch {
    public:
        std::string name = "";
        std::string package = "";
        std::string stageID = "";
        std::vector<RecipeTopicData> topicList;
        
        int64_t pid;
        RecipeLaunchStatus launchStatus;

        bool topicExists(const char *topicName);
        bool operator==(const RecipeLaunch&);
        bool operator!=(const RecipeLaunch&);
    };

    /*
     *   Structure representing the <stage> tag in a recipe.
     */
    class RecipeStage {
    public:
        std::string id = "";
        std::vector<std::string> outstandingDependencyIds;
        // TODO: Maybe make this into a map for easier accesses?
        std::vector<RecipeLaunch> launches;

        bool operator==(const RecipeStage&);
        bool operator!=(const RecipeStage&);
    };

    enum class RecipeXMLErrorCode {
        SUCCESS                             = tinyxml2::XML_SUCCESS,
        XML_NO_ATTRIBUTE                    = tinyxml2::XML_NO_ATTRIBUTE,
        XML_WRONG_ATTRIBUTE_TYPE            = tinyxml2::XML_WRONG_ATTRIBUTE_TYPE,
        XML_ERROR_FILE_NOT_FOUND            = tinyxml2::XML_ERROR_FILE_NOT_FOUND,
        XML_ERROR_FILE_COULD_NOT_BE_OPENED  = tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED,
        XML_ERROR_FILE_READ_ERROR           = tinyxml2::XML_ERROR_FILE_READ_ERROR,
        XML_ERROR_PARSING_ELEMENT           = tinyxml2::XML_ERROR_PARSING_ELEMENT,
        XML_ERROR_PARSING_ATTRIBUTE         = tinyxml2::XML_ERROR_PARSING_ATTRIBUTE,
        XML_ERROR_PARSING_TEXT              = tinyxml2::XML_ERROR_PARSING_TEXT,
        XML_ERROR_PARSING_CDATA             = tinyxml2::XML_ERROR_PARSING_CDATA,
        XML_ERROR_PARSING_COMMENT           = tinyxml2::XML_ERROR_PARSING_COMMENT,
        XML_ERROR_PARSING_DECLARATION       = tinyxml2::XML_ERROR_PARSING_DECLARATION,
        XML_ERROR_PARSING_UNKNOWN           = tinyxml2::XML_ERROR_PARSING_UNKNOWN,
        XML_ERROR_EMPTY_DOCUMENT            = tinyxml2::XML_ERROR_EMPTY_DOCUMENT,
        XML_ERROR_MISMATCHED_ELEMENT        = tinyxml2::XML_ERROR_MISMATCHED_ELEMENT,
        XML_ERROR_PARSING                   = tinyxml2::XML_ERROR_PARSING,
        XML_CAN_NOT_CONVERT_TEXT            = tinyxml2::XML_CAN_NOT_CONVERT_TEXT,
        XML_NO_TEXT_NODE                    = tinyxml2::XML_NO_TEXT_NODE,
	    XML_ELEMENT_DEPTH_EXCEEDED          = tinyxml2::XML_ELEMENT_DEPTH_EXCEEDED,
	    XML_ERROR_COUNT                     = tinyxml2::XML_ERROR_COUNT,

        NO_LAUNCHES_TAG,
        NON_STAGE_TAG,
        UNKNOWN_TAG_TYPE,
        MISSING_ID_ATTRIBUTE,
        MISSING_NAME_ATTRIBUTE,
        MISSING_PACKAGE_ATTRIBUTE,
        MISSING_TYPE_ATTRIBUTE,
        MISSING_QOS_ATTRIBUTE,
        INVALID_QOS_TYPE,
        EMPTY_RECIPE,
        DUPLICATE_STAGE_IDS,
        DUPLICATE_LAUNCH_NAMES,
        DUPLICATE_TOPIC,
        STAGE_WITH_NO_LAUNCH,
    };

    struct RecipeXMLError {
        RecipeXMLErrorCode errorCode;
        int lineNumber;
    };

    /*
     * This function turns a RecipeXMLError into a printable message. The 
     * printable message will include the line number and some information
     * about the error stored within RecipeXMLError.errorCode.
     */
    std::string getRecipeXMLErrorMessage(RecipeXMLError);

    class Recipe {
    public:
        /*
         * Initializes the Recipe object from an XML file.
         * 
         * "recipePath" must be a full path to the XML document.
         * 
         * On success: The current Recipe object will be initialized, with the
         * stages and launches vectors filled with relavent information. The
         * method will return a RecipeXMLError, with the errorCode set to
         * XML_SUCCESS (an enum constant provided by tinyxml2) with the
         * lineNumber set to -1.
         * 
         * On failure: The current Recipe object will be in an unknown state,
         * The method will return a RecipeXMLError, with the errorCode set to
         * some value in the XMLError enum provided by tinyxml2. The lineNumber
         * will also be set to what line in the XML file caused the error.
         * 
         * NOTE: tinyxml2 may set the lineNumber to 0 if the XML error occurred
         * at an unknown location
         */
        RecipeXMLError loadXml(std::string const& recipePath);
        void setLaunchStatus(int64_t pid, RecipeLaunchStatus status);

        // Maybe change the parameter to the launch name?
        RecipeLaunch getLaunchInformation(int64_t pid);

        bool operator==(const Recipe&);
        bool operator!=(const Recipe&);

        std::vector<RecipeStage> stages;
    private:
        // TODO: It may be cleaner if these parse functions were in there 
        // respective class definitions (i.e. "parseStageTag" was in the
        // RecipeStage class
        RecipeXMLError parseStageTag(const tinyxml2::XMLElement *stageXML, RecipeStage &stage);
        RecipeXMLError parseDependencyTag(const tinyxml2::XMLElement *dependsXML, RecipeStage &stage);
        RecipeXMLError parseLaunchTag(const tinyxml2::XMLElement *launchXML, const char * stageID, RecipeLaunch &launch);
        bool stageExists(const char * stageID);
        bool launchExists(const char * launchName);
    };

} // namespace riptide_rviz