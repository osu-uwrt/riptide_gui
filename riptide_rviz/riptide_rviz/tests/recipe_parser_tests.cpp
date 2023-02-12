#include <iostream>
#include <string>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "bringup_recipe.hpp"

#define RVIZ_PACKAGE "riptide_rviz"

using namespace riptide_rviz;

struct TestResult {
    RecipeXMLError err;
    Recipe recipe;
};

std::string tabs(int num) {
    std::ostringstream oss;
    for (int i = 0; i < num; ++i) {
        oss << "\t";
    }
    return oss.str();
}

void printRecipe(const Recipe recipe, int tabSize) {
    for (auto stage : recipe.stages) {
        std::cout << tabs(tabSize) << "id: " << stage.id << "\n";
        std::cout << tabs(tabSize) << "dependencies:\n";

        for (auto dep : stage.outstandingDependencyIds) {
            std::cout << tabs(tabSize + 1) << "* " << dep << "\n";
        }

        std::cout << tabs(tabSize) << "launches:\n";
        for (auto launch : stage.launches) {
            std::cout << tabs(tabSize + 1) << "* name: " << launch.name << "\n";
            std::cout << tabs(tabSize + 1) << "* package: " << launch.package << "\n";
            std::cout << tabs(tabSize + 1) << "* pid: " << launch.pid << "\n";
            std::cout << tabs(tabSize + 1) << "* launchStatus: " << (int) launch.launchStatus << "\n";
            std::cout << tabs(tabSize + 1) << "* topics:\n";

            for (auto topic : launch.topicList) {
                std::cout << tabs(tabSize + 2) 
                          << topic.name << " | " 
                          << topic.type_name << " | "
                          << topic.qos_type << " \n";
            }

        }
    }
}

std::string errEnumToStr(RecipeXMLErrorCode code) {

    switch(code) {
    case RecipeXMLErrorCode::SUCCESS:
        return "SUCCESS";

    case RecipeXMLErrorCode::XML_NO_ATTRIBUTE:
        return "XML_NO_ATTRIBUTE";
    case RecipeXMLErrorCode::XML_WRONG_ATTRIBUTE_TYPE:
        return "XML_WRONG_ATTRIBUTE_TYPE";
    case RecipeXMLErrorCode::XML_ERROR_FILE_NOT_FOUND:
        return "XML_ERROR_FILE_NOT_FOUND";
    case RecipeXMLErrorCode::XML_ERROR_FILE_COULD_NOT_BE_OPENED:
        return "XML_ERROR_FILE_COULD_NOT_BE_OPENED";
    case RecipeXMLErrorCode::XML_ERROR_FILE_READ_ERROR:
        return "XML_ERROR_FILE_READ_ERROR";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_ELEMENT:
        return "XML_ERROR_PARSING_ELEMENT";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_ATTRIBUTE:
        return "XML_ERROR_PARSING_ATTRIBUTE";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_TEXT:
        return "XML_ERROR_PARSING_TEXT";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_CDATA:
        return "XML_ERROR_PARSING_CDATA";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_COMMENT:
        return "XML_ERROR_PARSING_COMMENT";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_DECLARATION:
        return "XML_ERROR_PARSING_DECLARATION";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_UNKNOWN:
        return "XML_ERROR_PARSING_UNKNOWN";
    case RecipeXMLErrorCode::XML_ERROR_EMPTY_DOCUMENT:
        return "XML_ERROR_EMPTY_DOCUMENT";
    case RecipeXMLErrorCode::XML_ERROR_MISMATCHED_ELEMENT:
        return "XML_ERROR_MISMATCHED_ELEMENT";
    case RecipeXMLErrorCode::XML_ERROR_PARSING:
        return "XML_ERROR_PARSING";
    case RecipeXMLErrorCode::XML_CAN_NOT_CONVERT_TEXT:
        return "XML_CAN_NOT_CONVERT_TEXT";
    case RecipeXMLErrorCode::XML_NO_TEXT_NODE:
        return "XML_NO_TEXT_NODE";
    case RecipeXMLErrorCode::XML_ELEMENT_DEPTH_EXCEEDED:
        return "XML_ELEMENT_DEPTH_EXCEEDED";
    case RecipeXMLErrorCode::XML_ERROR_COUNT:
        return "XML_ERROR_COUNT";

    case RecipeXMLErrorCode::NO_LAUNCHES_TAG:
        return "NO_LAUNCHES_TAG";
    case RecipeXMLErrorCode::NON_STAGE_TAG:
        return "NON_STAGE_TAG";
    case RecipeXMLErrorCode::UNKNOWN_TAG_TYPE:
        return "UNKNOWN_TAG_TYPE";
    case RecipeXMLErrorCode::MISSING_ID_ATTRIBUTE:
        return "MISSING_ID_ATTRIBUTE";
    case RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE:
        return "MISSING_NAME_ATTRIBUTE";
    case RecipeXMLErrorCode::MISSING_PACKAGE_ATTRIBUTE:
        return "MISSING_PACKAGE_ATTRIBUTE";
    case RecipeXMLErrorCode::MISSING_TYPE_ATTRIBUTE:
        return "MISSING_TYPE_ATTRIBUTE";
    case RecipeXMLErrorCode::MISSING_QOS_ATTRIBUTE:
        return "MISSING_QOS_ATTRIBUTE";
    case RecipeXMLErrorCode::INVALID_QOS_TYPE:
        return "INVALID_QOS_TYPE";
    case RecipeXMLErrorCode::EMPTY_RECIPE:
        return "EMPTY_RECIPE";
    case RecipeXMLErrorCode::DUPLICATE_STAGE_IDS:
        return "DUPLICATE_STAGE_IDS";
    case RecipeXMLErrorCode::DUPLICATE_LAUNCH_NAMES:
        return "DUPLICATE_LAUNCH_NAMES";
    case RecipeXMLErrorCode::DUPLICATE_TOPIC:
        return "DUPLICATE_TOPIC";
    case RecipeXMLErrorCode::STAGE_WITH_NO_LAUNCH:
        return "STAGE_WITH_NO_LAUNCH";
    default:
        return "unknown recipe xml error";
    }
}

/*
 * This compares two test results and gives diagnostic information on where
 * they are different.
 * 
 * Returns: true when the results are equal, false when they are not.
 * 
 * Side effect: This prints diagnostic information to the screen.
 */
bool compareWithDiagnostics(std::string testName, TestResult expected, TestResult actual) {
    std::cout << "* " << testName << " ";
    if (expected.err.errorCode != actual.err.errorCode) {
        std::cout << "FAILED\n\texpected errorCode: " 
                  << errEnumToStr(expected.err.errorCode)
                  << "\tactual errorCode: " 
                  << errEnumToStr(actual.err.errorCode) << "\n";
        return false;
    }

    if (expected.err.lineNumber != actual.err.lineNumber) {
        std::cout << "FAILED\n\texpected lineNumber: " 
                  << expected.err.lineNumber
                  << "\tactual lineNumber: "
                  << actual.err.lineNumber << "\n";
        return false;
    }


    if (expected.err.errorCode != RecipeXMLErrorCode::SUCCESS) {
        // This test is a malformed input, so the internal state of the Recipe
        // doesn't matter
        std::cout << "PASSED\n";
        return true;
    } else {
        // This test is uses a well-formed input ensure the two recipe's are equal

        auto expectedStages = expected.recipe;
        auto actualStages = actual.recipe;

        if (expected.recipe != actual.recipe) {
            std::cout << "FAILED\n\tExpected recipe: \n";
            printRecipe(expectedStages, 2);
            std::cout << "\tActual Recipe: \n"; 
            printRecipe(actualStages, 2);

            return false;
        }
    }

    std::cout << "PASSED";

    return true;
}

void test_bad_xml(const std::string &path) {

    std::string testName = "test_bad_xml.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::XML_ERROR_PARSING_TEXT,
        1
    };

    TestResult expectedResult; 
        expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_good_1(const std::string &path) {

    std::string testName = "test_good_1.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    Recipe expected;
    RecipeTopicData expectedTopic = RecipeTopicData {
        "a",                // name
        "t1",               // topic_name
        "system_default"    // qos_type
    };

    RecipeLaunch expectedLaunch;
    expectedLaunch.name = "something.launch.py";
    expectedLaunch.package = "abcdef";
    expectedLaunch.stageID = "1";
    expectedLaunch.topicList.emplace_back(expectedTopic);
    expectedLaunch.pid = -1;
    expectedLaunch.launchStatus = RecipeLaunchStatus::NOT_STARTED;

    RecipeStage expectedStage;
    expectedStage.id = "1";
    expectedStage.launches.emplace_back(expectedLaunch);

    expected.stages.emplace_back(expectedStage);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::SUCCESS,
        -1
    };

    TestResult expectedResult = TestResult {
        expectedErr,
        expected
    };

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

int main() {

    std::cout << "+--------------------+\n";
    std::cout << "|RECIPE PARSER TESTER|\n";
    std::cout << "+--------------------+\n\n";

    std::string testsRoot = ament_index_cpp::get_package_share_directory(RVIZ_PACKAGE) + "/tests/recipies/";
    std::cout << "testsRoot: " << testsRoot << "\n";

    test_bad_xml(testsRoot);

    test_good_1(testsRoot);

}