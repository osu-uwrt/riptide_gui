#include <iostream>
#include <string>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "bringup_recipe.hpp"

#define RVIZ_PACKAGE "riptide_rviz"


/*
 * This may have been a bad idea, but this tester uses a homebrew testing
 * "framework" which prints out tailored information for each test so it is
 * easier to know what went wrong.
 * 
 * The compareWithDiagnostics function does the actual testing + tailored info
 * and all of the test functions call it.
 * 
 * The test function call loadXml on an xml file that shares the same name with
 * the function (E.g. test_good_example() opens test_good_example.xml).
 * 
 * TODO: Some of these functions might be worth factoring out to
 * bringup_recipe.hpp. For example the errEnumToStr is actually useful to see
 * what error is happening, as opposed to just an integer.
 */

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
            std::cout << tabs(tabSize + 1) << "* name: " << launch->name << "\n";
            std::cout << tabs(tabSize + 1) << "* package: " << launch->package << "\n";
            std::cout << tabs(tabSize + 1) << "* topics:\n";

            for (auto topic : launch->topicList) {
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

    case RecipeXMLErrorCode::XML_NO_ATTRIBUTE: return "XML_NO_ATTRIBUTE";
    case RecipeXMLErrorCode::XML_WRONG_ATTRIBUTE_TYPE: return "XML_WRONG_ATTRIBUTE_TYPE";
    case RecipeXMLErrorCode::XML_ERROR_FILE_NOT_FOUND: return "XML_ERROR_FILE_NOT_FOUND";
    case RecipeXMLErrorCode::XML_ERROR_FILE_COULD_NOT_BE_OPENED: return "XML_ERROR_FILE_COULD_NOT_BE_OPENED";
    case RecipeXMLErrorCode::XML_ERROR_FILE_READ_ERROR: return "XML_ERROR_FILE_READ_ERROR";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_ELEMENT: return "XML_ERROR_PARSING_ELEMENT";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_ATTRIBUTE: return "XML_ERROR_PARSING_ATTRIBUTE";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_TEXT: return "XML_ERROR_PARSING_TEXT";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_CDATA: return "XML_ERROR_PARSING_CDATA";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_COMMENT: return "XML_ERROR_PARSING_COMMENT";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_DECLARATION: return "XML_ERROR_PARSING_DECLARATION";
    case RecipeXMLErrorCode::XML_ERROR_PARSING_UNKNOWN: return "XML_ERROR_PARSING_UNKNOWN";
    case RecipeXMLErrorCode::XML_ERROR_EMPTY_DOCUMENT: return "XML_ERROR_EMPTY_DOCUMENT";
    case RecipeXMLErrorCode::XML_ERROR_MISMATCHED_ELEMENT: return "XML_ERROR_MISMATCHED_ELEMENT";
    case RecipeXMLErrorCode::XML_ERROR_PARSING: return "XML_ERROR_PARSING";
    case RecipeXMLErrorCode::XML_CAN_NOT_CONVERT_TEXT: return "XML_CAN_NOT_CONVERT_TEXT";
    case RecipeXMLErrorCode::XML_NO_TEXT_NODE: return "XML_NO_TEXT_NODE";
    case RecipeXMLErrorCode::XML_ELEMENT_DEPTH_EXCEEDED: return "XML_ELEMENT_DEPTH_EXCEEDED";
    case RecipeXMLErrorCode::XML_ERROR_COUNT: return "XML_ERROR_COUNT";

    case RecipeXMLErrorCode::NO_LAUNCHES_TAG: return "NO_LAUNCHES_TAG";
    case RecipeXMLErrorCode::NON_STAGE_TAG: return "NON_STAGE_TAG";
    case RecipeXMLErrorCode::UNKNOWN_TAG_TYPE: return "UNKNOWN_TAG_TYPE";
    case RecipeXMLErrorCode::MISSING_ID_ATTRIBUTE: return "MISSING_ID_ATTRIBUTE";
    case RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE: return "MISSING_NAME_ATTRIBUTE";
    case RecipeXMLErrorCode::MISSING_PACKAGE_ATTRIBUTE: return "MISSING_PACKAGE_ATTRIBUTE";
    case RecipeXMLErrorCode::MISSING_TYPE_ATTRIBUTE: return "MISSING_TYPE_ATTRIBUTE";
    case RecipeXMLErrorCode::MISSING_QOS_ATTRIBUTE: return "MISSING_QOS_ATTRIBUTE";
    case RecipeXMLErrorCode::INVALID_QOS_TYPE: return "INVALID_QOS_TYPE";
    case RecipeXMLErrorCode::EMPTY_RECIPE: return "EMPTY_RECIPE";
    case RecipeXMLErrorCode::DUPLICATE_STAGE_IDS: return "DUPLICATE_STAGE_IDS";
    case RecipeXMLErrorCode::DUPLICATE_LAUNCH_NAMES: return "DUPLICATE_LAUNCH_NAMES";
    case RecipeXMLErrorCode::DUPLICATE_TOPIC: return "DUPLICATE_TOPIC";
    case RecipeXMLErrorCode::STAGE_WITH_NO_LAUNCH: return "STAGE_WITH_NO_LAUNCH";
    case RecipeXMLErrorCode::NON_EXISTANT_DEPENDENCY: return "NON_EXISTANT_DEPENDENCY";
    case RecipeXMLErrorCode::DEPENDENCY_CYCLE: return "DEPENDENCY_CYCLE";
    default: return "unknown recipe xml error";
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

    std::cout << "PASSED\n";

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

void test_bad_launches(const std::string &path) {

    std::string testName = "test_bad_launches.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::NO_LAUNCHES_TAG,
        3
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_empty_recipe(const std::string &path) {

    std::string testName = "test_bad_empty_recipe.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::EMPTY_RECIPE,
        3
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_non_stage(const std::string &path) {

    std::string testName = "test_bad_non_stage.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::NON_STAGE_TAG,
        4
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_stage_missing_id(const std::string &path) {

    std::string testName = "test_bad_stage_missing_id.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::MISSING_ID_ATTRIBUTE,
        4
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_stage_duplicate(const std::string &path) {

    std::string testName = "test_bad_stage_duplicate.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::DUPLICATE_STAGE_IDS,
        10
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_stage_child_tag(const std::string &path) {

    std::string testName = "test_bad_stage_child_tag.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::UNKNOWN_TAG_TYPE,
        5
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_stage_no_launch(const std::string &path) {

    std::string testName = "test_bad_stage_no_launch.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::STAGE_WITH_NO_LAUNCH,
        4
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_dep_missing_id(const std::string &path) {

    std::string testName = "test_bad_dep_missing_id.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::MISSING_ID_ATTRIBUTE,
        5
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_launch_name(const std::string &path) {

    std::string testName = "test_bad_launch_name.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE,
        5
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_launch_package(const std::string &path) {

    std::string testName = "test_bad_launch_package.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::MISSING_PACKAGE_ATTRIBUTE,
        5
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_launch_duplicate(const std::string &path) {

    std::string testName = "test_bad_launch_duplicate.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::DUPLICATE_LAUNCH_NAMES,
        9
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_launch_unknown_tag(const std::string &path) {

    std::string testName = "test_bad_launch_unknown_tag.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::UNKNOWN_TAG_TYPE,
        6
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_topic_no_name(const std::string &path) {

    std::string testName = "test_bad_topic_no_name.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::MISSING_NAME_ATTRIBUTE,
        6
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_topic_no_type(const std::string &path) {

    std::string testName = "test_bad_topic_no_type.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::MISSING_TYPE_ATTRIBUTE,
        6
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_topic_no_qos(const std::string &path) {

    std::string testName = "test_bad_topic_no_qos.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::MISSING_QOS_ATTRIBUTE,
        6
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_topic_bad_qos(const std::string &path) {

    std::string testName = "test_bad_topic_bad_qos.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::INVALID_QOS_TYPE,
        6
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_good_minimal(const std::string &path) {

    std::string testName = "test_good_minimal.xml";
    
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

    RecipeStage expectedStage;
    expectedStage.id = "1";
    expectedStage.launches.push_back(std::make_shared<RecipeLaunch>(expectedLaunch));

    expected.stages.push_back(expectedStage);

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

void test_bad_stage_bad_dep(const std::string &path) {

    std::string testName = "test_bad_stage_bad_dep.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::NON_EXISTANT_DEPENDENCY,
        4
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_self_dep(const std::string &path) {

    std::string testName = "test_bad_self_dep.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::DEPENDENCY_CYCLE,
        4
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_dep_cycle_1(const std::string &path) {

    std::string testName = "test_bad_dep_cycle_1.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::DEPENDENCY_CYCLE,
        4
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_bad_dep_cycle_2(const std::string &path) {

    std::string testName = "test_bad_dep_cycle_2.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    RecipeXMLError expectedErr = RecipeXMLError {
        RecipeXMLErrorCode::DEPENDENCY_CYCLE,
        11
    };

    TestResult expectedResult; 
    expectedResult.err = expectedErr;

    TestResult actualResult = TestResult {
        actualErr,
        actual
    };

    compareWithDiagnostics(testName, expectedResult, actualResult);
}

void test_good_example(const std::string &path) {

    std::string testName = "test_good_example.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    Recipe expected;
    RecipeStage expectedStage;
    RecipeLaunch expectedLaunch;

    // Create information for stage 1
    // something.launch.py's topic data
    RecipeTopicData expectedTopic = RecipeTopicData {
        "a",                // name
        "t1",               // topic_name
        "system_default"    // qos_type
    };
    expectedLaunch.topicList.emplace_back(expectedTopic);

    expectedTopic = RecipeTopicData {
        "b",                // name
        "t2",               // topic_name
        "sensor_data"       // qos_type
    };
    expectedLaunch.topicList.emplace_back(expectedTopic);
    
    expectedTopic = RecipeTopicData {
        "c",                // name
        "t3",               // topic_name
        "system_default"    // qos_type
    };
    expectedLaunch.topicList.emplace_back(expectedTopic);

    expectedTopic = RecipeTopicData {
        "d",                // name
        "t4",               // topic_name
        "system_default"    // qos_type
    };
    expectedLaunch.topicList.emplace_back(expectedTopic);

    expectedTopic = RecipeTopicData {
        "e",                // name
        "t5",               // topic_name
        "system_default"    // qos_type
    };
    expectedLaunch.topicList.emplace_back(expectedTopic);

    expectedLaunch.name = "something.launch.py";
    expectedLaunch.package = "abcdef";
    expectedLaunch.stageID = "1";

    expectedStage.id = "1";
    expectedStage.launches.push_back(std::make_shared<RecipeLaunch>(expectedLaunch));

    expected.stages.push_back(expectedStage);

    // Stage 2
    expectedTopic = RecipeTopicData {
        "/joint_states",                // name
        "sensor_msgs/msg/JointState",   // topic_name
        "sensor_data"                   // qos_type
    };
    expectedLaunch.topicList.clear();
    expectedLaunch.topicList.emplace_back(expectedTopic);

    expectedLaunch.name = "dummy_robot_bringup.launch.py";
    expectedLaunch.package = "dummy_robot_bringup";
    expectedLaunch.stageID = "2";

    expectedStage.id = "2";
    expectedStage.outstandingDependencyIds.emplace_back("1");
    expectedStage.launches.clear();
    expectedStage.launches.push_back(std::make_shared<RecipeLaunch>(expectedLaunch));

    expected.stages.push_back(expectedStage);

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

void test_good_deps(const std::string &path) {

    std::string testName = "test_good_deps.xml";
    
    Recipe actual;

    RecipeXMLError actualErr = actual.loadXml(path + testName);

    Recipe expected;
    RecipeStage expectedStage;
    RecipeLaunch expectedLaunch;

    // Create information for stage "start"
    // something.launch.py's topic data
    RecipeTopicData expectedTopic = RecipeTopicData {
        "a",                // name
        "t1",               // topic_name
        "system_default"    // qos_type
    };
    expectedLaunch.topicList.emplace_back(expectedTopic);

    expectedLaunch.name = "something.launch.py";
    expectedLaunch.package = "abcdef";
    expectedLaunch.stageID = "start";

    expectedStage.id = "start";
    expectedStage.launches.push_back(std::make_shared<RecipeLaunch>(expectedLaunch));

    expected.stages.push_back(expectedStage);

    // Stage 1
    expectedTopic = RecipeTopicData {
        "/joint_states",                // name
        "sensor_msgs/msg/JointState",   // topic_name
        "sensor_data"                   // qos_type
    };
    expectedLaunch.topicList.clear();
    expectedLaunch.topicList.emplace_back(expectedTopic);

    expectedLaunch.name = "dummy_robot_bringup.launch.py";
    expectedLaunch.package = "dummy_robot_bringup";
    expectedLaunch.stageID = "1";

    expectedStage.id = "1";
    expectedStage.outstandingDependencyIds.emplace_back("start");
    expectedStage.launches.clear();
    expectedStage.launches.push_back(std::make_shared<RecipeLaunch>(expectedLaunch));

    expected.stages.push_back(expectedStage);

    // Stage 2
    expectedTopic = RecipeTopicData {
        "name",         // name
        "type",         // topic_name
        "sensor_data"   // qos_type
    };
    expectedLaunch.topicList.clear();
    expectedLaunch.topicList.emplace_back(expectedTopic);

    expectedLaunch.name = "blah";
    expectedLaunch.package = "blah";
    expectedLaunch.stageID = "2";

    expectedStage.id = "2";
    expectedStage.launches.clear();
    expectedStage.launches.push_back(std::make_shared<RecipeLaunch>(expectedLaunch));

    expected.stages.push_back(expectedStage);

    // Stage end
    expectedTopic = RecipeTopicData {
        "end_name",     // name
        "end_type",     // topic_name
        "sensor_data"   // qos_type
    };
    expectedLaunch.topicList.clear();
    expectedLaunch.topicList.emplace_back(expectedTopic);

    expectedLaunch.name = "end.launch.py";
    expectedLaunch.package = "package";
    expectedLaunch.stageID = "end";

    expectedStage.id = "end";
    expectedStage.outstandingDependencyIds.clear();
    expectedStage.outstandingDependencyIds.emplace_back("1");
    expectedStage.outstandingDependencyIds.emplace_back("2");
    expectedStage.launches.clear();
    expectedStage.launches.push_back(std::make_shared<RecipeLaunch>(expectedLaunch));

    expected.stages.push_back(expectedStage);

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

    // Run malformed input tests
    test_bad_xml(testsRoot);
    test_bad_launches(testsRoot);
    test_bad_empty_recipe(testsRoot);
    test_bad_non_stage(testsRoot);
    test_bad_stage_missing_id(testsRoot);
    test_bad_stage_duplicate(testsRoot);
    test_bad_stage_child_tag(testsRoot);
    test_bad_stage_no_launch(testsRoot);
    test_bad_dep_missing_id(testsRoot);
    test_bad_launch_name(testsRoot);
    test_bad_launch_package(testsRoot);
    test_bad_launch_duplicate(testsRoot);
    test_bad_launch_unknown_tag(testsRoot);
    test_bad_topic_no_name(testsRoot);
    test_bad_topic_no_type(testsRoot);
    test_bad_topic_no_qos(testsRoot);
    test_bad_topic_bad_qos(testsRoot);
    test_bad_stage_bad_dep(testsRoot);
    test_bad_self_dep(testsRoot);
    test_bad_dep_cycle_1(testsRoot);
    test_bad_dep_cycle_2(testsRoot);

    // Run well formed input tests
    test_good_minimal(testsRoot);
    test_good_example(testsRoot);
    test_good_deps(testsRoot);

}