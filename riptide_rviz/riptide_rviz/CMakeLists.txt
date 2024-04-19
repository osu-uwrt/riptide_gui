cmake_minimum_required(VERSION 3.8)
project(riptide_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find general dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tinyxml2_vendor REQUIRED)

# find message deps
find_package(nav_msgs REQUIRED)
find_package(launch_msgs REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(chameleon_tf_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(riptide_meshes REQUIRED)

find_package(zed_interfaces QUIET) #not required because we will handle it if we cant find this. Its only installed on certain computers

# rviz specific deps
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# pick up headerfiles
include_directories(include)

# pick up soirce files to build
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

# pick up headers to moc
file(GLOB MOC_HEADERS ${PROJECT_SOURCE_DIR}/include/riptide_rviz/*.hpp)

file(GLOB UIS ${PROJECT_SOURCE_DIR}/src/*.ui)

# moc the headers
foreach(header "${MOC_HEADERS}")
  qt5_wrap_cpp(MOC_FILES "${header}") # this should spit out the moc files for each header
endforeach()

QT5_WRAP_UI( UI_HEADERS ${UIS} )

# add all of the source files here
add_library(${PROJECT_NAME} SHARED
            ${SRC_FILES}
            ${MOC_FILES}
            ${UI_HEADERS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_NS="talos")

# TODO: Maybe make bringup_recipe it's own library?
add_executable(recipe_parser_test tests/recipe_parser_tests.cpp src/bringup_recipe.cpp)

target_include_directories(recipe_parser_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/riptide_rviz>
  $<INSTALL_INTERFACE:include>)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Link ament packages
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  rclcpp_action
  ament_index_cpp
  tf2
  rviz_common 
  rviz_rendering 
  riptide_msgs2
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  launch_msgs
  diagnostic_msgs
  chameleon_tf_msgs
  vectornav_msgs
  robot_localization
  interactive_markers
  riptide_meshes
)

if(zed_interfaces_FOUND)
  ament_target_dependencies(${PROJECT_NAME}
    zed_interfaces)
  
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    USE_ZED_INTERFACES)
endif()

# Link non ament packages
target_link_libraries(${PROJECT_NAME} 
  Qt5::Widgets
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  tinyxml2
)

target_link_libraries(recipe_parser_test
  tinyxml2
)

ament_target_dependencies(recipe_parser_test
  ament_index_cpp
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  rviz_ogre_vendor
  resource_retriever
  ament_index_cpp
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

file(GLOB python_programs python/*.py)
install(PROGRAMS 
  ${python_programs}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch recipes
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY tests/recipes
  DESTINATION share/${PROJECT_NAME}/tests)

install(TARGETS recipe_parser_test
  DESTINATION lib/${PROJECT_NAME}
)
  
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
