cmake_minimum_required(VERSION 2.8.3)
project(champ_setup_assistant)

find_package(catkin 
    REQUIRED
    COMPONENTS
)
catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
)

set(URL "https://github.com/chvmp/robot_state_plugin")
set(DOWNLOAD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(DEPENDENCY_PATH ${DOWNLOAD_PATH}/robot_state_plugin)

if (NOT EXISTS "${DEPENDENCY_PATH}")
  message(STATUS "NO robot_state_plugin found. Downloading now...")
  message(STATUS "${DOWNLOAD_PATH}")

  execute_process(
    COMMAND git clone ${URL} ${DEPENDENCY_PATH}
  )
  execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
    COMMAND catkin_make --pkg robot_state_plugin
  )
endif()
 
install(PROGRAMS
  scripts/setup_assistant.py
  scripts/package_creator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)