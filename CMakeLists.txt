cmake_minimum_required(VERSION 2.8.3)
project(champ_setup_assistant)

find_package(catkin 
    REQUIRED
    COMPONENTS
)

set(URL "https://github.com/chvmp/robot_state_plugin")
set(DOWNLOAD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(DEPENDENCY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../robot_state_plugin)

if (NOT EXISTS "${DEPENDENCY_PATH}")
  message(STATUS "NO robot_state_plugin found. Downloading now...")
  message(STATUS "${DOWNLOAD_PATH}")

  execute_process(
    COMMAND git clone ${URL} ${DOWNLOAD_PATH}/robot_state_plugin
  )
  execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../
    COMMAND catkin_make --pkg robot_state_plugin
  )
endif()
