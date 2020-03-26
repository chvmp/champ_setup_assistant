cmake_minimum_required(VERSION 2.8.3)
project(champ_setup_assistant)

find_package(catkin REQUIRED
    COMPONENTS
)

##set the URL to download
set(URL "https://github.com/chvmp/robot_state_plugin")
set(DOWNLOAD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(DEPENDENCY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../robot_state_plugin)

if (NOT EXISTS "${DEPENDENCY_PATH}")
  message(STATUS "NO visualization")
  message(STATUS "${DOWNLOAD_PATH}")

  execute_process(
    COMMAND git clone ${URL} ${DOWNLOAD_PATH}/robot_state_plugin
  )
endif()

catkin_package(
    CATKIN_DEPENDS
)

install(DIRECTORY launch urdf config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
