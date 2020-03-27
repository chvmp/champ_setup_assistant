cmake_minimum_required(VERSION 2.8.3)
project(champ_setup_assistant)

find_package(catkin 
    REQUIRED
    COMPONENTS
)

catkin_package(
    CATKIN_DEPENDS
)

install(DIRECTORY launch urdf config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
