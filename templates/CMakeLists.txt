cmake_minimum_required(VERSION 2.8.3)
project({{ robot_name }}_config)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    rospy
)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()