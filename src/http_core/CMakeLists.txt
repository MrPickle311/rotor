cmake_minimum_required(VERSION 3.0.2)
project(http_core)

find_package(catkin REQUIRED COMPONENTS
        rospy
        roslint
        )

catkin_package(
        CATKIN_DEPENDS
)

catkin_python_setup()

roslint_python()

catkin_install_python(PROGRAMS
        scripts/http_client.py
        scripts/http_core.py
        DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
        )