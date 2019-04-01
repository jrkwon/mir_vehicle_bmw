cmake_minimum_required(VERSION 2.8.12)
project(mir_bmw_urdf)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
