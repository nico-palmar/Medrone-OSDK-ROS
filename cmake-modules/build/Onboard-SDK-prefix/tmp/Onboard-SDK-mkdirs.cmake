# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/cmake-modules/../Onboard-SDK-3.8.1"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/cmake-modules/../Onboard-SDK-3.8.1/../Onboard-SDK-3.8.1-build"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/cmake-modules/build/Onboard-SDK-prefix"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/cmake-modules/build/Onboard-SDK-prefix/tmp"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/cmake-modules/build/Onboard-SDK-prefix/src/Onboard-SDK-stamp"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/cmake-modules/build/Onboard-SDK-prefix/src"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/cmake-modules/build/Onboard-SDK-prefix/src/Onboard-SDK-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/nico/catkin_ws/src/Onboard-SDK-ROS/cmake-modules/build/Onboard-SDK-prefix/src/Onboard-SDK-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/nico/catkin_ws/src/Onboard-SDK-ROS/cmake-modules/build/Onboard-SDK-prefix/src/Onboard-SDK-stamp${cfgdir}") # cfgdir has leading slash
endif()
