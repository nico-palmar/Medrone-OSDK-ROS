# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/AdvancedSensing"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/tmp"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src"
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp${cfgdir}") # cfgdir has leading slash
endif()
