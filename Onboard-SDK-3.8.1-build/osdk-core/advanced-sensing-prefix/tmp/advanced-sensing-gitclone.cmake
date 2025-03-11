# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if(EXISTS "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp/advanced-sensing-gitclone-lastrun.txt" AND EXISTS "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp/advanced-sensing-gitinfo.txt" AND
  "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp/advanced-sensing-gitclone-lastrun.txt" IS_NEWER_THAN "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp/advanced-sensing-gitinfo.txt")
  message(STATUS
    "Avoiding repeated git clone, stamp file is up to date: "
    "'/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp/advanced-sensing-gitclone-lastrun.txt'"
  )
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git" 
            clone --no-checkout --config "advice.detachedHead=false" "https://github.com/dji-sdk/Onboard-SDK-Resources.git" "advanced-sensing"
    WORKING_DIRECTORY "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src"
    RESULT_VARIABLE error_code
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/dji-sdk/Onboard-SDK-Resources.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git" 
          checkout "advanced-sensing-2.0.3-x86_64" --
  WORKING_DIRECTORY "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'advanced-sensing-2.0.3-x86_64'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git" 
            submodule update --recursive --init 
    WORKING_DIRECTORY "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing"
    RESULT_VARIABLE error_code
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp/advanced-sensing-gitinfo.txt" "/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp/advanced-sensing-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/nico/catkin_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/advanced-sensing-prefix/src/advanced-sensing-stamp/advanced-sensing-gitclone-lastrun.txt'")
endif()
