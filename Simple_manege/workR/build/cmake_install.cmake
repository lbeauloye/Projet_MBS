# Install script for directory: /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/Debug")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/cmake_aux/flags/cmake_install.cmake")
  include("/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/cmake_aux/listing/cmake_install.cmake")
  include("/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/cmake_aux/libraries/cmake_install.cmake")
  include("/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/cmake_aux/make_opt/cmake_install.cmake")
  include("/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/mbs_common/cmake_install.cmake")
  include("/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/symbolicR/cmake_install.cmake")
  include("/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/userfctR/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
