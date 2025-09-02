# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/carbs/Desktop/PX4-Autopilot/src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client")
  file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client")
endif()
file(MAKE_DIRECTORY
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/tmp"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src/uclient-stamp"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src/uclient-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src/uclient-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src/uclient-stamp${cfgdir}") # cfgdir has leading slash
endif()
