# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/temp_install"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/tmp"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp${cfgdir}") # cfgdir has leading slash
endif()
