# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/external/Source/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/external/Build/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/external/Install/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/external/tmp/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/external/Stamp/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/external/Download/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/external/Stamp/FlameGraph"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/external/Stamp/FlameGraph/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_default/external/Stamp/FlameGraph${cfgdir}") # cfgdir has leading slash
endif()
