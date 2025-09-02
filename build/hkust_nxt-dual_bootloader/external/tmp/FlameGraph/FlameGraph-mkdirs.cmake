# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/Source/FlameGraph")
  file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/Source/FlameGraph")
endif()
file(MAKE_DIRECTORY
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/Build/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/Install/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/tmp/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/Stamp/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/Download/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/Stamp/FlameGraph"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/Stamp/FlameGraph/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/hkust_nxt-dual_bootloader/external/Stamp/FlameGraph${cfgdir}") # cfgdir has leading slash
endif()
