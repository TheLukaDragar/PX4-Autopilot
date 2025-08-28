# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/Source/FlameGraph")
  file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/Source/FlameGraph")
endif()
file(MAKE_DIRECTORY
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/Build/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/Install/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/tmp/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/Stamp/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/Download/FlameGraph"
  "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/Stamp/FlameGraph"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/Stamp/FlameGraph/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/external/Stamp/FlameGraph${cfgdir}") # cfgdir has leading slash
endif()
