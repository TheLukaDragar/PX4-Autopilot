# Install script for directory: /Users/carbs/Desktop/PX4-Autopilot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
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
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/opt/homebrew/bin/arm-none-eabi-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/platforms/nuttx/src/px4/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/drivers/adc/board_adc/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/drivers/cdcacm_autostart/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/drivers/dshot/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/drivers/gps/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/drivers/imu/invensense/icm42688p/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/drivers/pwm_out/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/drivers/rc_input/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/drivers/telemetry/frsky_telemetry/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/drivers/tone_alarm/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/attitude_estimator_q/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/battery_status/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/commander/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/control_allocator/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/dataman/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/flight_mode_manager/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/land_detector/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/load_mon/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/logger/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/manual_control/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/mavlink/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/mc_att_control/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/mc_pos_control/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/mc_rate_control/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/navigator/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/rc_update/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/modules/sensors/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/systemcmds/dmesg/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/systemcmds/hardfault_log/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/systemcmds/param/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/systemcmds/top/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/systemcmds/topic_listener/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/src/systemcmds/uorb/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/boards/matek/f405-hdte/src/cmake_install.cmake")
  include("/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/platforms/nuttx/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/carbs/Desktop/PX4-Autopilot/build/matek_f405-hdte_default/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
