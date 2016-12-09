cmake_minimum_required (VERSION 2.6)
project (LSM303DLHC)

set(CMAKE_CURRENT_SOURCE_DIR .)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_PREFIX /usr/)

set(LSM303_SOURCE
  lsm303Accelerometer.cpp
  lsm303Magnetometer.cpp
)

set(LSM303_HEADERS
  lsm303Accelerometer.h
  lsm303Magnetometer.h
)


add_library(lsm303 SHARED ${LSM303_SOURCE} ${LSM303_HEADERS})
target_link_libraries(lsm303 firmware_i2c)

set_target_properties(lsm303 PROPERTIES
  VERSION 1.1.0
  SOVERSION 1
)

install(TARGETS lsm303 DESTINATION lib)
install(FILES ${LSM303_HEADERS} DESTINATION include)
