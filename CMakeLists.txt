# Copyright 2012 OSMOCOM Project
#
# This file is part of MiriSDR
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 3.12)
project(mirisdr C)

########################################################################
# Install directories: lib${LIB_SUFFIX} when LIB_SUFFIX is given, else the
# platform's own (lib/<multiarch> on Debian for /usr, lib64 on Fedora)
########################################################################
if(LIB_SUFFIX)
    set(CMAKE_INSTALL_LIBDIR "lib${LIB_SUFFIX}" CACHE PATH "Object code libraries (lib)" FORCE)
endif()
include(GNUInstallDirs)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

# the library's own directory, unless the linker searches it anyway
list(FIND CMAKE_C_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_FULL_LIBDIR}" _libdir_is_system)
if(_libdir_is_system EQUAL -1)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR}")
endif()

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

# Set the version information here
set(VERSION_INFO_MAJOR_VERSION 5) # increment major on api compatibility changes
set(VERSION_INFO_MINOR_VERSION 1) # increment minor on feature-level changes
set(VERSION_INFO_PATCH_VERSION git) # increment patch for bug fixes and docs
include(Version) # setup version info

########################################################################
# Compiler specific setup
########################################################################
if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
    ADD_DEFINITIONS(-Wall)
    ADD_DEFINITIONS(-Wextra)
    ADD_DEFINITIONS(-Wno-unused)
    ADD_DEFINITIONS(-Wsign-compare)
    #http://gcc.gnu.org/wiki/Visibility
    add_definitions(-fvisibility=hidden)
endif()

########################################################################
# Find build dependencies
########################################################################
find_package(PkgConfig)
find_package(LibUSB)
set(THREADS_USE_PTHREADS_WIN32 true)
find_package(Threads)

if(NOT LIBUSB_FOUND)
    message(FATAL_ERROR "LibUSB 1.0 required to compile MiriSDR")
endif()

########################################################################
# Setup the include and linker paths
########################################################################
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${LIBUSB_INCLUDE_DIR}
)

#link_directories(
#    ...
#)

# Set component parameters
#set(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "" FORCE)

########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)


########################################################################
# Handle detach kernel driver option
########################################################################
# on by default on Linux, where msi2500 can hold the receiver: the library gives it
# back when it closes the receiver
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(DETACH_KERNEL_DRIVER_DEFAULT ON)
else()
    set(DETACH_KERNEL_DRIVER_DEFAULT OFF)
endif()
option(DETACH_KERNEL_DRIVER "Detach kernel driver if loaded" ${DETACH_KERNEL_DRIVER_DEFAULT})
if (DETACH_KERNEL_DRIVER)
    message (STATUS "Building with kernel driver detaching enabled")
    add_definitions(-DDETACH_KERNEL_DRIVER=1)
else (DETACH_KERNEL_DRIVER)
    message (STATUS "Building with kernel driver detaching disabled, use -DDETACH_KERNEL_DRIVER=ON to enable")
endif (DETACH_KERNEL_DRIVER)

########################################################################
# Add subdirectories
########################################################################
########################################################################
# SoapySDR module, and miri_test's tests of it
########################################################################
option(ENABLE_SOAPY "Build the SoapySDR module when SoapySDR is found" ON)
set(MIRISDR_SOAPY OFF)
if(ENABLE_SOAPY)
    find_package(SoapySDR CONFIG QUIET)
    if(SoapySDR_FOUND)
        message(STATUS "Building the SoapySDR module")
        enable_language(CXX)
        set(MIRISDR_SOAPY ON)
    else()
        message(STATUS "SoapySDR not found, not building its module")
    endif()
endif()

# SoapySDR's module helper installs to lib${LIB_SUFFIX}, so keep it in step. Only
# after finding SoapySDR, which looks for its own library with LIB_SUFFIX
if(NOT LIB_SUFFIX AND CMAKE_INSTALL_LIBDIR MATCHES "^lib(.+)$")
    set(LIB_SUFFIX "${CMAKE_MATCH_1}")
endif()

add_subdirectory(include)
add_subdirectory(src)

if(MIRISDR_SOAPY)
    add_subdirectory(src/soapy)
endif()

########################################################################
# Create Pkg Config File
########################################################################
# libusb by its own pkg-config file when it has one, else by where it was found
IF(LIBUSB_PKG_FOUND)
    SET(MIRISDR_PC_REQUIRES "libusb-1.0")
ELSEIF(NOT CMAKE_CROSSCOMPILING)
    FOREACH(lib ${LIBUSB_LIBRARIES})
        GET_FILENAME_COMPONENT(libdir_usb "${lib}" DIRECTORY)
        LIST(APPEND MIRISDR_PC_LIBS "-L${libdir_usb}")
    ENDFOREACH(lib)
    LIST(APPEND MIRISDR_PC_LIBS "-lusb-1.0")
ELSE()
    LIST(APPEND MIRISDR_PC_LIBS "-lusb-1.0")
ENDIF(LIBUSB_PKG_FOUND)

# the transfers' lock: a critical section on Windows, pthreads elsewhere
IF(NOT WIN32)
    LIST(APPEND MIRISDR_PC_LIBS "${CMAKE_THREAD_LIBS_INIT}")
ENDIF(NOT WIN32)

# the baseband NCO's sine and cosine
IF(UNIX)
    LIST(APPEND MIRISDR_PC_LIBS "-lm")
ENDIF(UNIX)

# use space-separation format for the pc file
STRING(REPLACE ";" " " MIRISDR_PC_LIBS "${MIRISDR_PC_LIBS}")

set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix \${prefix})
if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
    set(libdir "${CMAKE_INSTALL_LIBDIR}")
else()
    set(libdir \${exec_prefix}/${CMAKE_INSTALL_LIBDIR})
endif()
set(includedir \${prefix}/include)

CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/libmirisdr.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libmirisdr.pc
@ONLY)

INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/libmirisdr.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)

########################################################################
# Print Summary
########################################################################
MESSAGE(STATUS "Building for version: ${VERSION} / ${LIBVER}")
MESSAGE(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "Installing libraries to: ${CMAKE_INSTALL_LIBDIR}")
