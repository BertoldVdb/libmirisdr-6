# Find libusb-1.0
#
# Sets:
#   LIBUSB_FOUND        - whether libusb-1.0 was found
#   LIBUSB_INCLUDE_DIR  - the directory holding libusb.h
#   LIBUSB_LIBRARIES    - the library to link
#   LIBUSB_PKG_FOUND    - whether pkg-config described it (used for the .pc Requires)
#
# Hints (any one is enough to point at a libusb that is not on the default path,
# e.g. a cross-compiled or downloaded build):
#   LIBUSB_ROOT         - a libusb prefix, or an unpacked release tree, as a CMake
#                         variable or an environment variable
# or set LIBUSB_INCLUDE_DIR and LIBUSB_LIBRARIES directly; both are then checked to exist.

# pkg-config gives the paths on systems that have it; skip it where it is absent
# (e.g. a bare Windows toolchain) so its absence is not a fatal error. Skip it too when
# cross-compiling: the host's pkg-config describes the host libusb, not the target's,
# and would point the build at the wrong library. Cross builds pass LIBUSB_ROOT (or
# LIBUSB_INCLUDE_DIR/LIBUSB_LIBRARIES) instead.
if(PKG_CONFIG_FOUND AND NOT CMAKE_CROSSCOMPILING)
  pkg_check_modules(LIBUSB_PKG QUIET libusb-1.0)
endif()

# an explicit LIBUSB_ROOT is authoritative, so it comes before the pkg-config hints
find_path(LIBUSB_INCLUDE_DIR
  NAMES libusb.h
  HINTS
    ${LIBUSB_ROOT}
    ENV LIBUSB_ROOT
    ${LIBUSB_PKG_INCLUDE_DIRS}
  PATHS
    /usr/include
    /usr/local/include
  PATH_SUFFIXES
    libusb-1.0
    include
    include/libusb-1.0
)

# Where an unpacked Windows release keeps the library for this compiler. Each compiler
# only searches its own folders, so an MSVC build never picks the MinGW libusb-1.0.a.
# MSVC takes the import library of a VS20xx build (they all link the same, the newest
# is tried first), and the oldest releases had MS64/dll at the top. MinGW takes the
# MinGW64 or MinGW32 ones.
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(_libusb_bits 64)
else()
  set(_libusb_bits 32)
endif()
set(_libusb_suffixes lib)
if(MSVC)
  foreach(_vs VS2022 VS2019 VS2017 VS2015 VS2013)
    list(APPEND _libusb_suffixes ${_vs}/MS${_libusb_bits}/dll)
  endforeach()
  list(APPEND _libusb_suffixes MS${_libusb_bits}/dll)
elseif(MINGW)
  list(APPEND _libusb_suffixes MinGW${_libusb_bits}/static MinGW${_libusb_bits}/dll)
endif()

# usb-1.0:        the Unix name (libusb-1.0.so / .a)
# libusb-1.0:     the import/static name shipped by the Windows releases
#                 (libusb-1.0.dll.a for MinGW, libusb-1.0.lib for MSVC)
find_library(LIBUSB_LIBRARIES
  NAMES usb-1.0 libusb-1.0 libusb
  HINTS
    ${LIBUSB_ROOT}
    ENV LIBUSB_ROOT
    ${LIBUSB_PKG_LIBRARY_DIRS}
  PATHS
    /usr/lib
    /usr/local/lib
  PATH_SUFFIXES
    ${_libusb_suffixes}
)

# A path given by hand (-DLIBUSB_INCLUDE_DIR / -DLIBUSB_LIBRARIES) is taken as-is by
# find_path/find_library without checking it exists, so a typo would only surface at
# link time. Catch it here instead.
if(LIBUSB_INCLUDE_DIR AND NOT EXISTS "${LIBUSB_INCLUDE_DIR}")
  message(FATAL_ERROR "LIBUSB_INCLUDE_DIR is set to a path that does not exist: ${LIBUSB_INCLUDE_DIR}")
endif()
if(LIBUSB_LIBRARIES AND NOT EXISTS "${LIBUSB_LIBRARIES}")
  message(FATAL_ERROR "LIBUSB_LIBRARIES is set to a path that does not exist: ${LIBUSB_LIBRARIES}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LibUSB
  REQUIRED_VARS LIBUSB_LIBRARIES LIBUSB_INCLUDE_DIR
  VERSION_VAR LIBUSB_PKG_VERSION
)

# the project checks LIBUSB_FOUND (upper case); FPHSA sets LibUSB_FOUND (package case)
set(LIBUSB_FOUND ${LibUSB_FOUND})

mark_as_advanced(LIBUSB_INCLUDE_DIR LIBUSB_LIBRARIES)
