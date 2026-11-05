# SoapySDR module, built when SoapySDR is found
enable_language(CXX)

# On ELF platforms the module takes the library in statically, its symbols kept inside. Linked
# to the shared library, its mirisdr_* calls would resolve to libmirisdr-4 when an application
# loaded that first (gqrx does, through gr-osmosdr), and mix the two libraries
if(NOT WIN32 AND NOT APPLE)
    set(_mirisdr_lib mirisdr_static -Wl,--exclude-libs,ALL)
else()
    set(_mirisdr_lib mirisdr_shared)
endif()

SOAPY_SDR_MODULE_UTIL(
    TARGET mirisdrSupport
    SOURCES Registration.cpp Settings.cpp Streaming.cpp
    LIBRARIES ${_mirisdr_lib}
)
target_include_directories(mirisdrSupport PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include)
if(NOT WIN32 AND NOT APPLE)
    target_compile_definitions(mirisdrSupport PRIVATE mirisdr_STATIC)
endif()
