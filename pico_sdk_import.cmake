# This is a copy of pico-sdk/external/pico_sdk_import.cmake (v1.5+).
#
# It lets a project find and import the Pico SDK by setting PICO_SDK_PATH
# (env var or cache var) before calling include(pico_sdk_import.cmake) from
# the top-level CMakeLists.txt. The file is intentionally short and stable;
# copy it verbatim from your local pico-sdk checkout if you need a newer
# version.
#
# Usage:
#   export PICO_SDK_PATH=/path/to/pico-sdk
#   mkdir build && cd build
#   cmake ..
#   make
#
# OR define PICO_SDK_PATH as a -D flag on the cmake command line.

if (NOT DEFINED PICO_SDK_PATH)
    if (DEFINED ENV{PICO_SDK_PATH})
        set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
        message("Using PICO_SDK_PATH from environment ('${PICO_SDK_PATH}')")
    else ()
        message(FATAL_ERROR
          "PICO_SDK_PATH not defined. Set it to the path of your pico-sdk checkout,
           e.g. export PICO_SDK_PATH=/path/to/pico-sdk")
    endif ()
endif ()

if (NOT EXISTS ${PICO_SDK_PATH}/pico_sdk_init.cmake)
    message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' does not appear to be a pico-sdk
            checkout (no pico_sdk_init.cmake found).")
endif ()

set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake CACHE INTERNAL "Path to pico_sdk_init.cmake")
include(${PICO_SDK_INIT_CMAKE_FILE})
