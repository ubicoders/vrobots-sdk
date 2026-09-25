# The C++ example programs, built against the prebuilt VRobots SDK C bundle.
#
# The SDK itself is not built here. Download the C bundle for your OS from the
# GitHub Releases page of this repository:
#
#     vrobots_sdk-cpp-<version>-windows-x86_64.zip
#     vrobots_sdk-cpp-<version>-linux-x86_64.tar.gz
#
# The archive has no top-level folder: `include/` holds both headers
# (vrobots_sdk.h and vrobots_sdk.hpp) and `lib/` holds the vrobots_sdk_capi
# library. Unpack it into a folder of its own, then point CMake at it in one of
# these ways:
#
#   1. NEXT TO THE REPOSITORY (no cache variables needed). Unpack into a folder
#      named `vrobots_sdk-cpp` or `vrobots_sdk-cpp-<anything>` beside your
#      checkout of this repository:
#
#          <parent>/vrobots-sdk/examples/cpp/CMakeLists.txt   (this file)
#          <parent>/vrobots_sdk-cpp-<version>-<os>/include/
#          <parent>/vrobots_sdk-cpp-<version>-<os>/lib/
#
#      then, from the repository root:
#
#          cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
#          cmake --build target/cpp-build --config Release
#
#      If more than one matching folder sits there, none is guessed; use one of
#      the overrides below.
#
#   2. ANYWHERE ELSE. Pass the unpacked root:
#
#          cmake -S examples/cpp -B target/cpp-build -DVROBOTS_SDK_DIR=/path/to/bundle
#
#      or name the pieces individually:
#
#          -DVROBOTS_SDK_INCLUDE_DIR=/path/to/bundle/include
#          -DVROBOTS_SDK_LIBRARY=/path/to/bundle/lib/<library file>
#
#      (<library file> is libvrobots_sdk_capi.so on Linux and the import library
#      vrobots_sdk_capi.dll.lib on Windows.)
#
# The bundle also carries its own copy of these sources in `examples/`, one level
# under the unpacked root; that layout is detected as well, so
# `cmake -S examples -B build` inside an unpacked bundle works unchanged.

cmake_minimum_required(VERSION 3.16)
project(vrobots_examples_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# A single-config generator with no build type produces an unoptimised binary
# that is still called "release" by everyone who runs it. Pick one.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# Locate the SDK
# ---------------------------------------------------------------------------

# Every search is relative to this file, never to the current directory.
#   _vr_repo_root    : the repository root (this file is examples/cpp/CMakeLists.txt)
#   _vr_bundle_root  : one level up, for when these sources sit in the `examples/`
#                      folder of an unpacked bundle
get_filename_component(_vr_repo_root "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(_vr_parent_dir "${_vr_repo_root}/.." ABSOLUTE)
get_filename_component(_vr_bundle_root "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

set(VROBOTS_SDK_DIR "" CACHE PATH
    "Root of an unpacked VRobots SDK C bundle (the folder holding include/ and lib/)")
set(VROBOTS_SDK_INCLUDE_DIR "" CACHE PATH
    "Folder holding vrobots_sdk.h and vrobots_sdk.hpp (overrides the search)")

# An unpacked bundle beside the repository checkout: `vrobots_sdk-cpp` or
# `vrobots_sdk-cpp-<anything>`. Exactly one match is used; several are reported
# and skipped rather than guessed between.
set(_vr_sibling_roots "")
file(GLOB _vr_sibling_candidates LIST_DIRECTORIES true
    "${_vr_parent_dir}/vrobots_sdk-cpp"
    "${_vr_parent_dir}/vrobots_sdk-cpp-*")
foreach(_vr_candidate IN LISTS _vr_sibling_candidates)
    if(IS_DIRECTORY "${_vr_candidate}/include" AND IS_DIRECTORY "${_vr_candidate}/lib")
        list(APPEND _vr_sibling_roots "${_vr_candidate}")
    endif()
endforeach()
list(LENGTH _vr_sibling_roots _vr_sibling_count)
if(_vr_sibling_count GREATER 1)
    message(STATUS "Several unpacked bundles beside the repository, none chosen:")
    foreach(_vr_candidate IN LISTS _vr_sibling_roots)
        message(STATUS "    ${_vr_candidate}")
    endforeach()
    message(STATUS "  Pick one with -DVROBOTS_SDK_DIR=...")
    set(_vr_sibling_roots "")
endif()

set(_vr_include_hints "")
set(_vr_lib_hints "")
if(VROBOTS_SDK_INCLUDE_DIR)
    list(APPEND _vr_include_hints "${VROBOTS_SDK_INCLUDE_DIR}")
endif()
if(VROBOTS_SDK_DIR)
    list(APPEND _vr_include_hints "${VROBOTS_SDK_DIR}/include")
    list(APPEND _vr_lib_hints "${VROBOTS_SDK_DIR}/lib")
endif()
foreach(_vr_root IN LISTS _vr_sibling_roots)
    list(APPEND _vr_include_hints "${_vr_root}/include")
    list(APPEND _vr_lib_hints "${_vr_root}/lib")
endforeach()
list(APPEND _vr_include_hints "${_vr_bundle_root}/include")
list(APPEND _vr_lib_hints "${_vr_bundle_root}/lib")

# The bundle puts the C header and the C++ wrapper in one directory, but each is
# found on its own (and both directories end up on the include path) so that
# -DVROBOTS_SDK_C_INCLUDE_DIR=... and -DVROBOTS_SDK_CXX_INCLUDE_DIR=... can still
# point at two different places.
find_path(VROBOTS_SDK_C_INCLUDE_DIR
    NAMES vrobots_sdk.h
    HINTS ${_vr_include_hints}
    NO_DEFAULT_PATH
)
find_path(VROBOTS_SDK_CXX_INCLUDE_DIR
    NAMES vrobots_sdk.hpp
    HINTS ${_vr_include_hints}
    NO_DEFAULT_PATH
)

# On MSVC a crate that is both cdylib and staticlib emits three files: the DLL,
# its import library (`.dll.lib`) and the static library (`.lib`). Link the
# import library and copy the DLL next to the executables. Elsewhere the .so is
# linked directly.
if(MSVC)
    find_file(VROBOTS_SDK_LIBRARY
        NAMES vrobots_sdk_capi.dll.lib
        HINTS ${_vr_lib_hints}
        NO_DEFAULT_PATH
    )
    find_file(VROBOTS_SDK_RUNTIME
        NAMES vrobots_sdk_capi.dll
        HINTS ${_vr_lib_hints}
        NO_DEFAULT_PATH
    )
else()
    find_library(VROBOTS_SDK_LIBRARY
        NAMES vrobots_sdk_capi
        HINTS ${_vr_lib_hints}
        NO_DEFAULT_PATH
    )
endif()

if(NOT VROBOTS_SDK_C_INCLUDE_DIR OR NOT VROBOTS_SDK_CXX_INCLUDE_DIR)
    message(FATAL_ERROR
        "Could not find vrobots_sdk.h and/or vrobots_sdk.hpp.\n"
        "  Searched: ${_vr_include_hints}\n"
        "  Download the C bundle for your OS from this repository's GitHub Releases,\n"
        "  unpack it beside the repository, or pass -DVROBOTS_SDK_DIR=/path/to/bundle\n"
        "  or -DVROBOTS_SDK_INCLUDE_DIR=/path/to/bundle/include")
endif()

if(NOT VROBOTS_SDK_LIBRARY)
    message(FATAL_ERROR
        "Could not find the vrobots_sdk_capi library.\n"
        "  Searched: ${_vr_lib_hints}\n"
        "  Download the C bundle for your OS from this repository's GitHub Releases,\n"
        "  unpack it beside the repository, or pass -DVROBOTS_SDK_DIR=/path/to/bundle\n"
        "  or -DVROBOTS_SDK_LIBRARY=/path/to/the/library")
endif()

message(STATUS "vrobots_sdk.h    : ${VROBOTS_SDK_C_INCLUDE_DIR}")
message(STATUS "vrobots_sdk.hpp  : ${VROBOTS_SDK_CXX_INCLUDE_DIR}")
message(STATUS "vrobots_sdk_capi : ${VROBOTS_SDK_LIBRARY}")

add_library(vrobots_sdk INTERFACE)
target_include_directories(vrobots_sdk INTERFACE
    "${VROBOTS_SDK_C_INCLUDE_DIR}"
    "${VROBOTS_SDK_CXX_INCLUDE_DIR}"
)
target_link_libraries(vrobots_sdk INTERFACE "${VROBOTS_SDK_LIBRARY}")

# A cdylib built by rustc links the platform runtime itself, but the executable
# still needs the pieces the Rust std library expects from the host.
if(WIN32)
    target_link_libraries(vrobots_sdk INTERFACE ws2_32 userenv bcrypt ntdll)
elseif(UNIX AND NOT APPLE)
    find_package(Threads REQUIRED)
    target_link_libraries(vrobots_sdk INTERFACE Threads::Threads ${CMAKE_DL_LIBS} m)
    # So the examples find libvrobots_sdk_capi.so without LD_LIBRARY_PATH.
    get_filename_component(_vr_lib_dir "${VROBOTS_SDK_LIBRARY}" DIRECTORY)
    set(CMAKE_BUILD_RPATH "${_vr_lib_dir}")
endif()

# ---------------------------------------------------------------------------
# The examples
# ---------------------------------------------------------------------------

# Every example is a complete program with its own `int main` -- setup, then a
# plain loop. Settings live in the constants at the top of each file, and a
# permutation worth showing is its own file rather than a flag.
#
# The one exception is the sys_id of a SCENE-AUTHORED robot: those robots are not
# in any spawn catalog, their ids are handed out at scene load and keep
# incrementing, so no file can hard-code one. It is always a single positional
# argument -- still no flags anywhere -- and it is REQUIRED by ex29-ex33 and ex35 (whose
# robots can only be attached to) and OPTIONAL for ex21, ex22 and ex27 (which
# create their own multirotor when it is left off).
#
# Add a name here and CMake picks up `<name>.cpp` beside this file.
set(VROBOTS_EXAMPLES
    ex01_hello_states
    ex02_hello_control
    ex03_hello_image
    ex04_hello_service
    ex05_hello_car
    ex06_hello_throttle
    ex07_body_wrench
    ex08_generic_cmd
    ex09_state_paced_loop
    ex10_sensors_tour
    ex11_topic_discovery
    ex12_version_info
    ex13_open_camera
    ex14_camera_save
    ex15_camera_formats
    ex16_two_cameras
    ex17_camera_pose
    ex18_multi_robot
    ex19_robust_loop
    ex20_logging_tour
    ex21_reset
    ex22_physical_params
    ex23_skins
    ex24_sensor_config
    ex25_frames
    ex26_drive_config
    ex27_rotor_config
    ex28_hello_msd
    ex29_hello_cartpole
    ex30_hello_halfdrone
    ex31_globalhawk_direct
    ex32_fw_rate_controller
    ex33_fw_est_source
    ex35_publish_estimate
    ex36_rotations
)

foreach(example IN LISTS VROBOTS_EXAMPLES)
    add_executable(${example} "${CMAKE_CURRENT_SOURCE_DIR}/${example}.cpp")
    target_link_libraries(${example} PRIVATE vrobots_sdk)
    if(MSVC)
        target_compile_options(${example} PRIVATE /W4 /permissive- /utf-8)
        # The examples use plain <cstdio> on purpose so they read the same on
        # every platform; MSVC's "consider fopen_s" advice does not apply.
        target_compile_definitions(${example} PRIVATE _CRT_SECURE_NO_WARNINGS)
        # The DLL has to sit beside the .exe: Windows has no rpath.
        if(VROBOTS_SDK_RUNTIME)
            add_custom_command(TARGET ${example} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${VROBOTS_SDK_RUNTIME}" "$<TARGET_FILE_DIR:${example}>"
                COMMENT "copying vrobots_sdk_capi.dll next to ${example}")
        endif()
    else()
        target_compile_options(${example} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endforeach()

# ---------------------------------------------------------------------------
# ex34: the one example with a dependency outside the SDK
# ---------------------------------------------------------------------------

# Built only when OpenCV is installed, so a machine without it still builds
# every other example (and CI stays green). Point CMake at a private install
# with -DOpenCV_DIR=/path/to/opencv/build.
find_package(OpenCV QUIET COMPONENTS core imgproc highgui)
if(OpenCV_FOUND)
    message(STATUS "OpenCV ${OpenCV_VERSION} : building ex34_camera_view")
    add_executable(ex34_camera_view "${CMAKE_CURRENT_SOURCE_DIR}/ex34_camera_view.cpp")
    target_link_libraries(ex34_camera_view PRIVATE vrobots_sdk ${OpenCV_LIBS})
    if(MSVC)
        target_compile_options(ex34_camera_view PRIVATE /W4 /permissive- /utf-8)
        target_compile_definitions(ex34_camera_view PRIVATE _CRT_SECURE_NO_WARNINGS)
        if(VROBOTS_SDK_RUNTIME)
            add_custom_command(TARGET ex34_camera_view POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${VROBOTS_SDK_RUNTIME}" "$<TARGET_FILE_DIR:ex34_camera_view>"
                COMMENT "copying vrobots_sdk_capi.dll next to ex34_camera_view")
        endif()
    else()
        target_compile_options(ex34_camera_view PRIVATE -Wall -Wextra -Wpedantic)
    endif()
else()
    message(STATUS "OpenCV not found : skipping ex34_camera_view")
endif()

# A sim-less smoke test: proves the library links, the ABI matches and the error
# path works, without Unity. `ctest` runs it; CI builds it as part of the loop
# above.
enable_testing()
add_executable(smoke_test "${CMAKE_CURRENT_SOURCE_DIR}/smoke_test.cpp")
target_link_libraries(smoke_test PRIVATE vrobots_sdk)
if(MSVC)
    target_compile_options(smoke_test PRIVATE /W4 /permissive- /utf-8)
    target_compile_definitions(smoke_test PRIVATE _CRT_SECURE_NO_WARNINGS)
    if(VROBOTS_SDK_RUNTIME)
        add_custom_command(TARGET smoke_test POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${VROBOTS_SDK_RUNTIME}" "$<TARGET_FILE_DIR:smoke_test>"
            COMMENT "copying vrobots_sdk_capi.dll next to smoke_test")
    endif()
else()
    target_compile_options(smoke_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME smoke_test COMMAND smoke_test)
