cmake_minimum_required(VERSION 2.8)

project(dart_lcm)

# enable C++ 11
add_definitions(-std=c++11)

add_definitions ("-Wall")

include(ExternalProject)

ExternalProject_Add(lcm_types
    GIT_REPOSITORY https://github.com/openhumanoids/bot_core_lcmtypes.git
    INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lcm_types
    INSTALL_COMMAND make install
    )

ExternalProject_Get_Property(lcm_types binary_dir)

find_package(CUDA REQUIRED)
find_package(Boost REQUIRED thread system)
find_package(PkgConfig)
pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(dart REQUIRED dart cuda-7.5)

# LCM
include_directories(${lcm_INCLUDE_DIRS})
include_directories(${binary_dir}/lcmgen)

# DART
include_directories(${dart_INCLUDE_DIRS})
include_directories(${dart_dart_INCLUDEDIR}/dart)
link_directories(${dart_LIBRARY_DIRS})

# CUDA
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc)
add_definitions(-DCUDA_BUILD)

# Boost
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(SRC_LIST
    src/lcm_provider_base.cpp
    src/dart_lcm_joints.cpp
    src/lcm_state_publish.cpp
    src/lcm_frame_pose_publish.cpp
    )

set(HDR_LIST
    include/dart_lcm_depth_provider.hpp
    include/dart_lcm_joints.hpp
    include/lcm_state_publish.hpp
    include/lcm_frame_pose_publish.hpp
    include/lcm_state_merge.hpp
    include/utils.hpp
    )

include_directories(include/)
add_library(dart_lcm SHARED ${SRC_LIST} ${HDR_LIST})
target_link_libraries(dart_lcm ${lcm_LIBRARIES})
target_link_libraries(dart_lcm ${dart_LIBRARIES})
target_link_libraries(dart_lcm ${Boost_SYSTEM_LIBRARIES} ${Boost_THREAD_LIBRARIES})


## install
install(TARGETS dart_lcm LIBRARY DESTINATION lib)  # shared library
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# TODO: do we need these LCM types later?
install(DIRECTORY ${binary_dir}/lcmgen/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")

# setup pkg-config files
SET(PKG_CONFIG_REQUIRES "dart lcm")
SET(PKG_CONFIG_LIBDIR       "\${prefix}/lib" )
SET(PKG_CONFIG_INCLUDEDIR   "\${prefix}/include" )
SET(PKG_CONFIG_LIBS         "-L\${libdir} -ldart_lcm" )
SET(PKG_CONFIG_CFLAGS       "-I\${includedir}" )

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/pkg-config.pc.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/dart_lcm.pc"
)

INSTALL(FILES "${CMAKE_BINARY_DIR}/dart_lcm.pc"
        DESTINATION lib/pkgconfig)
