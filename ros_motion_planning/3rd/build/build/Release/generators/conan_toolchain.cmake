

# Conan automatically generated toolchain file
# DO NOT EDIT MANUALLY, it will be overwritten

# Avoid including toolchain file several times (bad if appending to variables like
#   CMAKE_CXX_FLAGS. See https://github.com/android/ndk/issues/323
include_guard()

message(STATUS "Using Conan toolchain: ${CMAKE_CURRENT_LIST_FILE}")

if(${CMAKE_VERSION} VERSION_LESS "3.15")
    message(FATAL_ERROR "The 'CMakeToolchain' generator only works with CMake >= 3.15")
endif()












# Extra c, cxx, linkflags and defines


if(DEFINED CONAN_CXX_FLAGS)
  string(APPEND CMAKE_CXX_FLAGS_INIT " ${CONAN_CXX_FLAGS}")
endif()
if(DEFINED CONAN_C_FLAGS)
  string(APPEND CMAKE_C_FLAGS_INIT " ${CONAN_C_FLAGS}")
endif()
if(DEFINED CONAN_SHARED_LINKER_FLAGS)
  string(APPEND CMAKE_SHARED_LINKER_FLAGS_INIT " ${CONAN_SHARED_LINKER_FLAGS}")
endif()
if(DEFINED CONAN_EXE_LINKER_FLAGS)
  string(APPEND CMAKE_EXE_LINKER_FLAGS_INIT " ${CONAN_EXE_LINKER_FLAGS}")
endif()

get_property( _CMAKE_IN_TRY_COMPILE GLOBAL PROPERTY IN_TRY_COMPILE )
if(_CMAKE_IN_TRY_COMPILE)
    message(STATUS "Running toolchain IN_TRY_COMPILE")
    return()
endif()

set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

# Definition of CMAKE_MODULE_PATH
# The root (which is the default builddirs) path of dependencies in the host context
list(PREPEND CMAKE_MODULE_PATH "/home/lcy/.conan/data/osqp/0.6.3/_/_/package/69f8db40e683434496cb58e4d4030f8979815e1a/" "/home/lcy/.conan/data/glog/0.6.0/_/_/package/0c86a9905d0d18d33ee45f13b9dcafe6555fd5ca/" "/home/lcy/.conan/data/gflags/2.2.2/_/_/package/0bbdf01a2de39319f345d77184fc6dd2dd074592/" "/home/lcy/.conan/data/xz_utils/5.4.5/_/_/package/c3baf9fae083edda2e0c5ef3337b6a111016a898/" "/home/lcy/.conan/data/zlib/1.3.1/_/_/package/c3baf9fae083edda2e0c5ef3337b6a111016a898/")
# the generators folder (where conan generates files, like this toolchain)
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

# Definition of CMAKE_PREFIX_PATH, CMAKE_XXXXX_PATH
# The Conan local "generators" folder, where this toolchain is saved.
list(PREPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_LIST_DIR} )
list(PREPEND CMAKE_PROGRAM_PATH "/home/lcy/.conan/data/osqp/0.6.3/_/_/package/69f8db40e683434496cb58e4d4030f8979815e1a/bin" "/home/lcy/.conan/data/ceres-solver/1.14.0/_/_/package/0f9698a1b33a9f1c0a2424ab168611b7b74df414/bin" "/home/lcy/.conan/data/glog/0.6.0/_/_/package/0c86a9905d0d18d33ee45f13b9dcafe6555fd5ca/bin" "/home/lcy/.conan/data/gflags/2.2.2/_/_/package/0bbdf01a2de39319f345d77184fc6dd2dd074592/bin" "/home/lcy/.conan/data/libunwind/1.8.0/_/_/package/0c85726fc4c915811c7ddfb90f7ff8ab6b4096bc/bin" "/home/lcy/.conan/data/xz_utils/5.4.5/_/_/package/c3baf9fae083edda2e0c5ef3337b6a111016a898/bin" "/home/lcy/.conan/data/zlib/1.3.1/_/_/package/c3baf9fae083edda2e0c5ef3337b6a111016a898/bin")
list(PREPEND CMAKE_LIBRARY_PATH "/home/lcy/.conan/data/osqp/0.6.3/_/_/package/69f8db40e683434496cb58e4d4030f8979815e1a/lib" "/home/lcy/.conan/data/ceres-solver/1.14.0/_/_/package/0f9698a1b33a9f1c0a2424ab168611b7b74df414/lib" "/home/lcy/.conan/data/glog/0.6.0/_/_/package/0c86a9905d0d18d33ee45f13b9dcafe6555fd5ca/lib" "/home/lcy/.conan/data/gflags/2.2.2/_/_/package/0bbdf01a2de39319f345d77184fc6dd2dd074592/lib" "/home/lcy/.conan/data/libunwind/1.8.0/_/_/package/0c85726fc4c915811c7ddfb90f7ff8ab6b4096bc/lib" "/home/lcy/.conan/data/xz_utils/5.4.5/_/_/package/c3baf9fae083edda2e0c5ef3337b6a111016a898/lib" "/home/lcy/.conan/data/zlib/1.3.1/_/_/package/c3baf9fae083edda2e0c5ef3337b6a111016a898/lib")
list(PREPEND CMAKE_INCLUDE_PATH "/home/lcy/.conan/data/osqp/0.6.3/_/_/package/69f8db40e683434496cb58e4d4030f8979815e1a/include" "/home/lcy/.conan/data/ceres-solver/1.14.0/_/_/package/0f9698a1b33a9f1c0a2424ab168611b7b74df414/include" "/home/lcy/.conan/data/ceres-solver/1.14.0/_/_/package/0f9698a1b33a9f1c0a2424ab168611b7b74df414/include/ceres" "/home/lcy/.conan/data/eigen/3.4.0/_/_/package/5ab84d6acfe1f23c4fae0ab88f26e3a396351ac9/include/eigen3" "/home/lcy/.conan/data/glog/0.6.0/_/_/package/0c86a9905d0d18d33ee45f13b9dcafe6555fd5ca/include" "/home/lcy/.conan/data/gflags/2.2.2/_/_/package/0bbdf01a2de39319f345d77184fc6dd2dd074592/include" "/home/lcy/.conan/data/libunwind/1.8.0/_/_/package/0c85726fc4c915811c7ddfb90f7ff8ab6b4096bc/include" "/home/lcy/.conan/data/xz_utils/5.4.5/_/_/package/c3baf9fae083edda2e0c5ef3337b6a111016a898/include" "/home/lcy/.conan/data/zlib/1.3.1/_/_/package/c3baf9fae083edda2e0c5ef3337b6a111016a898/include")



if (DEFINED ENV{PKG_CONFIG_PATH})
set(ENV{PKG_CONFIG_PATH} "/home/lcy/robot_graduation1/ros_motion_planning/3rd/build/build/Release/generators:$ENV{PKG_CONFIG_PATH}")
else()
set(ENV{PKG_CONFIG_PATH} "/home/lcy/robot_graduation1/ros_motion_planning/3rd/build/build/Release/generators:")
endif()




# Variables
# Variables  per configuration


# Preprocessor definitions
# Preprocessor definitions per configuration
