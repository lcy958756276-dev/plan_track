set(CMAKE_C_COMPILER "/usr/bin/cc")
set(CMAKE_C_COMPILER_ARG1 "")
set(CMAKE_C_COMPILER_ID "GNU")
set(CMAKE_C_COMPILER_VERSION "9.4.0")
set(CMAKE_C_COMPILER_VERSION_INTERNAL "")
set(CMAKE_C_COMPILER_WRAPPER "")
set(CMAKE_C_STANDARD_COMPUTED_DEFAULT "11")
set(CMAKE_C_COMPILE_FEATURES "c_std_90;c_function_prototypes;c_std_99;c_restrict;c_variadic_macros;c_std_11;c_static_assert")
set(CMAKE_C90_COMPILE_FEATURES "c_std_90;c_function_prototypes")
set(CMAKE_C99_COMPILE_FEATURES "c_std_99;c_restrict;c_variadic_macros")
set(CMAKE_C11_COMPILE_FEATURES "c_std_11;c_static_assert")

set(CMAKE_C_PLATFORM_ID "Linux")
set(CMAKE_C_SIMULATE_ID "")
set(CMAKE_C_COMPILER_FRONTEND_VARIANT "")
set(CMAKE_C_SIMULATE_VERSION "")



set(CMAKE_AR "/usr/bin/ar")
set(CMAKE_C_COMPILER_AR "/usr/bin/gcc-ar-9")
set(CMAKE_RANLIB "/usr/bin/ranlib")
set(CMAKE_C_COMPILER_RANLIB "/usr/bin/gcc-ranlib-9")
set(CMAKE_LINKER "/usr/bin/ld")
set(CMAKE_MT "")
set(CMAKE_COMPILER_IS_GNUCC 1)
set(CMAKE_C_COMPILER_LOADED 1)
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_C_ABI_COMPILED TRUE)
set(CMAKE_COMPILER_IS_MINGW )
set(CMAKE_COMPILER_IS_CYGWIN )
if(CMAKE_COMPILER_IS_CYGWIN)
  set(CYGWIN 1)
  set(UNIX 1)
endif()

set(CMAKE_C_COMPILER_ENV_VAR "CC")

if(CMAKE_COMPILER_IS_MINGW)
  set(MINGW 1)
endif()
set(CMAKE_C_COMPILER_ID_RUN 1)
set(CMAKE_C_SOURCE_FILE_EXTENSIONS c;m)
set(CMAKE_C_IGNORE_EXTENSIONS h;H;o;O;obj;OBJ;def;DEF;rc;RC)
set(CMAKE_C_LINKER_PREFERENCE 10)

# Save compiler ABI information.
set(CMAKE_C_SIZEOF_DATA_PTR "8")
set(CMAKE_C_COMPILER_ABI "ELF")
set(CMAKE_C_LIBRARY_ARCHITECTURE "aarch64-linux-gnu")

if(CMAKE_C_SIZEOF_DATA_PTR)
  set(CMAKE_SIZEOF_VOID_P "${CMAKE_C_SIZEOF_DATA_PTR}")
endif()

if(CMAKE_C_COMPILER_ABI)
  set(CMAKE_INTERNAL_PLATFORM_ABI "${CMAKE_C_COMPILER_ABI}")
endif()

if(CMAKE_C_LIBRARY_ARCHITECTURE)
  set(CMAKE_LIBRARY_ARCHITECTURE "aarch64-linux-gnu")
endif()

set(CMAKE_C_CL_SHOWINCLUDES_PREFIX "")
if(CMAKE_C_CL_SHOWINCLUDES_PREFIX)
  set(CMAKE_CL_SHOWINCLUDES_PREFIX "${CMAKE_C_CL_SHOWINCLUDES_PREFIX}")
endif()





set(CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES "/usr/lib/gcc/aarch64-linux-gnu/9/include;/usr/local/include;/usr/include/aarch64-linux-gnu;/usr/include")
set(CMAKE_C_IMPLICIT_LINK_LIBRARIES "gcc;gcc_s;c;gcc;gcc_s")
set(CMAKE_C_IMPLICIT_LINK_DIRECTORIES "/home/lcy/.conan/data/gflags/2.2.2/_/_/package/0bbdf01a2de39319f345d77184fc6dd2dd074592/lib;/home/lcy/.conan/data/glog/0.6.0/_/_/package/0c86a9905d0d18d33ee45f13b9dcafe6555fd5ca/lib;/home/lcy/.conan/data/ceres-solver/1.14.0/_/_/package/0f9698a1b33a9f1c0a2424ab168611b7b74df414/lib;/home/lcy/.conan/data/osqp/0.6.3/_/_/package/69f8db40e683434496cb58e4d4030f8979815e1a/lib;/home/lcy/.conan/data/libunwind/1.8.0/_/_/package/0c85726fc4c915811c7ddfb90f7ff8ab6b4096bc/lib;/usr/lib/gcc/aarch64-linux-gnu/9;/usr/lib/aarch64-linux-gnu;/usr/lib;/lib/aarch64-linux-gnu;/lib")
set(CMAKE_C_IMPLICIT_LINK_FRAMEWORK_DIRECTORIES "")
