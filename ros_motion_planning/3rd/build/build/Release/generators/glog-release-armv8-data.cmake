########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

set(glog_COMPONENT_NAMES "")
list(APPEND glog_FIND_DEPENDENCY_NAMES gflags libunwind)
list(REMOVE_DUPLICATES glog_FIND_DEPENDENCY_NAMES)
set(gflags_FIND_MODE "NO_MODULE")
set(libunwind_FIND_MODE "NO_MODULE")

########### VARIABLES #######################################################################
#############################################################################################
set(glog_PACKAGE_FOLDER_RELEASE "/home/lcy/.conan/data/glog/0.6.0/_/_/package/0c86a9905d0d18d33ee45f13b9dcafe6555fd5ca")
set(glog_BUILD_MODULES_PATHS_RELEASE )


set(glog_INCLUDE_DIRS_RELEASE "${glog_PACKAGE_FOLDER_RELEASE}/include")
set(glog_RES_DIRS_RELEASE )
set(glog_DEFINITIONS_RELEASE )
set(glog_SHARED_LINK_FLAGS_RELEASE )
set(glog_EXE_LINK_FLAGS_RELEASE )
set(glog_OBJECTS_RELEASE )
set(glog_COMPILE_DEFINITIONS_RELEASE )
set(glog_COMPILE_OPTIONS_C_RELEASE )
set(glog_COMPILE_OPTIONS_CXX_RELEASE )
set(glog_LIB_DIRS_RELEASE "${glog_PACKAGE_FOLDER_RELEASE}/lib")
set(glog_LIBS_RELEASE glog)
set(glog_SYSTEM_LIBS_RELEASE pthread)
set(glog_FRAMEWORK_DIRS_RELEASE )
set(glog_FRAMEWORKS_RELEASE )
set(glog_BUILD_DIRS_RELEASE "${glog_PACKAGE_FOLDER_RELEASE}/")

# COMPOUND VARIABLES
set(glog_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${glog_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${glog_COMPILE_OPTIONS_C_RELEASE}>")
set(glog_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${glog_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${glog_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${glog_EXE_LINK_FLAGS_RELEASE}>")


set(glog_COMPONENTS_RELEASE )