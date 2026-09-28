########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

list(APPEND eigen_COMPONENT_NAMES Eigen3::Eigen)
list(REMOVE_DUPLICATES eigen_COMPONENT_NAMES)
set(eigen_FIND_DEPENDENCY_NAMES "")

########### VARIABLES #######################################################################
#############################################################################################
set(eigen_PACKAGE_FOLDER_RELEASE "/home/lcy/.conan/data/eigen/3.4.0/_/_/package/5ab84d6acfe1f23c4fae0ab88f26e3a396351ac9")
set(eigen_BUILD_MODULES_PATHS_RELEASE )


set(eigen_INCLUDE_DIRS_RELEASE "${eigen_PACKAGE_FOLDER_RELEASE}/include/eigen3")
set(eigen_RES_DIRS_RELEASE )
set(eigen_DEFINITIONS_RELEASE )
set(eigen_SHARED_LINK_FLAGS_RELEASE )
set(eigen_EXE_LINK_FLAGS_RELEASE )
set(eigen_OBJECTS_RELEASE )
set(eigen_COMPILE_DEFINITIONS_RELEASE )
set(eigen_COMPILE_OPTIONS_C_RELEASE )
set(eigen_COMPILE_OPTIONS_CXX_RELEASE )
set(eigen_LIB_DIRS_RELEASE )
set(eigen_LIBS_RELEASE )
set(eigen_SYSTEM_LIBS_RELEASE m)
set(eigen_FRAMEWORK_DIRS_RELEASE )
set(eigen_FRAMEWORKS_RELEASE )
set(eigen_BUILD_DIRS_RELEASE )

# COMPOUND VARIABLES
set(eigen_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${eigen_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${eigen_COMPILE_OPTIONS_C_RELEASE}>")
set(eigen_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${eigen_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${eigen_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${eigen_EXE_LINK_FLAGS_RELEASE}>")


set(eigen_COMPONENTS_RELEASE Eigen3::Eigen)
########### COMPONENT Eigen3::Eigen VARIABLES ############################################

set(eigen_Eigen3_Eigen_INCLUDE_DIRS_RELEASE "${eigen_PACKAGE_FOLDER_RELEASE}/include/eigen3")
set(eigen_Eigen3_Eigen_LIB_DIRS_RELEASE )
set(eigen_Eigen3_Eigen_RES_DIRS_RELEASE )
set(eigen_Eigen3_Eigen_DEFINITIONS_RELEASE )
set(eigen_Eigen3_Eigen_OBJECTS_RELEASE )
set(eigen_Eigen3_Eigen_COMPILE_DEFINITIONS_RELEASE )
set(eigen_Eigen3_Eigen_COMPILE_OPTIONS_C_RELEASE "")
set(eigen_Eigen3_Eigen_COMPILE_OPTIONS_CXX_RELEASE "")
set(eigen_Eigen3_Eigen_LIBS_RELEASE )
set(eigen_Eigen3_Eigen_SYSTEM_LIBS_RELEASE m)
set(eigen_Eigen3_Eigen_FRAMEWORK_DIRS_RELEASE )
set(eigen_Eigen3_Eigen_FRAMEWORKS_RELEASE )
set(eigen_Eigen3_Eigen_DEPENDENCIES_RELEASE )
set(eigen_Eigen3_Eigen_SHARED_LINK_FLAGS_RELEASE )
set(eigen_Eigen3_Eigen_EXE_LINK_FLAGS_RELEASE )
# COMPOUND VARIABLES
set(eigen_Eigen3_Eigen_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${eigen_Eigen3_Eigen_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${eigen_Eigen3_Eigen_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${eigen_Eigen3_Eigen_EXE_LINK_FLAGS_RELEASE}>
)
set(eigen_Eigen3_Eigen_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${eigen_Eigen3_Eigen_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${eigen_Eigen3_Eigen_COMPILE_OPTIONS_C_RELEASE}>")