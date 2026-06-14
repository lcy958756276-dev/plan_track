########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

list(APPEND ceres-solver_COMPONENT_NAMES Ceres::ceres)
list(REMOVE_DUPLICATES ceres-solver_COMPONENT_NAMES)
list(APPEND ceres-solver_FIND_DEPENDENCY_NAMES Eigen3 glog)
list(REMOVE_DUPLICATES ceres-solver_FIND_DEPENDENCY_NAMES)
set(Eigen3_FIND_MODE "NO_MODULE")
set(glog_FIND_MODE "NO_MODULE")

########### VARIABLES #######################################################################
#############################################################################################
set(ceres-solver_PACKAGE_FOLDER_RELEASE "/home/lcy/.conan/data/ceres-solver/1.14.0/_/_/package/0f9698a1b33a9f1c0a2424ab168611b7b74df414")
set(ceres-solver_BUILD_MODULES_PATHS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/lib/cmake/conan-official-ceres-solver-variables.cmake")


set(ceres-solver_INCLUDE_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/include"
			"${ceres-solver_PACKAGE_FOLDER_RELEASE}/include/ceres")
set(ceres-solver_RES_DIRS_RELEASE )
set(ceres-solver_DEFINITIONS_RELEASE )
set(ceres-solver_SHARED_LINK_FLAGS_RELEASE )
set(ceres-solver_EXE_LINK_FLAGS_RELEASE )
set(ceres-solver_OBJECTS_RELEASE )
set(ceres-solver_COMPILE_DEFINITIONS_RELEASE )
set(ceres-solver_COMPILE_OPTIONS_C_RELEASE )
set(ceres-solver_COMPILE_OPTIONS_CXX_RELEASE )
set(ceres-solver_LIB_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/lib")
set(ceres-solver_LIBS_RELEASE ceres)
set(ceres-solver_SYSTEM_LIBS_RELEASE m)
set(ceres-solver_FRAMEWORK_DIRS_RELEASE )
set(ceres-solver_FRAMEWORKS_RELEASE )
set(ceres-solver_BUILD_DIRS_RELEASE )

# COMPOUND VARIABLES
set(ceres-solver_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${ceres-solver_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${ceres-solver_COMPILE_OPTIONS_C_RELEASE}>")
set(ceres-solver_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${ceres-solver_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${ceres-solver_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${ceres-solver_EXE_LINK_FLAGS_RELEASE}>")


set(ceres-solver_COMPONENTS_RELEASE Ceres::ceres)
########### COMPONENT Ceres::ceres VARIABLES ############################################

set(ceres-solver_Ceres_ceres_INCLUDE_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/include"
			"${ceres-solver_PACKAGE_FOLDER_RELEASE}/include/ceres")
set(ceres-solver_Ceres_ceres_LIB_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/lib")
set(ceres-solver_Ceres_ceres_RES_DIRS_RELEASE )
set(ceres-solver_Ceres_ceres_DEFINITIONS_RELEASE )
set(ceres-solver_Ceres_ceres_OBJECTS_RELEASE )
set(ceres-solver_Ceres_ceres_COMPILE_DEFINITIONS_RELEASE )
set(ceres-solver_Ceres_ceres_COMPILE_OPTIONS_C_RELEASE "")
set(ceres-solver_Ceres_ceres_COMPILE_OPTIONS_CXX_RELEASE "")
set(ceres-solver_Ceres_ceres_LIBS_RELEASE ceres)
set(ceres-solver_Ceres_ceres_SYSTEM_LIBS_RELEASE m)
set(ceres-solver_Ceres_ceres_FRAMEWORK_DIRS_RELEASE )
set(ceres-solver_Ceres_ceres_FRAMEWORKS_RELEASE )
set(ceres-solver_Ceres_ceres_DEPENDENCIES_RELEASE Eigen3::Eigen glog::glog)
set(ceres-solver_Ceres_ceres_SHARED_LINK_FLAGS_RELEASE )
set(ceres-solver_Ceres_ceres_EXE_LINK_FLAGS_RELEASE )
# COMPOUND VARIABLES
set(ceres-solver_Ceres_ceres_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${ceres-solver_Ceres_ceres_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${ceres-solver_Ceres_ceres_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${ceres-solver_Ceres_ceres_EXE_LINK_FLAGS_RELEASE}>
)
set(ceres-solver_Ceres_ceres_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${ceres-solver_Ceres_ceres_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${ceres-solver_Ceres_ceres_COMPILE_OPTIONS_C_RELEASE}>")