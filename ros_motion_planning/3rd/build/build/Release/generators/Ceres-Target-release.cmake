# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(ceres-solver_FRAMEWORKS_FOUND_RELEASE "") # Will be filled later
conan_find_apple_frameworks(ceres-solver_FRAMEWORKS_FOUND_RELEASE "${ceres-solver_FRAMEWORKS_RELEASE}" "${ceres-solver_FRAMEWORK_DIRS_RELEASE}")

set(ceres-solver_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET ceres-solver_DEPS_TARGET)
    add_library(ceres-solver_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET ceres-solver_DEPS_TARGET
             PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Release>:${ceres-solver_FRAMEWORKS_FOUND_RELEASE}>
             $<$<CONFIG:Release>:${ceres-solver_SYSTEM_LIBS_RELEASE}>
             $<$<CONFIG:Release>:Eigen3::Eigen;glog::glog>
             APPEND)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### ceres-solver_DEPS_TARGET to all of them
conan_package_library_targets("${ceres-solver_LIBS_RELEASE}"    # libraries
                              "${ceres-solver_LIB_DIRS_RELEASE}" # package_libdir
                              ceres-solver_DEPS_TARGET
                              ceres-solver_LIBRARIES_TARGETS  # out_libraries_targets
                              "_RELEASE"
                              "ceres-solver")    # package_name

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${ceres-solver_BUILD_DIRS_RELEASE} ${CMAKE_MODULE_PATH})

########## COMPONENTS TARGET PROPERTIES Release ########################################

    ########## COMPONENT Ceres::ceres #############

        set(ceres-solver_Ceres_ceres_FRAMEWORKS_FOUND_RELEASE "")
        conan_find_apple_frameworks(ceres-solver_Ceres_ceres_FRAMEWORKS_FOUND_RELEASE "${ceres-solver_Ceres_ceres_FRAMEWORKS_RELEASE}" "${ceres-solver_Ceres_ceres_FRAMEWORK_DIRS_RELEASE}")

        set(ceres-solver_Ceres_ceres_LIBRARIES_TARGETS "")

        ######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
        if(NOT TARGET ceres-solver_Ceres_ceres_DEPS_TARGET)
            add_library(ceres-solver_Ceres_ceres_DEPS_TARGET INTERFACE IMPORTED)
        endif()

        set_property(TARGET ceres-solver_Ceres_ceres_DEPS_TARGET
                     PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_FRAMEWORKS_FOUND_RELEASE}>
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_SYSTEM_LIBS_RELEASE}>
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_DEPENDENCIES_RELEASE}>
                     APPEND)

        ####### Find the libraries declared in cpp_info.component["xxx"].libs,
        ####### create an IMPORTED target for each one and link the 'ceres-solver_Ceres_ceres_DEPS_TARGET' to all of them
        conan_package_library_targets("${ceres-solver_Ceres_ceres_LIBS_RELEASE}"
                                      "${ceres-solver_Ceres_ceres_LIB_DIRS_RELEASE}"
                                      ceres-solver_Ceres_ceres_DEPS_TARGET
                                      ceres-solver_Ceres_ceres_LIBRARIES_TARGETS
                                      "_RELEASE"
                                      "ceres-solver_Ceres_ceres")

        ########## TARGET PROPERTIES #####################################
        set_property(TARGET Ceres::ceres
                     PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_OBJECTS_RELEASE}>
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_LIBRARIES_TARGETS}>
                     APPEND)

        if("${ceres-solver_Ceres_ceres_LIBS_RELEASE}" STREQUAL "")
            # If the component is not declaring any "cpp_info.components['foo'].libs" the system, frameworks etc are not
            # linked to the imported targets and we need to do it to the global target
            set_property(TARGET Ceres::ceres
                         PROPERTY INTERFACE_LINK_LIBRARIES
                         ceres-solver_Ceres_ceres_DEPS_TARGET
                         APPEND)
        endif()

        set_property(TARGET Ceres::ceres PROPERTY INTERFACE_LINK_OPTIONS
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_LINKER_FLAGS_RELEASE}> APPEND)
        set_property(TARGET Ceres::ceres PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_INCLUDE_DIRS_RELEASE}> APPEND)
        set_property(TARGET Ceres::ceres PROPERTY INTERFACE_COMPILE_DEFINITIONS
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_COMPILE_DEFINITIONS_RELEASE}> APPEND)
        set_property(TARGET Ceres::ceres PROPERTY INTERFACE_COMPILE_OPTIONS
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_COMPILE_OPTIONS_RELEASE}> APPEND)

    ########## AGGREGATED GLOBAL TARGET WITH THE COMPONENTS #####################
    set_property(TARGET Ceres::ceres PROPERTY INTERFACE_LINK_LIBRARIES Ceres::ceres APPEND)

########## For the modules (FindXXX)
set(ceres-solver_LIBRARIES_RELEASE Ceres::ceres)
