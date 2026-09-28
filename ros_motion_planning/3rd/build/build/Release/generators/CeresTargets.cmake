# Load the debug and release variables
get_filename_component(_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
file(GLOB DATA_FILES "${_DIR}/Ceres-*-data.cmake")

foreach(f ${DATA_FILES})
    include(${f})
endforeach()

# Create the targets for all the components
foreach(_COMPONENT ${ceres-solver_COMPONENT_NAMES} )
    if(NOT TARGET ${_COMPONENT})
        add_library(${_COMPONENT} INTERFACE IMPORTED)
        message(${Ceres_MESSAGE_MODE} "Conan: Component target declared '${_COMPONENT}'")
    endif()
endforeach()

if(NOT TARGET Ceres::ceres)
    add_library(Ceres::ceres INTERFACE IMPORTED)
    message(${Ceres_MESSAGE_MODE} "Conan: Target declared 'Ceres::ceres'")
endif()
if(NOT TARGET ceres)
    add_library(ceres INTERFACE IMPORTED)
    set_property(TARGET ceres PROPERTY INTERFACE_LINK_LIBRARIES Ceres::ceres)
else()
    message(WARNING "Target name 'ceres' already exists.")
endif()
# Load the debug and release library finders
get_filename_component(_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
file(GLOB CONFIG_FILES "${_DIR}/Ceres-Target-*.cmake")

foreach(f ${CONFIG_FILES})
    include(${f})
endforeach()