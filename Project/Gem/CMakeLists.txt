
set(gem_path ${CMAKE_CURRENT_LIST_DIR})
set(gem_json ${gem_path}/gem.json)
o3de_restricted_path(${gem_json} gem_restricted_path gem_parent_relative_path)

# Currently we are in the Playground/Gem folder: ${CMAKE_CURRENT_LIST_DIR}
# Get the platform specific folder ${pal_dir} for the current folder: ${CMAKE_CURRENT_LIST_DIR}/Platform/${PAL_PLATFORM_NAME}
# Note: o3de_pal_dir will take care of the details for us, as this may be a restricted platform
#       in which case it will see if that platform is present here or in the restricted folder.
#       i.e. It could here : Playground/Gem/Platform/<platform_name>  or
#            <restricted_folder>/<platform_name>/Playground/Gem
o3de_pal_dir(pal_dir ${CMAKE_CURRENT_LIST_DIR}/Platform/${PAL_PLATFORM_NAME} "${gem_restricted_path}" "${gem_path}" "${gem_parent_relative_path}")

# Now that we have the platform abstraction layer (PAL) folder for this folder, thats where we will find the
# traits for this platform. Traits for a platform are defines for things like whether or not something in this project
# is supported by this platform.
include(${pal_dir}/PAL_${PAL_PLATFORM_NAME_LOWERCASE}.cmake)

# Now that we have loaded our project traits for this platform, see if this project is even supported on this platform.
# If its not supported we just return after including the unsupported.
if(NOT PAL_TRAIT_PLAYGROUND_SUPPORTED)
    return()
endif()

if(NOT COMMAND target_depends_on_ros2_packages)
    function(target_depends_on_ros2_package TARGET_NAME)
        list(GET ARGN 0 _package)
        find_package(${ARGN})
        if(${${_package}_FOUND})
            target_link_libraries(${TARGET_NAME} PUBLIC ${${_package}_TARGETS})
        endif()
    endfunction()
    function(target_depends_on_ros2_packages TARGET_NAME)
        foreach(_package IN LISTS ARGN)
            target_depends_on_ros2_package(${TARGET_NAME} ${_package} REQUIRED)
        endforeach()
    endfunction()
endif()

# We are on a supported platform, so add the Playground target
# Note: We include the common files and the platform specific files which are set in playground_files.cmake and
# in ${pal_dir}/playground_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
ly_add_target(
    NAME Playground.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        playground_files.cmake
        ${pal_dir}/playground_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    PLATFORM_INCLUDE_FILES
        ${CMAKE_CURRENT_LIST_DIR}/Platform/Common/${PAL_TRAIT_COMPILER_ID}/exceptions_${PAL_TRAIT_COMPILER_ID_LOWERCASE}.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            Gem::ROS2.Static
        PRIVATE
            AZ::AzFramework
            Gem::PhysX5.Static
)

target_depends_on_ros2_packages(Playground.Static std_srvs nav_msgs sensor_msgs)

ly_add_target(
    NAME Playground ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        playground_shared_files.cmake
        ${pal_dir}/playground_shared_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::Playground.Static
            AZ::AzCore
            AZ::AzFramework
)

# By default, we will specify that the above target Playground would be used by
# Client and Server type targets when this gem is enabled.  If you don't want it
# active in Clients or Servers by default, delete one or more of the following lines:
ly_create_alias(NAME Playground.Clients  NAMESPACE Gem TARGETS Gem::Playground)
ly_create_alias(NAME Playground.Servers  NAMESPACE Gem TARGETS Gem::Playground)

################################################################################
# Gem dependencies
################################################################################

# Enable the specified list of gems from GEM_FILE or GEMS list for this specific project:
ly_enable_gems(PROJECT_NAME Playground GEM_FILE enabled_gems.cmake)
