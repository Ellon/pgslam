cmake_minimum_required(VERSION 3.1.0)
project(pgslam VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED COMPONENTS graph
    # Below are the components required by GTSAM
    timer
    serialization
    system
    filesystem
    thread
    date_time
    regex
    timer
    chrono)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(GTSAM REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME} INTERFACE 
  ${libpointmatcher_LIBRARIES}
  Boost::graph
  gtsam
)

target_include_directories(${PROJECT_NAME} INTERFACE
    ${EIGEN3_INCLUDE_DIRS}
    ${libpointmatcher_INCLUDE_DIRS} # For some reason I cannot use generator expression INSTALL_INTERFACE here...
    $<INSTALL_INTERFACE:include/>
)

# Introduce variables:
# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)

# Layout. This works for all platforms:
#   * <prefix>/lib*/cmake/<PROJECT-NAME>
#   * <prefix>/lib*/
#   * <prefix>/include/
set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Configuration
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")

# Include module with fuctions:
#   * 'write_basic_package_version_file'
#   * 'configure_package_config_file'
include(CMakePackageConfigHelpers)

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Uses:
#   * PROJECT_VERSION
write_basic_package_version_file(
  "${version_config}" COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>Config.cmake'
# Uses variables:
#   * TARGETS_EXPORT_NAME
#   * PROJECT_NAME
#   * CMAKE_INSTALL_INCLUDEDIR
configure_package_config_file(
    "cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
    PATH_VARS
      CMAKE_INSTALL_INCLUDEDIR
)


####
# Installation

# Targets:
#   * <prefix>/lib/lib${PROJECT_NAME}.so
#   * header location after install: <prefix>/include/${PROJECT_NAME}/*.h
#   * headers can be included by C++ code `#include <${PROJECT_NAME}/*.h>`
install(
    TARGETS ${PROJECT_NAME}
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
)

# Config
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake
install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
)

# Header files
install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

# Config
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
)
