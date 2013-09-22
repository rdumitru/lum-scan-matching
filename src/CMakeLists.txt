################################################################################
#-------------------------------------------------------------------------------
# Dependencies.
#-------------------------------------------------------------------------------
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#-------------------------------------------------------------------------------
# Source files.
#------------------------------------------------------------------------------
#file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")

#-------------------------------------------------------------------------------
# Executables.
#-------------------------------------------------------------------------------
set(CORE_LIBS ${PCL_LIBRARIES})
set(USER_LIBS reader timer)

#add_executable(lumPcl ${SOURCES})
add_executable(lumPcl lumPcl)
target_link_libraries(lumPcl ${USER_LIBS} ${CORE_LIBS})

add_executable(lumTdtk lumTdtk)
target_link_libraries(lumTdtk ${USER_LIBS} ${CORE_LIBS})

add_executable(pcTranslate pcTranslate)
target_link_libraries(pcTranslate ${USER_LIBS} ${CORE_LIBS})

#-------------------------------------------------------------------------------
# Directories.
#-------------------------------------------------------------------------------
add_subdirectory(reader)
add_subdirectory(timer)
################################################################################
