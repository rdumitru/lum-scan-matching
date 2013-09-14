################################################################################
#-------------------------------------------------------------------------------
# Dependencies.
#-------------------------------------------------------------------------------
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)

#-------------------------------------------------------------------------------
# Source files.
#------------------------------------------------------------------------------
file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")

#-------------------------------------------------------------------------------
# Executables.
#-------------------------------------------------------------------------------
set(CORE_LIBS ${OpenGL_LIBRARY} ${GLUT_LIBRARY} ${GLU_LIBRARY} ${GLEW_LIBRARY})
set(USER_LIBS testLib)

add_executable(run ${SOURCES})
target_link_libraries(run ${USER_LIBS} ${CORE_LIBS})

#-------------------------------------------------------------------------------
# Directories.
#-------------------------------------------------------------------------------
add_subdirectory(testLib)
################################################################################
