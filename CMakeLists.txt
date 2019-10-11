cmake_minimum_required(VERSION 3.1)
project(MR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_subdirectory(Cleaner)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"                   ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"                     ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"            ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use OpenGL viewer with ImGui" ON)


find_package(LIBIGL REQUIRED QUIET)
find_package(QHULL REQUIRED QUIET)

# Add your project files
file( GLOB_RECURSE SRCFILES src/*.cpp src/*.h src/*.c )
file( GLOB RPLY_FILES include/rply/*.c )
add_compile_definitions(ASSET_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/")
add_executable( ${PROJECT_NAME}_bin ${SRCFILES} ${RPLY_FILES} )
target_link_libraries(${PROJECT_NAME}_bin       PRIVATE igl::core igl::opengl_glfw igl::opengl_glfw_imgui ${qhull_LIBRARIES})
target_include_directories(${PROJECT_NAME}_bin  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${qhull_INCLUDE_DIRS})
if(MSVC)
# enable updated __cplusplus macro value
if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 19.14)
    target_compile_options(${PROJECT_NAME}_bin PRIVATE "/Zc:__cplusplus") # available since VS 2017 15.7
endif()
endif()
