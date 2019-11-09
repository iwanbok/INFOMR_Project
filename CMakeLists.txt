cmake_minimum_required(VERSION 3.1)
project(MR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"                   ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"                     ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"            ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use OpenGL viewer with ImGui" ON)

find_package(LIBIGL REQUIRED QUIET)
find_package(QHULL REQUIRED QUIET)

# find_package(PythonInterp 3.5 REQUIRED)
# This goes after, since it uses PythonInterp as hint
# find_package(PythonLibs 3.5 REQUIRED)

# exec_program(${PYTHON_EXECUTABLE}
#              ARGS "-c \"import numpy; print(numpy.get_include())\""
#              OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
#              RETURN_VALUE NUMPY_NOT_FOUND
#             )
# string(STRIP ${NUMPY_INCLUDE_DIR} NUMPY_INCLUDE_DIR)
# if(NUMPY_NOT_FOUND)
#     message(FATAL_ERROR "NumPy headers not found")
# endif()

# Add your project files
file( GLOB RPLY_FILES include/rply/*.c )
file( GLOB EMD_FILES src/emd/*.cpp src/emd/*.h)
file( GLOB_RECURSE OPEN3D_FILES src/Open3D/*.h src/Open3D/*.cpp)
file( GLOB ANN_FILES src/ANN/*.h src/ANN/*.cpp include/ANN/*.h)
file( GLOB TSNE_FILES src/tsne/*.h src/tsne/*.cpp)
file( GLOB SRC_FILES src/*.cpp src/*.c)
file( GLOB HDR_FILES src/*.hpp src/*.h)

add_compile_definitions(ASSET_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/" )
add_executable( ${PROJECT_NAME}_bin ${RPLY_FILES} ${EMD_FILES} ${OPEN3D_FILES} ${ANN_FILES} ${TSNE_FILES} ${SRC_FILES} ${HDR_FILES})

target_link_libraries(${PROJECT_NAME}_bin       PRIVATE igl::core igl::opengl_glfw igl::opengl_glfw_imgui ${qhull_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(${PROJECT_NAME}_bin  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${qhull_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
# target_include_directories(${PROJECT_NAME}_bin  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${qhull_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})

source_group("rply"         FILES ${RPLY_FILES})
source_group("emd"          FILES ${EMD_FILES})
source_group("Open3d"       FILES ${OPEN3D_FILES})
source_group("ANN"          FILES ${ANN_FILES})
source_group("tsne"         FILES ${TSNE_FILES})
source_group("Source Files" FILES ${SRC_FILES})
source_group("Header Files" FILES ${HDR_FILES})

if(MSVC)
    # enable updated __cplusplus macro value
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 19.14)
        target_compile_options(${PROJECT_NAME}_bin PRIVATE "/Zc:__cplusplus") # available since VS 2017 15.7
    endif()
    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME}_bin)
endif()
