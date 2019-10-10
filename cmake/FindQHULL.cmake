message(STATUS "Building QHULL from source")
include_directories("qhull/src")
set(libqhullr_SOURCES
    qhull/src/libqhull_r/global_r.c
    qhull/src/libqhull_r/stat_r.c
    qhull/src/libqhull_r/geom2_r.c
    qhull/src/libqhull_r/poly2_r.c
    qhull/src/libqhull_r/merge_r.c
    qhull/src/libqhull_r/libqhull_r.c
    qhull/src/libqhull_r/geom_r.c
    qhull/src/libqhull_r/poly_r.c
    qhull/src/libqhull_r/qset_r.c
    qhull/src/libqhull_r/mem_r.c
    qhull/src/libqhull_r/random_r.c
    qhull/src/libqhull_r/usermem_r.c
    qhull/src/libqhull_r/userprintf_r.c
    qhull/src/libqhull_r/io_r.c
    qhull/src/libqhull_r/user_r.c
    qhull/src/libqhull_r/rboxlib_r.c
    qhull/src/libqhull_r/userprintf_rbox_r.c
)
add_library(qhullstatic_r STATIC ${libqhullr_SOURCES})
set(libqhullcpp_SOURCES
    qhull/src/libqhullcpp/Coordinates.cpp
    qhull/src/libqhullcpp/PointCoordinates.cpp
    qhull/src/libqhullcpp/Qhull.cpp
    qhull/src/libqhullcpp/QhullFacet.cpp
    qhull/src/libqhullcpp/QhullFacetList.cpp
    qhull/src/libqhullcpp/QhullFacetSet.cpp
    qhull/src/libqhullcpp/QhullHyperplane.cpp
    qhull/src/libqhullcpp/QhullPoint.cpp
    qhull/src/libqhullcpp/QhullPointSet.cpp
    qhull/src/libqhullcpp/QhullPoints.cpp
    qhull/src/libqhullcpp/QhullQh.cpp
    qhull/src/libqhullcpp/QhullRidge.cpp
    qhull/src/libqhullcpp/QhullSet.cpp
    qhull/src/libqhullcpp/QhullStat.cpp
    qhull/src/libqhullcpp/QhullVertex.cpp
    qhull/src/libqhullcpp/QhullVertexSet.cpp
    qhull/src/libqhullcpp/RboxPoints.cpp
    qhull/src/libqhullcpp/RoadError.cpp
    qhull/src/libqhullcpp/RoadLogEvent.cpp
)
add_library(qhullcpp STATIC ${libqhullcpp_SOURCES})
if (NOT BUILD_SHARED_LIBS)
    install(TARGETS qhullstatic_r
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
    install(TARGETS qhullcpp
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()
set(qhull_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/qhull/src")
set(qhull_LIBRARIES qhullcpp qhullstatic_r)