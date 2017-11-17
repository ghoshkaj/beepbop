cmake_minimum_required(VERSION 2.8.11)
project(beepbop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0 -Wall -Wextra -pedantic -std=c++11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MASON_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/.mason/mason)
include(${CMAKE_CURRENT_SOURCE_DIR}/.mason/mason.cmake)

mason_use(libosmium VERSION 2.13.1 HEADER_ONLY)
include_directories(SYSTEM ${MASON_PACKAGE_libosmium_INCLUDE_DIRS})

mason_use(utfcpp VERSION 2.3.4 HEADER_ONLY)
include_directories(SYSTEM ${MASON_PACKAGE_utfcpp_INCLUDE_DIRS})

mason_use(protozero VERSION 1.5.2 HEADER_ONLY)
include_directories(SYSTEM ${MASON_PACKAGE_protozero_INCLUDE_DIRS})

mason_use(bzip2 VERSION 1.0.6)
include_directories(SYSTEM ${MASON_PACKAGE_bzip2_INCLUDE_DIRS})
set(BZIP2_LIBRARIES ${MASON_PACKAGE_bzip2_STATIC_LIBS})

mason_use(zlib VERSION 1.2.8)
include_directories(SYSTEM ${MASON_PACKAGE_zlib_INCLUDE_DIRS})
set(ZLIB_LIBRARY ${MASON_PACKAGE_zlib_STATIC_LIBS})

mason_use(expat VERSION 2.2.0)
include_directories(SYSTEM ${MASON_PACKAGE_expat_INCLUDE_DIRS})
set(EXPAT_LIBRARIES ${MASON_PACKAGE_expat_STATIC_LIBS})

mason_use(boost VERSION 1.63.0 HEADER_ONLY)
include_directories(SYSTEM ${MASON_PACKAGE_boost_INCLUDE_DIRS})
set(Boost_USE_STATIC_LIBS ON)
set(Boost_GRAPH_LIBRARY ${MASON_PACKAGE_boost_graph_STATIC_LIBS})
mason_use(boost_libprogram_options VERSION 1.63.0)

set(ALL_LIBRARIES ${ZLIB_LIBRARY} ${BZIP2_LIBRARIES} ${EXPAT_LIBRARIES} ${Boost_GRAPH_LIBRARY})

add_executable(build_beepbop build_beepbop.cpp)

target_link_libraries(build_beepbop ${ALL_LIBRARIES})
