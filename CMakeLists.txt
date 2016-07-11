project(teoMWI-example)

cmake_minimum_required(VERSION 2.8)
aux_source_directory(./src SRC_LIST)


#teoMWI library
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/teoMWI)
set(teoMWI_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/teoMWI ${teoMWI_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${teoMWI_INCLUDE_DIR})


find_package(YARP REQUIRED)


add_executable(${PROJECT_NAME} ${SRC_LIST})

#link with libs
target_link_libraries( ${PROJECT_NAME} teoMWI ${YARP_LIBRARIES})