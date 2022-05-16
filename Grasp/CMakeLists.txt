FILE(GLOB_RECURSE SOURCES_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
								"${CMAKE_SOURCE_DIR}/Core/Math/src/dxPoint3.cpp")
FILE(GLOB_RECURSE INCLUDE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
								"${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
								"${CMAKE_SOURCE_DIR}/Core/Math/include/dxPoint3.h"
								"${CMAKE_SOURCE_DIR}/Core/Search/include/*.h")

# Posix threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
 
#Generate Library
ADD_LIBRARY(dxGraspLoCoMo ${SOURCES_FILES} ${INCLUDE_FILES} )
target_link_libraries(dxGraspLoCoMo PUBLIC Threads::Threads)
INCLUDE_DIRECTORIES(dxGraspLoCoMo PUBLIC 	${CMAKE_CURRENT_SOURCE_DIR}/include
									${NANO_FLANN_INCLUDE}
									${EIGEN3_INCLUDE}
									${CMAKE_SOURCE_DIR}/Core/Sys/include/
									${CMAKE_SOURCE_DIR}/Core/Math/include/
									${CMAKE_SOURCE_DIR}/Core/Search/include/)

SET_PROPERTY(TARGET dxGraspLoCoMo PROPERTY PROJECT_LABEL "dxGraspLoCoMo")
SET_PROPERTY(TARGET dxGraspLoCoMo PROPERTY FOLDER "Modules/dxGraspLoCoMo")
SOURCE_GROUP("Include Files" FILES ${INCLUDE_FILES})
 
IF (DX_BUILD_TEST)
    FILE(GLOB_RECURSE TEST_SOURCES_FILES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp" )
    FILE(GLOB_RECURSE TEST_INCLUDE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.h")
    ADD_EXECUTABLE(dxTestGraspLoCoMo ${TEST_SOURCES_FILES} ${TEST_INCLUDE_FILES})
	TARGET_LINK_LIBRARIES(dxTestGraspLoCoMo dxGraspLoCoMo)

    SET_PROPERTY(TARGET dxTestGraspLoCoMo PROPERTY PROJECT_LABEL "TestdxGraspLoCoMo")
    SET_PROPERTY(TARGET dxTestGraspLoCoMo PROPERTY FOLDER "Modules/dxGraspLoCoMo/Test")

ENDIF (DX_BUILD_TEST)
