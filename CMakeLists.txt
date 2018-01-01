PROJECT(PreTZel)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
INCLUDE(GNUInstallDirs)

SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

FIND_PACKAGE(OpenCV REQUIRED)
LINK_LIBRARIES(${OpenCV_LIBS})

FIND_PACKAGE(trax COMPONENTS core opencv client)

IF (TRAX_FOUND)
LINK_DIRECTORIES(${TRAX_LIBRARY_DIRS})
LINK_LIBRARIES(${TRAX_LIBRARIES})
ADD_DEFINITIONS(-DPRETZEL_TRAX)
ENDIF()

INCLUDE_DIRECTORIES(AFTER ${OpenCV_INCLUDE_DIR} ${TRAX_INCLUDE_DIRS} src/)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")

ADD_LIBRARY(pretzel STATIC
		src/lib/projections.cpp
		src/lib/annotations.cpp
		src/lib/camera.cpp
		src/lib/tracker.cpp
		src/lib/time.cpp
		src/lib/getopt.c
)

ADD_EXECUTABLE(pretzel_converter src/converter.cpp)
TARGET_LINK_LIBRARIES(pretzel_converter pretzel)

ADD_EXECUTABLE(pretzel_player src/player.cpp)
TARGET_LINK_LIBRARIES(pretzel_player pretzel)

ADD_EXECUTABLE(pretzel_annotator src/annotator.cpp)
TARGET_LINK_LIBRARIES(pretzel_annotator pretzel)

IF (TRAX_FOUND)
ADD_EXECUTABLE(pretzel_simulator src/simulator.cpp)
TARGET_LINK_LIBRARIES(pretzel_simulator pretzel)
ENDIF()

ADD_EXECUTABLE(pretzel_exporter src/exporter.cpp )
TARGET_LINK_LIBRARIES(pretzel_exporter pretzel)

SET (BUILD_SEEK FALSE CACHE BOOL "Build with fast seek support for annotator (requires libav)")

IF (BUILD_SEEK)
	FIND_PACKAGE(Libav REQUIRED)
	INCLUDE_DIRECTORIES(${LIBAV_INCLUDE_DIR})
	ADD_LIBRARY(seek STATIC src/seek/video_reader.cpp)
	TARGET_LINK_LIBRARIES(seek ${LIBAV_LIBRARIES})

	TARGET_COMPILE_OPTIONS(pretzel_annotator PUBLIC -DFAST_SEEK)
	TARGET_LINK_LIBRARIES(pretzel_annotator seek)
	INSTALL(TARGETS seek RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
ENDIF (BUILD_SEEK)

INSTALL(TARGETS
	pretzel
	pretzel_converter
	pretzel_player
	pretzel_annotator
	pretzel_simulator
	pretzel_exporter
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

INSTALL(DIRECTORY toolkit/ DESTINATION ${CMAKE_INSTALL_DATADIR}/pretzel/vot)

