cmake_minimum_required(VERSION 2.8)
project( main )
find_package( OpenCV REQUIRED )

option(BUILD_ON_PI 	"Option description" OFF)
option(BUILD_ON_HOST 	"Option description" ON)

include(svm_sign_detect/dlib/cmake)

set(CMAKE_BUILD_TYPE Release)
set(DLIB_NO_GUI_SUPPORT OFF)

MACRO(add_source name)
    ADD_EXECUTABLE(${name} svm_sign_detect/src/${name}.cpp)
    TARGET_LINK_LIBRARIES(${name} dlib )
    TARGET_LINK_LIBRARIES(${name} ${OpenCV_LIBS} )
ENDMACRO()

# add_source(my_detect)

SET(GCC_COVERAGE_COMPILE_FLAGS "-g -pthread")
SET(GCC_COVERAGE_LINK_FLAGS    "-g -pthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")


if(BUILD_ON_PI)
	add_library(DetectLane src/pi_lane_detect.cpp)
	add_library(Control src/control.cpp)
	add_library(utility src/utility.cpp)
	add_library(raspi_uart src/raspi_uart.cpp)

	install(TARGETS DetectLane Control raspi_uart utility DESTINATION lib)
	install(FILES inc/pi_lane_detect.h inc/control.h inc/raspi_uart.h inc/utility.h DESTINATION include)

	add_library(SignDetect svm_sign_detect/src/sign_detect.cpp)
	install(TARGETS SignDetect DESTINATION lib)
	install(FILES svm_sign_detect/src/sign_detect.h DESTINATION include)
	target_link_libraries(SignDetect dlib ${OpenCV_LIBS})

	message(STATUS "BUILD ON PI")

	SET(GCC_COVERAGE_COMPILE_FLAGS "-lwiringPi")
	SET(GCC_COVERAGE_LINK_FLAGS    "-lwiringPi")
	SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
	add_executable( main src/main.cpp )
	target_link_libraries( main DetectLane ${OpenCV_LIBS} dlib raspi_uart Control utility SignDetect)
endif(BUILD_ON_PI)


if(BUILD_ON_HOST)
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	
	SET(SOURCE_FILES src/main.cpp src/pi_lane_detect.cpp src/control.cpp src/utility.cpp src/jetson_uart.cpp)
	# add_library(DetectLane src/pi_lane_detect.cpp)
	# add_library(Control src/control.cpp)
	# add_library(utility src/utility.cpp)
	# install(TARGETS DetectLane Control utility DESTINATION lib)
	# install(FILES inc/pi_lane_detect.h inc/control.h inc/utility.h DESTINATION include)

	add_library(SignDetect svm_sign_detect/src/sign_detect.cpp)
	# install(TARGETS SignDetect DESTINATION lib)
	# install(FILES svm_sign_detect/src/sign_detect.h DESTINATION include)
	target_link_libraries(SignDetect dlib ${OpenCV_LIBS})

	message(STATUS "BUILD ON HOST")
	add_executable( main ${SOURCE_FILES})
	target_link_libraries( main ${OpenCV_LIBS} SignDetect)
endif(BUILD_ON_HOST)





