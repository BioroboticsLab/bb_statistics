
#########################
#Get all the libraries into a variable
#########################
set(LIBS 
    ${Qt5Core_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
		)

set(CMAKE_INCLUDE_CURRENT_DIR OFF)

message("Boost_LIBRARIES is ${Boost_LIBRARIES}")
message("Qt5Core_LIBRARIES is ${Qt5Core_LIBRARIES}")
message("OpenCV_LIBRARIES is ${OpenCV_LIBRARIES}")
message("QT5_INCLUDE_DIRS is ${Qt5Core_INCLUDE_DIRS}")


if("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(INCLUDE_DIRS 
		/usr/local/include 
		${Qt5Core_INCLUDE_DIRS} 
		${OpenCV_INCLUDE_DIRS} )
	set(LIBS -L/usr/local/lib ${LIBS} pthread dl rt )
	
	message("Configuring for linux...")
else()
	set(INCLUDE_DIRS ${Boost_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
	set(LIBS ${LIBS})
	message("Configuring for windows...")
endif()


message("All libs: ${LIBS}")

#########################
#Put everything together
#########################

include_directories(${INCLUDE_DIRS})

message("Configuring Statistics...")
set(EXE_NAME bb_statistics)
file(GLOB_RECURSE StatisticsSrc RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.c *.h *.hpp)


message("Sources are ${StatisticsSrc}")
add_executable(${EXE_NAME} ${StatisticsSrc} )
target_link_libraries(${EXE_NAME} ${LIBS} )





