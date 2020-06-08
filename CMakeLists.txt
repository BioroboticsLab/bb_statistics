cmake_minimum_required(VERSION 3.15)

project(bb_statistics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

find_package(Qt5 COMPONENTS Core REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.65 COMPONENTS system filesystem program_options QUIET)
if(NOT Boost_FOUND)
    set(Boost_USE_STATIC_LIBS OFF)
    find_package(Boost 1.65 COMPONENTS system filesystem program_options
                                       REQUIRED QUIET)
endif()

find_package(OpenCV REQUIRED)

add_executable(
    ${PROJECT_NAME}
    src/Settings/ParamNames.h
    src/Settings/Settings.h
    src/Settings/Settings.cpp
    src/Settings/StringTranslator.h
    src/Settings/StringTranslator.cpp
    src/Settings/stringTools.h
    src/Settings/stringTools.cpp
    src/Settings/utility.h
    src/Settings/utility.cpp
    src/statistics.h
    src/statistics.cpp
    src/main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE ${CMAKE_DL_LIBS}
            Threads::Threads
            Qt5::Core
            Boost::headers
            Boost::system
            Boost::filesystem
            Boost::program_options
            Boost::disable_autolinking
            ${OpenCV_LIBRARIES})
