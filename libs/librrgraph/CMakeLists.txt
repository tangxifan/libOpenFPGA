cmake_minimum_required(VERSION 2.8.12)

project("librrgraph")

file(GLOB_RECURSE EXEC_SOURCES src/main.cpp)
file(GLOB_RECURSE LIB_SOURCES src/*.cpp)
file(GLOB_RECURSE LIB_HEADERS src/*.h)
files_to_dirs(LIB_HEADERS LIB_INCLUDE_DIRS)

#Remove test executable from library
list(REMOVE_ITEM LIB_SOURCES ${EXEC_SOURCES})

#Create the library
add_library(librrgraph STATIC
             ${LIB_HEADERS}
             ${LIB_SOURCES})
target_include_directories(librrgraph PUBLIC ${LIB_INCLUDE_DIRS})
set_target_properties(librrgraph PROPERTIES PREFIX "") #Avoid extra 'lib' prefix

#Specify link-time dependancies
target_link_libraries(librrgraph
                      libarchfpga
                      libvtrutil)

#Create the test executable
add_executable(test_rrgraph ${EXEC_SOURCES})
target_link_libraries(test_rrgraph librrgraph)

install(TARGETS test_rrgraph librrgraph DESTINATION bin)
