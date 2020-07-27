cmake_minimum_required(VERSION 3.1)
project(awaken_offline_demo)

add_compile_options(-std=c++11)
set(CMAKE_C_STANDARD 99)

find_package(Threads REQUIRED)
set(SOURCE ../../src/library/awaken_offline.cpp   ../../src/library/linuxrec.cpp ../../src/library/file_operation.cpp   ../../src/library/common_config.cpp)

include_directories(../../include )
link_directories(/usr/local/lib ../../lib )

add_executable(awaken_demo awaken_offline_example.cpp ${SOURCE})
target_link_libraries(awaken_demo msc asound pthread)
