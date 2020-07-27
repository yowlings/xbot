cmake_minimum_required(VERSION 3.1)
project(baidu_asr_demo)

add_compile_options(-std=c++11)
set(CMAKE_C_STANDARD 99)

find_package(Threads REQUIRED)
find_package(curl COMPONENTS)

set(SOURCE ../../src/library/asr_online.cpp   ../../src/library/linuxrec.cpp ../../src/library/file_operation.cpp   ../../src/library/common_config.cpp)

include_directories(../../include )
link_directories(/usr/local/lib ../../lib )

add_executable(baidu_asr_demo baidu_asr_example.cpp ${SOURCE})
target_link_libraries(baidu_asr_demo msc asound pthread gflags curl)
