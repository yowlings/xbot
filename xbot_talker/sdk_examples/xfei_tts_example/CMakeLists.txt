cmake_minimum_required(VERSION 3.1)
project(tts_demo)

add_compile_options(-std=c++11)
set(CMAKE_C_STANDARD 99)

find_package(Threads REQUIRED)
set(SOURCE ../../src/library/linuxrec.cpp ../../src/library/file_operation.cpp   ../../src/library/common_config.cpp ../../src/library/text_to_speech.cpp)

include_directories(../../include )
link_directories(/usr/local/lib ../../lib )

add_executable(tts_demo tts_example.cpp ${SOURCE})
target_link_libraries(tts_demo msc asound pthread boost_system)
