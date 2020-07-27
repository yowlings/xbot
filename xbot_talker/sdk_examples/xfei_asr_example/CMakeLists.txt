cmake_minimum_required(VERSION 3.1)
project(asr_demo)

add_compile_options(-std=c++11)
set(CMAKE_C_STANDARD 99)

find_package(Threads REQUIRED)
set(SOURCE ../../src/library/xfei_speech_recog.cpp   ../../src/library/linuxrec.cpp ../../src/library/file_operation.cpp   ../../src/library/common_config.cpp ../../src/library/build_xunfei_grammar.cpp)

include_directories(../../include )
link_directories(/usr/local/lib ../../lib )

add_executable(asr_demo asr_example.cpp ${SOURCE})
target_link_libraries(asr_demo msc asound pthread boost_system)
