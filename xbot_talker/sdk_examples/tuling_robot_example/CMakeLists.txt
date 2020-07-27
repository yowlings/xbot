cmake_minimum_required(VERSION 3.1)
project(tuling_robot_demo)

add_compile_options(-std=c++11)
set(CMAKE_C_STANDARD 99)

find_package(Threads REQUIRED)
set(SOURCE ../../src/library/file_operation.cpp  ../../src/library/linuxrec.cpp ../../src/library/common_config.cpp ../../src/library/nlp_feedback.cpp ../../src/library/text_to_speech.cpp)

include_directories(../../include )
link_directories(/usr/local/lib ../../lib )

add_executable(tuling_robot_demo tuling_robot_example.cpp ${SOURCE})
target_link_libraries(tuling_robot_demo asound msc curl pthread boost_system)
