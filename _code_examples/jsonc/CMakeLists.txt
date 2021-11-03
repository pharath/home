cmake_minimum_required(VERSION 2.8)
project(json-parse04)

ADD_EXECUTABLE(json-parse04 json-parse04.c)
target_link_libraries(json-parse04 -ljson-c)
