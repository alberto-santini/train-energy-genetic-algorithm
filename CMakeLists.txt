cmake_minimum_required(VERSION 3.5)
project(tega)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -Wall -Werror")
set(CMAKE_C_FLAGS_DEBUG  "${CMAKE_C_FLAGS_DEBUG} -Og -ggdb")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")

find_library(MATH m)
find_library(JANSSON jansson)

set(SOURCE_FILES src/main.c src/segment.h src/train.h src/davis.h src/davis.c src/instance.h src/instance.c src/train.c src/segment.c src/lookup.h src/lookup.c src/eps.h src/segment_evaluation.h src/segment_evaluation.c)
add_executable(tega ${SOURCE_FILES})

target_link_libraries(tega ${MATH})
target_link_libraries(tega ${JANSSON})