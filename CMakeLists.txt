#Usage => cmake -DCMAKE_BUILD_TYPE=Debug .

cmake_minimum_required(VERSION 3.1.0)
project( Processor-CPP )

set(CMAKE_CXX_STANDARD 14)
set(ROOT ./)
include_directories(${ROOT})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package( OpenCV REQUIRED )


SET(GCC_COVERAGE_COMPILE_FLAGS "")
SET(GCC_COVERAGE_LINK_FLAGS "")

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

add_compile_options(
  "$<$<CONFIG:DEBUG>:-O3;-g3>"
)

#include_directories(/opt/pylon5/include)
#link_directories(/opt/pylon5/lib64)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

file(GLOB Processor_SRC
    "src/*.hpp"
    "src/*.cpp"
)
add_executable( Processor-CPP ${Processor_SRC})
target_link_libraries( Processor-CPP ${OpenCV_LIBS} Threads::Threads stdc++fs)


add_custom_target(run
    COMMAND bin/Processor-CPP
    DEPENDS Processor-CPP
)

