reachpower(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(reachpower)

add_executable(reachpower
    reachpower.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(reachpower
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(reachpower PRIVATE -Wall -Wextra)
else()
    add_compile_options(reachpower PRIVATE -W2)
endif()
