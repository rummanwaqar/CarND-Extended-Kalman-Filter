project(ExtendedKF)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

include_directories(include/)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

file(GLOB sources "src/*.cpp")
add_executable(ExtendedKF ${sources})
target_link_libraries(ExtendedKF z ssl uv uWS)
