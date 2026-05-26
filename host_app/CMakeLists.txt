cmake_minimum_required(VERSION 3.16)
project(host_app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

find_package(Curses REQUIRED)

add_executable(host_app main.cpp)

target_include_directories(host_app PRIVATE
    ${LIBSERIALPORT_INCLUDE_DIRS}
    ${CURSES_INCLUDE_DIR}
)

target_link_directories(host_app PRIVATE
    ${LIBSERIALPORT_LIBRARY_DIRS}
)

target_link_libraries(host_app PRIVATE
    ${LIBSERIALPORT_LIBRARIES}
    ${CURSES_LIBRARIES}
)

target_compile_options(host_app PRIVATE ${LIBSERIALPORT_CFLAGS_OTHER})
