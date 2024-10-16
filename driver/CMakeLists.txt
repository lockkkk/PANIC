cmake_minimum_required(VERSION 2.8)
project(ixy C)

set(CMAKE_C_STANDARD 11)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -flto")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")

add_compile_options(-g -O3 -march=native -fomit-frame-pointer -std=c11
	-D_XOPEN_SOURCE=700
	-D_DEFAULT_SOURCE
	-Wall
	-Wextra
	-Wno-unused-parameter
	-Wno-unused-function
	-Wformat=2
	-flto
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

# Attempt to find json-c
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONC json-c)
if(JSONC_FOUND)
    include_directories(${JSONC_INCLUDE_DIRS})
    link_directories(${JSONC_LIBRARY_DIRS})
else()
    message(STATUS "json-c not found using pkg-config, attempting manual find...")
    find_library(JSONC_LIBRARY NAMES json-c libjson-c)
    if(JSONC_LIBRARY)
        set(JSONC_LIBRARIES ${JSONC_LIBRARY})
        message(STATUS "Found json-c: ${JSONC_LIBRARIES}")
    else()
        message(FATAL_ERROR "json-c library not found.")
    endif()
endif()

set(SOURCE_COMMON src/pci.c src/memory.c src/msg.c  src/stats.c src/interrupts.c src/driver/device.c src/driver/mqnic.c src/libixy-vfio.c)

ADD_LIBRARY(dynamicixy SHARED 
             ${SOURCE_COMMON} )

add_executable(panic-receive src/app/ixy-receive.c ${SOURCE_COMMON})

target_link_libraries(panic-receive Threads::Threads ${JSONC_LIBRARIES})