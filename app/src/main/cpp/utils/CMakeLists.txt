
add_library(illixr_hmd OBJECT hmd.cpp
        hmd.hpp
)

target_include_directories(illixr_hmd PUBLIC ${CMAKE_SOURCE_DIR}/include)
