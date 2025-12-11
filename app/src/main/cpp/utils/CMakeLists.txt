
add_library(illixr_hmd OBJECT hmd.cpp
        hmd.hpp
)

target_include_directories(illixr_hmd PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_library(egl_context_manager OBJECT
        ${CMAKE_SOURCE_DIR}/include/illixr/egl/egl_context_manager.hpp
        egl_context_manager.cpp
)

target_include_directories(egl_context_manager PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(egl_context_manager PUBLIC
        spdlog::spdlog_android
)