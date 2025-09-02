cmake_minimum_required(VERSION 3.22)

project(native_renderer)
add_compile_definitions(ILLIXR_ANDROID_BUILD)

set(PLUGIN_NAME plugin.native_renderer${ILLIXR_BUILD_SUFFIX})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PLUGIN_NAME} SHARED plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/global_module_defs.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose_prediction.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/vk/display_provider.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/vk/render_pass.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/gl_util/lib/tiny_obj_loader.h
        $<TARGET_OBJECTS:illixr_vulkan_utils>
        $<TARGET_OBJECTS:illixr_switchboard>
)

set_target_properties(${PLUGIN_NAME} PROPERTIES OUTPUT_NAME ${PLUGIN_NAME})
if (NOT ANDROID)
    if (Vulkan_EXTERNAL)
        add_dependencies(${PLUGIN_NAME} ${Vulkan_DEP_STR})
    endif ()
endif ()
target_link_libraries(${PLUGIN_NAME} PUBLIC
        spdlog::spdlog_android
        Eigen3::Eigen
        ${VULKAN_LIB}
        android
)
target_include_directories(${PLUGIN_NAME} PUBLIC
        ${Vulkan_INCLUDE_DIRS}
        ${ILLIXR_SOURCE_DIR}/include
        ${ANDROID_NDK}/sources/android/native_app_glue
)

install(TARGETS ${PLUGIN_NAME} DESTINATION lib)

if (BUILDING_SPDLOG)
    add_dependencies(${PLUGIN_NAME} spdlog_android)
endif ()
if (BUILDING_BOOST)
    add_dependencies(${PLUGIN_NAME} Boost_Install)
endif ()
