set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(../protoc_generate_cpp.cmake)

set(PLUGIN_NAME plugin.offload_rendering_mediandk.rx${ILLIXR_BUILD_SUFFIX})

add_library(${PLUGIN_NAME} SHARED
        plugin.hpp
        plugin.cpp
        frame_decoder.cpp
        frame_decoder.hpp
        stereo_surface_decoder.cpp
        stereo_surface_decoder.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/quest3_params.hpp
        $<TARGET_OBJECTS:egl_context_manager>
)

set(protobuf_files
        ${CMAKE_CURRENT_SOURCE_DIR}/../proto/rendered_frame.proto
)

target_include_directories(${PLUGIN_NAME} PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${ANDROID_NDK}/sources/android/native_app_glue
        ${CMAKE_BINARY_DIR}/plugins/offload_rendering_mediandk/rx/protobuf
)

PROTOBUF_TARGET_CPP(${PLUGIN_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/../proto ${protobuf_files})

target_link_libraries(${PLUGIN_NAME} PUBLIC
        Boost::serialization
        Eigen3::Eigen
        EGL
        GLESv3
        protobuf::libprotobuf_android
        spdlog::spdlog_android
        mediandk
        android
)

install(TARGETS ${PLUGIN_NAME} DESTINATION lib)
