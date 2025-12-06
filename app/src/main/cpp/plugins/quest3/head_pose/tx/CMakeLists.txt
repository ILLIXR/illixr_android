
include(../protoc_generate_cpp.cmake)

set(PLUGIN_NAME plugin.quest3.head_pose.tx${ILLIXR_BUILD_SUFFIX})

add_library(${PLUGIN_NAME} SHARED
        plugin.cpp
        plugin.hpp
)

set(protobuf_files
        ${CMAKE_CURRENT_SOURCE_DIR}/../proto/quest_pose.proto
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${ILLIXR_SOURCE_DIR}/include
        ${CMAKE_BINARY_DIR}/plugins/quest3/pose/device_tx/protobuf
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue
)

PROTOBUF_TARGET_CPP(${PLUGIN_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/../proto ${protobuf_files})

target_link_libraries(${PLUGIN_NAME} PUBLIC
        Eigen3::Eigen
        Boost::serialization
        protobuf::libprotobuf_android
        spdlog::spdlog_android
)
target_compile_features(${PLUGIN_NAME} PRIVATE cxx_std_17)

install(TARGETS ${PLUGIN_NAME} DESTINATION lib)
