set(CMAKE_CXX_STANDARD 17)

set(OCULUS_OPENXR_SDK "$ENV{OCULUS_OPENXR_MOBILE_SDK}")
if(NOT OCULUS_OPENXR_SDK)
    set(OCULUS_OPENXR_SDK "${CMAKE_SOURCE_DIR}/ovr_openxr_mobile_sdk")
endif()

find_package(OpenXR REQUIRED)

#add_library(openxr_loader SHARED IMPORTED)
#set_target_properties(openxr_loader PROPERTIES
#        IMPORTED_LOCATION "${THIRD_PARTY_DIR}/lib/${ANDROID_ABI}/libopenxr_loader.so"
#)
set(PLUGIN_NAME plugin.openxr_interface${ILLIXR_BUILD_SUFFIX})

add_library(${PLUGIN_NAME} SHARED
        plugin.cpp
        plugin.hpp
        stereo_renderer.cpp
        stereo_renderer.hpp
        #${CMAKE_SOURCE_DIR}/include/illixr/openxr_interface.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/phonebook.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${ANDROID_TOOLS_INCLUDE}
        ${ILLIXR_SOURCE_DIR}/include
        ${ANDROID_NDK}/sources/android/native_app_glue
        "${OCULUS_OPENXR_SDK}/OpenXR/Include"
)
target_compile_definitions(${PLUGIN_NAME} PRIVATE
        XR_USE_GRAPHICS_API_OPENGL_ES
        XR_USE_PLATFORM_ANDROID
)

target_link_libraries(${PLUGIN_NAME} PUBLIC
        Eigen3::Eigen
        android
        spdlog::spdlog_android
        #OpenXR::headers
        OpenXR::openxr_loader
        EGL
        GLESv3
)