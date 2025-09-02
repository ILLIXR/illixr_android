set(PLUGIN_NAME plugin.extended_window${ILLIXR_BUILD_SUFFIX})

add_library(${PLUGIN_NAME} SHARED
        service.cpp
        service.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/phonebook.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/global_module_defs.hpp
        $<TARGET_OBJECTS:illixr_switchboard>
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(${PLUGIN_NAME}
        EGL
        GLESv3
        spdlog::spdlog_android
)

if (BUILDING_SPDLOG)
    add_dependencies(${PLUGIN_NAME} spdlog_android)
endif()
if (BUILDING_OPENCV)
    add_dependencies(${PLUGIN_NAME} OpenCV_Android)
endif()
if (BUILDING_BOOST)
    add_dependencies(${PLUGIN_NAME} Boost_Install)
endif()
