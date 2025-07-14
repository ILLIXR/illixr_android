find_package(spdlog_android 1.15.3 REQUIRED)
#if(NOT spdlog_android_FOUND)
#    externalproject_add(spdlog_android
#            GIT_REPOSITORY https://github.com/ILLIXR/spdlog.git
#            GIT_TAG 24f11060dd018404017f243ddee7cdd72e7d7a96
#            PREFIX ${CMAKE_BINARY_DIR}/_deps/spdlog_android
#            CMAKE_ARGS -DCMAKE_SYSTEM_NAME=Android -DANDROID_ABI=${ANDROID_ABI} -DANDROID_PLATFORM=${ANDROID_PLATFORM} -DCMAKE_ANDROID_NDK=${CMAKE_ANDROID_NDK} -DSPDLOG_BUILD_SHARED=ON -DSPDLOG_BUILD_PIC=ON -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
#    )
#    set(spdlog_android_LIBRARIES ${CMAKE_SOURCE_DIR}/third_party/lib/libspdlog_android.so PARENT_SCOPE)
#    set(spdlog_android_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/third_party/include PARENT_SCOPE)
#    set(BUILDING_SPDLOG ON PARENT_SCOPE)
#endif()