set(OPENVINS_CMAKE_ARGS "")

externalproject_add(OpenVINS
        GIT_REPOSITORY https://github.com/ILLIXR/open_vins.git   # Git repo for source code
        GIT_TAG e58ea03fde9330ffc869c2082e316655fa169ec9         # sha5 hash for specific commit to pull (if there is no specific tag to use)
        PREFIX ${CMAKE_BINARY_DIR}/_deps/OpenVINS                # the build directory
        #arguments to pass to CMake
        CMAKE_ARGS -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=${CMAKE_LIBRARY_OUTPUT_DIRECTORY} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DILLIXR_ROOT=${PROJECT_SOURCE_DIR} -DILLIXR_BUILD_SUFFIX=${ILLIXR_BUILD_SUFFIX} -DILLIXR_INTEGRATION=ON -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE} -DANDROID_ABI=${ANDROID_ABI} -DANDROID_PLATFORM=${ANDROID_PLATFORM} -DOpenCV_DIR=${OpenCV_DIR} -DBoost_DIR=${Boost_DIR} -Dspdlog_android_DIR=${spdlog_android_DIR} -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} -DEigen3_DIR=${Eigen3_DIR} -DANDROID_BUILD=ON
        BUILD_BYPRODUCTS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libplugin.openvins${ILLIXR_BUILD_SUFFIX}.so
)

add_library(OpenVINS_plugin SHARED IMPORTED GLOBAL)
set_property(TARGET OpenVINS_plugin PROPERTY IMPORTED_LOCATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libplugin.openvins${ILLIXR_BUILD_SUFFIX}.so)
add_dependencies(OpenVINS_plugin OpenVINS)
