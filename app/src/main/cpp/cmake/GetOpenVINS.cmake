set(OPENVINS_CMAKE_ARGS "")

EXTERNALPROJECT_ADD(OpenVINS
        GIT_REPOSITORY https://github.com/ILLIXR/open_vins.git   # Git repo for source code
        GIT_TAG 5584dad279c85aa4a526fef35f33370cb3eb106e         # sha5 hash for specific commit to pull (if there is no specific tag to use)
        PREFIX ${CMAKE_BINARY_DIR}/_deps/OpenVINS                # the build directory
        #arguments to pass to CMake
        CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DILLIXR_ROOT=${PROJECT_SOURCE_DIR}/include -DILLIXR_BUILD_SUFFIX=${ILLIXR_BUILD_SUFFIX} -DILLIXR_INTEGRATION=ON -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE} -DOpenCV_DIR=${OpenCV_DIR}, -DBoost_DIR=${BoostDIR} -Dspdlog_android_DIR=${spdlog_android_DIR} ${OPENVINS_CMAKE_ARGS}
)
