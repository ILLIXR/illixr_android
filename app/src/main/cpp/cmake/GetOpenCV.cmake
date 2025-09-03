find_package(OpenCV 4.5.5 QUIET CONFIG COMPONENTS opencv_java)

if(NOT OpenCV_FOUND)
    set(HOME_DIR "$ENV{HOME}")
    #file(DOWNLOAD https://github.com/opencv/opencv/archive/4.5.5.zip ${CMAKE_BINARY_DIR}/_deps/opencv-4.5.5.zip)
    #file(ARCHIVE_EXTRACT INPUT ${CMAKE_BINARY_DIR}/_deps/opencv-4.5.5.zip DESTINATION ${CMAKE_BINARY_DIR}/_deps)
    file(DOWNLOAD https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.5.zip ${CMAKE_BINARY_DIR}/_deps/opencv_contrib-4.5.5.zip)
    file(ARCHIVE_EXTRACT INPUT ${CMAKE_BINARY_DIR}/_deps/opencv_contrib-4.5.5.zip DESTINATION ${CMAKE_BINARY_DIR}/_deps)
    externalproject_add(OpenCV_Android
            URL https://github.com/opencv/opencv/archive/4.5.5.zip
            PREFIX ${CMAKE_BINARY_DIR}/_deps/opencv-4.5.5
            CMAKE_ARGS -DANDROID_ABI=${ANDROID_ABI} -DBUILD_DOCS=OFF -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_DIR} -DCMAKE_TOOLCHAIN_FILE=${HOME_DIR}/Android/Sdk/ndk/27.2.12479018/build/cmake/android.toolchain.cmake -DENABLE_PIC=ON -DOPENCV_EXTRA_MODULES_PATH=${CMAKE_BINARY_DIR}/_deps/opencv_contrib-4.5.5/modules -DANDROID_SDK_TOOLS=${HOME_DIR}/Android/Sdk/build-tools/33.0.0 -DANDROID_SDK_BUILD_TOOLS_VERSION=33.0.0
    )
    set(OpenCV_INCLUDE_DIRS ${THIRD_PARTY_DIR}/sdk/native/jni/include)
    set(OpenCV_ANDROID_NATIVE_API_LEVEL "21")
    set(OpenCV_SHARED OFF)

    set(OpenCV_LIB_COMPONENTS opencv_core;opencv_flann;opencv_imgproc;opencv_intensity_transform;opencv_ml;opencv_phase_unwrapping;opencv_photo;opencv_plot;opencv_quality;opencv_reg;opencv_surface_matching;opencv_xphoto;opencv_dnn;opencv_dnn_superres;opencv_features2d;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_imgcodecs;opencv_line_descriptor;opencv_saliency;opencv_text;opencv_videoio;opencv_wechat_qrcode;opencv_barcode;opencv_calib3d;opencv_datasets;opencv_highgui;opencv_mcc;opencv_objdetect;opencv_rapid;opencv_rgbd;opencv_shape;opencv_structured_light;opencv_video;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dnn_objdetect;opencv_dpm;opencv_face;opencv_gapi;opencv_optflow;opencv_stitching;opencv_superres;opencv_tracking;opencv_stereo)

    set(_THIRD_PARTY_COMPONENTS cpufeatures;libjpeg-turbo;libtiff;libwebp;libopenjp2;libpng;IlmImf;libprotobuf;quirc;tegra_hal;ittnotify;ade)

    foreach(_lib IN LISTS _THIRD_PARTY_COMPONENTS)
        add_library(${_lib} STATIC IMPORTED)
        set_property(TARGET ${_lib} APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
        set_target_properties(${_lib} PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
                IMPORTED_LOCATION_RELEASE "${THIRD_PARTY_DIR}/sdk/native/3rdparty/libs/arm64-v8a/lib${_lib}.a"
        )
    endforeach ()
    unset(_THIRD_PARTY_COMPONENTS)

    # set properties for specific libraries
    set_target_properties(libtiff PROPERTIES
            INTERFACE_LINK_LIBRARIES "z"
            IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C;CXX"
    )
    set_target_properties(libwebp PROPERTIES
            INTERFACE_LINK_LIBRARIES "cpufeatures"
    )
    set_target_properties(libopenjp2 PROPERTIES
            INTERFACE_COMPILE_DEFINITIONS "OPJ_STATIC"
            INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:m>;\$<LINK_ONLY:-pthread>"
    )
    set_target_properties(libpng PROPERTIES
            INTERFACE_LINK_LIBRARIES "z"
    )
    set_target_properties(IlmImf PROPERTIES
            INTERFACE_LINK_LIBRARIES "z"
            IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
    )
    set_target_properties(libprotobuf PROPERTIES
            INTERFACE_LINK_LIBRARIES "-landroid;-llog"
            IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
    )
    set_target_properties(tegra_hal PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
    )
    set_target_properties(ittnotify PROPERTIES
            INTERFACE_LINK_LIBRARIES "dl"
    )
    set_target_properties(ade PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
    )

    # OpenCV libraries

    # Create imported target ocv.3rdparty.android_mediandk
    add_library(ocv.3rdparty.android_mediandk INTERFACE IMPORTED)
    set_target_properties(ocv.3rdparty.android_mediandk PROPERTIES
            INTERFACE_COMPILE_DEFINITIONS "HAVE_ANDROID_MEDIANDK"
            INTERFACE_LINK_LIBRARIES "-landroid -llog -lmediandk"
    )



    foreach(_lib IN LISTS OpenCV_LIB_COMPONENTS)
        add_library(${_lib} STATIC IMPORTED)
        set_property(TARGET ${_lib} APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
        set_target_properties(${_lib} PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
                IMPORTED_LOCATION_RELEASE "${THIRD_PARTY_DIR}/sdk/native/staticlibs/arm64-v8a/lib${_lib}.a"
        )
    endforeach()
    set_target_properties(opencv_core PROPERTIES
            INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>;\$<LINK_ONLY:z>;\$<LINK_ONLY:cpufeatures>;\$<LINK_ONLY:ittnotify>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_flann PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_core;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_imgproc PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_core;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_intensity_transform PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_ml PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_core;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_phase_unwrapping PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_photo PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_plot PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_quality PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_ml;opencv_core;opencv_imgproc;opencv_ml;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_reg PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_surface_matching PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_core;opencv_flann;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_xphoto PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_photo;opencv_core;opencv_imgproc;opencv_photo;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_dnn PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>;\$<LINK_ONLY:libprotobuf>"
    )
    set_target_properties(opencv_dnn_superres PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_ml;opencv_quality;opencv_dnn;opencv_core;opencv_imgproc;opencv_ml;opencv_quality;opencv_dnn;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_features2d PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_core;opencv_flann;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_fuzzy PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_hfs PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_img_hash PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_imgcodecs PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>;\$<LINK_ONLY:libjpeg-turbo>;\$<LINK_ONLY:libwebp>;\$<LINK_ONLY:libpng>;\$<LINK_ONLY:libtiff>;\$<LINK_ONLY:libopenjp2>;\$<LINK_ONLY:IlmImf>;\$<LINK_ONLY:z>"
    )
    set_target_properties(opencv_line_descriptor PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_saliency PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_text PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_dnn;opencv_features2d;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_dnn;opencv_features2d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_videoio PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_core;opencv_imgproc;opencv_imgcodecs;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>;\$<LINK_ONLY:ocv.3rdparty.android_mediandk>"
    )
    set_target_properties(opencv_wechat_qrcode PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_dnn;opencv_core;opencv_imgproc;opencv_dnn;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_barcode PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_dnn;opencv_core;opencv_imgproc;opencv_dnn;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_calib3d PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_datasets PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_highgui PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_mcc PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_objdetect PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>;\$<LINK_ONLY:quirc>"
    )
    set_target_properties(opencv_rapid PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_rgbd PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_shape PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_structured_light PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_phase_unwrapping;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_phase_unwrapping;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_video PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_videostab PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_photo;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_video;opencv_core;opencv_flann;opencv_imgproc;opencv_photo;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_video;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_xfeatures2d PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_features2d;opencv_calib3d;opencv_shape;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_features2d;opencv_calib3d;opencv_shape;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_ximgproc PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_video;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_video;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_xobjdetect PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_objdetect;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_objdetect;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_aruco PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_bgsegm PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_video;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_video;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_bioinspired PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_ccalib PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_highgui;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_highgui;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_dnn_objdetect PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_dnn;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_core;opencv_imgproc;opencv_dnn;opencv_imgcodecs;opencv_videoio;opencv_highgui;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_dpm PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_highgui;opencv_objdetect;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_highgui;opencv_objdetect;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_face PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_photo;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_objdetect;opencv_core;opencv_flann;opencv_imgproc;opencv_photo;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_objdetect;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_gapi PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_video;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_video;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>;\$<LINK_ONLY:ade>;\$<LINK_ONLY:ittnotify>"
    )
    set_target_properties(opencv_optflow PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_video;opencv_ximgproc;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_video;opencv_ximgproc;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_stitching PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_features2d;opencv_calib3d;opencv_shape;opencv_xfeatures2d;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_features2d;opencv_calib3d;opencv_shape;opencv_xfeatures2d;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_superres PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_video;opencv_ximgproc;opencv_optflow;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_video;opencv_ximgproc;opencv_optflow;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_tracking PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_plot;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text;opencv_videoio;opencv_calib3d;opencv_datasets;opencv_highgui;opencv_video;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_plot;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text;opencv_videoio;opencv_calib3d;opencv_datasets;opencv_highgui;opencv_video;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )
    set_target_properties(opencv_stereo PROPERTIES
            INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_plot;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text;opencv_videoio;opencv_calib3d;opencv_datasets;opencv_highgui;opencv_video;opencv_tracking;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_plot;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text;opencv_videoio;opencv_calib3d;opencv_datasets;opencv_highgui;opencv_video;opencv_tracking;\$<LINK_ONLY:dl>;\$<LINK_ONLY:m>;\$<LINK_ONLY:log>;\$<LINK_ONLY:tegra_hal>"
    )


    # Create imported target opencv_java
    add_library(opencv_java SHARED IMPORTED)
    set_target_properties(opencv_java PROPERTIES
            INTERFACE_LINK_LIBRARIES "jnigraphics;jnigraphics;log;dl;z;log;dl;z"
    )
    set_property(TARGET opencv_java APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
    set_target_properties(opencv_java PROPERTIES
            IMPORTED_LOCATION_RELEASE "${THIRD_PARTY_DIR}/sdk/native/libs/arm64-v8a/libopencv_java4.so"
            IMPORTED_SONAME_RELEASE "libopencv_java4.so"
    )

    unset(OpenCV_LIB_COMPONENTS)
    set(BUILDING_OPENCV ON)
endif()
