# ILLIXR ANDROID

## Setup Instructions

### Disclaimer

This repository still needs reviews from ILLIXR developers. It is only meant to be used for reviewing code for Android. Currently, it only contains code to run ILLIXR with GlDemo. The instructions are only tested on Ubuntu 20.

### Android Studio Setup

1. Download Android Studio from [here](https://developer.android.com/studio?gclid=CjwKCAjwpayjBhAnEiwA-7enaxvq-eEdrW0vwql6iRJ5Q21RVU6-WT9VdIHysKJN9A1r1jI4VXdFQBoCqoAQAvD_BwE&gclsrc=aw.ds).

2. Extract the zip file and navigate to 

    `<extracted-folder>/android-studio/bin`
    
3. Launch Android Studio with the following and accept licences.

    `./studio.sh`

4. Now clone this repository and open it in Android Studio

    `git clone https://github.com/ILLIXR/illixr_android.git`

5. Wait for the Gradle Sync to finish. Required NDK version will be downloaded automatically as part of the sync process. Verify that NDK is downloaded by navigating to

    `~/Android/Sdk/ndk`

    It should have NDK version 21.4 installed.

### Setup OpenCV

1. Download compiled OpenCV libs for arm64 from [here](https://drive.google.com/file/d/1num-InuN1ti2WoRF7fFrb8d-XviG8zBe/view?usp=sharing). For other architectures build them following this [guide](https://mdeore.medium.com/latest-android-ndk-to-build-opencv-ccecd11efa82). Make sure that you build the required modules (see CMakeLists).

2. Change Line 21 and 22 in `app/src/main/cpp/CMakeLists.txt` to the path to the extracted folder.

[//]: # (3. Download OpenCV 4.5.5 from [here]&#40;https://opencv.org/releases/&#41;. Click on the Android option. Download and extract the zip file.)

[//]: # ()
[//]: # (4. Change Line 22 in `app/src/main/cpp/CMakeLists.txt` with the OpenCV folder path.)

### Setup Boost

1. Follow instructions [here](https://github.com/dec1/Boost-for-Android) to download and build boost for Android. Make sure you build it for the right architecture and instruction set you need. For e.g. Android Emulator may require x86 whereas most Android phones will need arm64.

2. Change Line 25 in `app/src/main/cpp/CMakeLists.txt` with the correct folder name.

### Setup Eigen

1. Download Latest release of Eigen from [here](https://eigen.tuxfamily.org/index.php?title=Main_Page).

2. Change Line 28 in `app/src/main/cpp/CMakeLists.txt` with the installed Eigen directory path.

### Install OpenVINS

1. `git clone https://github.com/ILLIXR/open_vins.git`

2. `git switch android_open_vins`

3. Copy OpenVINS to the `app/src/main/cpp` folder.

### Install Application Data

1. Now we have all the dependencies ready to build the project. Hit the build button in Android Studio to build the project. If there are errors make sure all the paths set in the previous steps are correct.

2. We need to manually install data required for GlDemo and Pose_lookup. However we can ony do it when the application is installed on the phone. So, hit the run button and install the application on the phone. The app won't run because we haven't added the data yet.

3. Now while the phone is connected open Device File Explorer located on the right hand side menu of Android Studio. Select the device name. Navigate to `/sdcard/Android/data/com.example.native_activity`. Right click and select Upload, now navigate to the folder `illixr_android/app/src/main/cpp/` and select mav0. Do the same for demo_data.
mav0 contains the groun truth pose form EuRoC dataset required for Pose_lookup and demo_data is used by gldemo.

4. It is possible that the app is not installed in sdcard but only present in the internal memory which is `/data/data/com.example.native_activity`. In this case install the mav0 and demo_data inside `/data/data/com.example.native_activity`.

5. Based on where the data is installed, change the application path in `illixr_android/app/src/main/cpp/main.cpp` line 23.

### Provide App permissions

1. The application needs permission to read and write files. Go to setting in the Android phone, search for native-activity. In the application details go to the permissions section. It will show all the permission that are denied to the application. Click on them to allow the app these permissions. This step may look different based on the specific phone model.

2. If you are using the Android IMU CAM plugin, you will also need to enable the Camera permission for the application to work.

### App Configuration

1. The user can select which plugins they want to run. To change the list of plugins edit "arguments" in `cpp/main.cpp`. Make sure you have built the plugin and provided the correct shared object name. If the .so is not found it will lead to a runtime error.

2. Common configurations:

   a. GlDemo with Pose Lookup: `std::vector<std::string> arguments = { "", "libpose_lookup.so", "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};`

   b. GlDemo with OpenVINS and EuROC dataset: `std::vector<std::string> arguments = { "", "libslam.so", "liboffline_imu_cam.so", "librk4_integrator.so", "libpose_prediction.so", "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};`

   c. GlDemo with OpenVINS and Android Cam/IMU: `std::vector<std::string> arguments = { "", "libslam.so", "libandroid_imu_cam.so", "librk4_integrator.so", "libpose_prediction.so", "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};`

3. OpenVINS params:

   OpenVINS needs the intrinsics of the Camera/IMU. These parameters will differ based on if you are using a dataset like EuROC or using the device's Camera/IMU. Currently, the default configuration is EuROC, so to use an Android phone's Camera/IMU we need to use enable it.

   Uncomment `#define ANDROID_CAM 1` to use a smartphone's intrinsics. Currently, we have kept general values applicable to most Android phones based on [this](https://github.com/OSUPCVLab/mobile-ar-sensor-logger/wiki#configure-imu-parameters). However, you can calibrate your device using [Kalibr](https://github.com/ethz-asl/kalibr) and update these device specific values.

4. Build Variant:

   The debug build of the application is very inefficient and most likely tracking won't work on most phones with the debug build. For release build we need to sign the APK, follow instructions [here](https://developer.android.com/studio/publish/app-signing) to create a signing configuration. Select APK option and not the App Bundle option.

   After you have created the signing config for release, to change the build variant navigate to ***Build -> Select Build Variants*** then on the left-hand side panel, change ***Active Build Variant*** to ***release***.

5. Now we have everything we need to run GlDemo on an Android Phone. Hit the run button, it should open the application and you can see the GlDemo scene. ***Note that tracking with OpenVINS will only work with release build***. Also, make sure to uncomment ANDROID_CAM in OpenVINS plugin for using the Android IMU Cam plugin for online tracking.