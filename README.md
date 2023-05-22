# ILLIXR ANDROID

## Setup Instructions

### Disclaimer

This repository still needs reviews from ILLIXR developers. It is only meant to be used for reviewing code for Android. Currently, it only contains code to run ILLIXR with GlDemo + pose_lookup (Ground Truth Pose from EuRoC Dataset). The instructions are only tested on Ubuntu 20 and Android Debug build (Release build requires signing the apk, those instructions will be added later).

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

1. Download OpenCV 4.5.5 from [here](https://opencv.org/releases/). Click on the Android option. Download and extract the zip file.

2. Change Line 21 and 22 in `app/src/main/cpp/CMakeLists.txt` with the OpenCV folder path.

### Setup Boost

1. Follow instructions [here](https://github.com/dec1/Boost-for-Android) to download and build boost for Android. Make sure you build it for the right architecture and instruction set you need. For e.g. Android Emulator may require x86 whereas most Android phones will need arm64.

2. Change Line 25 in `app/src/main/cpp/CMakeLists.txt` with the correct folder name.

### Setup Eigen

1. Download Latest release of EIgen from [here](https://eigen.tuxfamily.org/index.php?title=Main_Page).

2. Change Line 28 in `app/src/main/cpp/CMakeLists.txt` with the installed Eigen directory path.

### Install Application Data

1. Now we have all the dependencies ready to build the project. Hit the build button in Android Studio to build the project. If there are errors make sure all the paths set in the previous steps are correct.

2. We need to manually install data required for GlDemo and Pose_lookup. However we can ony do it when the application is installed on the phone. So, hit the run button and install the application on the phone. The app won't run because we haven't added the data yet.

3. Now while the phone is connected open Device File Explorer located on the right hand side menu of Android Studio. Select the device name. Navigate to `/sdcard/Android/data/com.example.native_activity`. Right click and select Upload, now navigate to the folder `illixr_android/app/src/main/cpp/` and select mav0. Do the same for demo_data.
mav0 contains the groun truth pose form EuRoC dataset required for Pose_lookup and demo_data is used by gldemo.

4. It is possible that the app is not installed in sdcard but only present in the internal memory which is `/data/data/com.example.native_activity`. In this case install the mav0 and demo_data inside `/data/data/com.example.native_activity`.

5. Based on where the data is installed, change the application path in `illixr_android/app/src/main/cpp/main.cpp` line 23.

### Provide App permissions

1. The application needs permissino to read and write files. Go to setting in the Android phone, search for native-activity. In the application details go to the permissions section. It will show all the permission that are denied to the application. Click on them to allow the app these permissions. This step may look different based on the specific phone model.

### Run the app

Now we have everything we need to run GlDemo on an Android Phone. Hit the run button, it should open the application and you can see the GlDemo scene.