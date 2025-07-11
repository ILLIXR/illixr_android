# ILLIXR on Android

## Setup Instructions

### Android Studio

1. Download Android Studio from [here][1].

2. Extract the archive

   ``` bash
   tar xf <FILE_NAME>
   ```

3. Launch Android Studio with the following and accept licences.

   ``` bash
   android-studio/bin/studio.sh
   ```

4. Configure the build system
    1. Open the `SDK Manager` `Tools -> SDK Manager`
    2. Select the platform version to use (in the `SDK Platforms` tab)
        1. this should be the Android version of your device, at minimum
        2. our testbed is running Android 13.0, so we will use API level 33 in this example
    3. Switch to the `SDK Tools` tab
    4. Under `Android SDK Build-Tools` check 33.0.0 if it is not already
    5. Scroll down to the `NDK` section and check `27.0.12077973` if it is not already checked
    6. Scroll down to the `Android SDK Command-line Tools` section and check the entry for 13.0, if it is not already checked
    7. Scroll down to the `CMake` section and check the entry for `3.22.1` if it is not already checked
    8. Click `OK`
    9. Android Studio will download and install the requested components, this may take a while
    10. The installed components should be in `~/Android/Sdk`

For this project, we only built for the `arm64-v8a` architecture, as it is the architecture of our test device.
If your has a different architecture, you will need to make changes to some of the commands below, as well as
some lines in a `build.gradle` file.

### Dependencies

While ILLIXR for Android
depends on several common packages, the versions installed by OS packages managers are not compatible with
the Android architecture and need to be built from source. For the purposes of this example we are calling the directory where we are downloading packages `ROOT_DIR`.

``` bash
export ROOT_DIR=<>
```

replace the `<>` above with your working directory.
#### OpenCV

1. Download OpenCV and its extras

   ``` bash
   wget -O opencv-4.5.5.zip https://github.com/opencv/opencv/archive/4.5.5.zip
   wget -O opencv_contrib-4.5.5.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.5.zip
   ```

2. Unzip the archives

   ``` bash
   unzip opencv-4.5.5.zip
   unzip opencv_contrib-4.5.5.zip
   ```

3. Configure and build. We are installing into `${ROOT_DIR}`, but you can choose any destination

   ``` bash
   cd opencv-4.5.5
   mkdir build
   cd build
   ~/Android/Sdk/cmake/3.22.1/bin/cmake -DANDROID_ABI=arm64-v8a -DBUILD_DOCS=OFF -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${ROOT_DIR} -DCMAKE_TOOLCHAIN_FILE=~/Android/Sdk/ndk/27.0.12077973/build/cmake/android.toolchain.cmake -DENABLE_PIC=ON -DOPENCV_EXTRA_MODULES_PATH=${ROOT_DIR}/opencv_contrib-4.5.5/modules -DANDROID_SDK_TOOLS=~/Android/Sdk/build-tools/33.0.0 -DANDROID_SDK_BUILD_TOOLS_VERSION=33.0.0 ..
   make -j4
   make install
   ```

4. Set an environment variable so the ILLIXR Android build can find this

   ``` bash
   export OPENCV_ANDROID_ROOT=${ROOT_DIR}
   ```

#### Boost

1. Download Boost and unpack

   ``` bash
   cd ${ROOT_DIR}
   wget https://archives.boost.io/release/1.87.0/source/boost_1_87_0.tar.gz
   tar xf boost_1_87_0.tar.gz
   ```
   
2. Get the `Boost-for-Android` code

   ``` bash
   git clone -b ndk_27c_boost_1.87.0 --single-branch https://github.com/dec1/Boost-for-Android.git
   ```

3. Configure and build

   ``` bash
   cd Boost-for-Android
   export BOOST_DIR=${ROOT_DIR}/boost_1_87_0
   export NDK_DIR=~/Android/Sdk/ndk/27.0.12077973
   export ABI_NAMES="arm64-v8a"
   export LINKAGES="shared static"
   ./__build.sh
   ```

    The build products will be in `${ROOT_DIR}/Boost-for-Android/build/install`

4. Set an environment variable so the ILLIXR Android build can find this

   ``` bash
   export BOOST_ANDROID_ROOT=${ROOT_DIR}/Boost-for-Android/build/install
   ```
   
5. If you run into any issues with building Boost. See the `README.md` file in `${ROOT_DIR}/Boost-for-Android`

#### Eigen3

Eigen3 does not need special compilation as it is a header-only library, so the system version is all that is needed.

``` bash
sudo apt install libeigen3-dev
```

#### spdlog

ILLIXR uses sdplog for its logging framework. This library is automatically built as part of the ILLIXR for Android system.

### ILLIXR

1. Clone the repository

   ``` bash
   cd ${ROOT_DIR}
   git clone https://github.com/ILLIXR/illixr_android.git
   ```

2. Open it in Android Studio
    1. `File -> Open`
    2. Navigate the repo you just cloned
    3. Click `Open`

3. Wait for the Gradle Sync to finish. 

4. If you need to build for a different architecture or for different SDK versions, edit the necessary lines in `${ROOT_DIR}/illixr_android/app/build.gradle`
 
### Install Application Data

1. Now we have all the dependencies ready to build the project. Hit the build button in Android Studio to build the project. If there are errors make sure all the paths set in the previous steps are correct.

2. We need to manually install data required for GlDemo and Pose_lookup. However we can ony do it when the application is installed on the phone. So, hit the run button and install the application on the phone. The app won't run because we haven't added the data yet.

3. Now while the phone is connected open Device File Explorer located on the right hand side menu of Android Studio. Select the device name. Navigate to `/sdcard/Android/data/com.example.native_activity`. Right click and select Upload, now navigate to the folder `illixr_android/app/src/main/cpp/` and select mav0. Do the same for demo_data.
   mav0 contains the ground truth pose form EuRoC dataset required for Pose_lookup and demo_data is used by gldemo.

4. It is possible that the app is not installed in sdcard but only present in the internal memory which is `/data/data/com.example.native_activity`. In this case install the mav0 and demo_data inside `/data/data/com.example.native_activity`.

5. Based on where the data is installed, change the application path in `illixr_android/app/src/main/cpp/main.cpp` line 23.

### Provide App permissions

1. The application needs permissions to read and write files. Go to setting in the Android phone, search for native-activity. In the application details go to the permissions section. It will show all the permission that are denied to the application. Click on them to allow the app these permissions. This step may look different based on the specific phone model.

### Run the app

Now we have everything we need to run GlDemo on an Android Phone. Hit the run button, it should open the application and you can see the GlDemo scene.

[//]: (- references -)

[1]:   https://developer.android.com/studio