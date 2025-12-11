# ILLIXR-net on Android

This repo is the initial version of ILLIXR-net for Android. It is designed to run on the Quest 3, but should run on any Android headset running and OpenXR runtime.

!!! note

    This repo is temporary. We will be merging this into the the main ILLIXR-net repo in the near future.

This code contains more parts than are currently needed by ILLIXR-net. The relevant plugins that are used in this setup are:

- `offline_rendering_mediandk/rx` (this plugin decodes the images received over wifi, using h.265)
- `openxr_interface` (this plugin is the OpenXR application that communicates with the Horizon OS to get sensor information and submit frames)
- `quest3/head_pose/tx` (this plugin sends the current head pose over wifi to the server running ILLIXR-net)
- `tcp_network_backend` (this plugin transmits to and receives messages from ILLIXR-net on the server)

Below is a schematic of the headset side architecture

``` mermaid
flowchart TD
    V[Server]
    subgraph Headset
    subgraph ILLIXR-net
    T[tcp_network_backend]
    O[offload_rendering_mediandk.rx]
    Q[quest3.head_pose.tx]
    X[openxr_interface]
    S[switchboard]
    X -->|head pose| S
    S -->|head pose| Q
    Q -->|head_pose| T
    T -->|compressed frames| S
    S -->|compressed frames| O
    O -->|frames| S
    S -->|frames| X
    end
    H[Horizon OS]
    end
   T -->|head_pose| V
   H -.->|head pose, OpenXR| X
   V -.->|compressed frames| T
   X -.->|swapchain, OpenXR| H
```
## Building Instructions

### Requirements

- Android Studio: the development platform, any recent version should be fine
  - Android NDK, version 27.2.12479018
  - Android SDK, version 13.0 (API 33)
- Linux machine: there are a few setup scripts that only work on Linux machines (Windows support will be added shortly)
- Eigen3 and protobuf compiler (protoc) need to be installed on the system

### Configuring

After cloning the repository be sure to run

```bash
cd <your repo location>
git submodule init
git submodule update
cd android_libraries/third_party
./prepare.sh
```

This imports the android_libraries submodule and expands its contents.

ILLIXR-net is a CMake-based C++ project. After opening the project in Android Studio, Gradle should sync (configure) the project automatically.

### Building

You will need to edit a single file before building. In `app/src/main/cpp/src/main.cpp`, edit the two lines:

- `ILLIXR_TCP_CLIENT_IP` should contain the IP address of the Quest headset
- `ILLIXR_TCP_SERVER_IP` should contain the IP address of the Windows server

Then just build the project.

### Running

You will need to enable development mode on the headset. See [here](https://developers.meta.com/horizon/documentation/unity/unity-env-device-setup) for instructions. Plug the headset into you machine via a USB 3.0 data cable, you will be prompted in the headset to allow a connection to your computer (first time only), then you will be prompted to allow data transfer over the USB. Click on the box to enable it. This question is asked every time you plug in the headset and the message only stays up for a few seconds, you may need to unplug and re-plug the headset if you miss it the first time. The device list at the top of the Android Studio editor should automatically list the Quest 3. To run ILLIXR-net, just click `Run->Run 'app'` or `Run->Debug 'app'`.

!!! note

    Currently, there is a specific order you must launch each of the components in:
    
    1. ILLIXR-net on the server
    2. ILLIXR-net on the headset
    3. Unity App

    It can take a bit before images start flowing as things are getting configured.

!!! warning

    There are a few known issues with the current setup:

    1. When running in Debug mode on the server, the encoder can get behind quickly
    2. The images displayed on the headset will make you feel like you are crosseyed, this is due to some transform being applied when it should not. This likely won't be tracked down and fixed (see below)

    Both of these issues should be addressed when ILLIXR-net on the server is using an OpenXR interface instead of getting frames driectly from the Unity App. OpenXR will enforce the cadence of the images, and images submitted to the swapchain will be in the proper format for the headset to display properly. 




# ILLIXR on Android

## Setup Instructions

Depending on which NDK, SDK, and API versions you are using, you may need to change the version of Java you have. Java-21 is not supported for may of the common
combinations of these versions, but Java-17 is widely supported.

> [!NOTE]
> ILLIXR for Android is known to work with Android SDK 33.0, NDK 27.2.12479018, and Gradle 8.10. Any other configuration may require changes to the code.

### Generate a Keystore

This is for signing the App

``` bash
$JAVA_HOME/bin/keytool -genkeypair -v -keystore /home/friedel/illixr.keystore -alias illixr -keyalg RSA -keysize 2048 -validity 10000
```

For any passwords, use `illixr`, most other questions are optional. If you want a different password, you will need to change a few lines
in `app/build.gradle`.

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
    5. Scroll down to the `NDK` section and check `27.2.12479018` and `21.4.7075529` if they are not already checked
    6. Scroll down to the `Android SDK Command-line Tools` section and check the entry for 13.0, if it is not already checked
    7. Scroll down to the `CMake` section and check the entry for `3.22.1` if it is not already checked
    8. Click `OK`
    9. Android Studio will download and install the requested components, this may take a while
    10. The installed components should be in `${HOME}/Android/Sdk`

For this project, we only built for the `arm64-v8a` architecture, as it is the architecture of our test device.
If your has a different architecture, you will need to make changes to some of the commands below, as well as
some lines in a `build.gradle` file.

We leave it up to the user to connect their Android device to Android Studio. Instructions on this and
how to build a run an App can be found on the Android documentation [pages][2].

### Dependencies

While ILLIXR for Android depends on several common packages, the versions installed by OS packages managers are not compatible with the Android architecture and need to be built from source. The ILLIXR build system will compile these libraries for you, and store them in the `third_party` folder. Only delete the contents of this folder if you wish to rebuild these external libraries.

### ILLIXR

1. Clone the repository

   ``` bash
   cd ${ROOT_DIR}
   git clone https://github.com/ILLIXR/illixr_android.git
   ```
2. Re-open Android Studio so it can pick up any new environment variables.

3. Open it in Android Studio
   1. `File -> Open`
   2. Navigate the repo you just cloned
   3. Click `Open`

4. Wait for the Gradle Sync to finish.

5. If you need to build for a different architecture or for different SDK versions, edit the necessary lines in `${ROOT_DIR}/illixr_android/app/build.gradle`

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

## Running with OpenXR

We use a version of Monado to act as an OpenXR interface.

### Get Monado

1. Clone the repo

   ``` bash
   git clone https://github.com/ILLIXR/Illixr_Monado_Android.git
   ```
2. Open Android Studio

3. Update the tools (these choices are much less flexible than for ILLIXR)
   1. Open the `SDK Manager` `Tools -> SDK Manager`
   2. Under `Android SDK Build-Tools` check 31.0.0 if it is not already
   3. Scroll down to the `NDK` section and check `21.4.7075529` if it is not already checked
   4. Scroll down to the `CMake` section and check the entry for `3.10.2` if it is not already checked

4. Open the ILLIXR for Android project

5. Edit `app/src/main/cpp/CMakeLists.txt`, by uncommenting the `add_definitions(-DENABLE_MONADO=1)` line
6. Rebuild ILLIXR for Android
7. Copy the libraries into the Monado build
 
   ``` bash
   mkdir -p ${ROOT_DIR}/Illixr_Monado_Android/src/xrt/targets/openxr_android/src/main/jniLibs/arm64-v8a
   cp ${ROOT_DIR}/illixr_android/app/build/intermediates/merged_native_libs/debug/out/lib/arm64-v8a/*.so ${ROOT_DIR}/Illixr_Monado_Android/src/xrt/targets/openxr_android/src/main/jniLibs/arm64-v8a/.
   ```

8. Open the ILLIXR_Monado_Android project in Android Studio

9. Configure and build

10. You can now run Monado on your phone in the background and connect to it via OpenXR enabled Apps.

[//]: (- references -)

## Running with OpenXR

We use a version of Monado to act as an OpenXR interface.

### Get Monado

1. Clone the repo

   ``` bash
   git clone https://github.com/ILLIXR/Illixr_Monado_Android.git
   ```
2. Open Android Studio

3. Update the tools (these choices are much less flexible than for ILLIXR)
   1. Open the `SDK Manager` `Tools -> SDK Manager`
   2. Under `Android SDK Build-Tools` check 31.0.0 if it is not already
   3. Scroll down to the `NDK` section and check `21.4.7075529` if it is not already checked
   4. Scroll down to the `CMake` section and check the entry for `3.10.2` if it is not already checked

4. Open the ILLIXR for Android project

5. Edit `app/src/main/cpp/CMakeLists.txt`, by uncommenting the `add_definitions(-DENABLE_MONADO=1)` line
6. Rebuild ILLIXR for Android
7. Copy the libraries into the Monado build

   ``` bash
   mkdir -p ${ROOT_DIR}/Illixr_Monado_Android/src/xrt/targets/openxr_android/src/main/jniLibs/arm64-v8a
   cp ${ROOT_DIR}/illixr_android/app/build/intermediates/merged_native_libs/debug/mergeDebugNativeLibs/out/lib/arm64-v8a/*.so ${ROOT_DIR}/Illixr_Monado_Android/src/xrt/targets/openxr_android/src/main/jniLibs/arm64-v8a/.
   ```

8. Open the ILLIXR_Monado_Android project in Android Studio

9. Configure/Sync and build

10. You can now run Monado on your phone in the background and connect to it via OpenXR enabled Apps.

[//]: # (- references -)

[1]:   https://developer.android.com/studio
[2]:   https://developer.android.com/studio/intro
