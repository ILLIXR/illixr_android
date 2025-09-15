#!/usr/bin/env sh

# run this script once to unpack the libraries

tar zxf includes.tar.gz

cd lib

for lib_path in arm64-v8a armeabi-v7a x86 x86_64; do
    cd $lib_path
    for zip_lib in *.zip; do
        unzip $zip_lib
    done
    cd ..
done

cd ../sdk/etc
tar zxf haarcascades.tgz
cd ../native

for main_path in libs staticlibs; do
    cd $main_path
    for lib_path in arm64-v8a armeabi-v7a x86 x86_64; do
        cd $lib_path
        for zip_lib in *.zip; do
            unzip $zip_lib
        done
        cd ..
    done
    cd ..
done

cd 3rdparty/libs
for lib_path in arm64-v8a armeabi-v7a x86 x86_64; do
    cd $lib_path
    for zip_lib in *.zip; do
        unzip $zip_lib
    done
    cd ..
done
cd ../..
