package com.example.ILLIXR;

import android.hardware.camera2.*;
import android.content.Context;
import android.util.Log;

public class QuestCamera {
    static {
        System.loadLibrary("native-lib");
    }

    private static native void onCustomKeyValues(int position_val, int source_val);

    public static void getCustomKeyValues(Context context, String cameraId) {
        Integer positionVal = -1;
        Integer sourceVal = -1;
        try {
            CameraManager manager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
            CameraCharacteristics camCharacteristics = manager.getCameraCharacteristics(cameraId);
            try {
                CameraCharacteristics.Key<Integer> positionKey = new CameraCharacteristics.Key<>("com.meta.extra_metadata.position", Integer.class);
                positionVal = camCharacteristics.get(positionKey);
            } catch (Exception e) {
                Log.e("QuestCamera", "Error querying custom key 'position'", e);
            }
            try {
                CameraCharacteristics.Key<Integer> sourceKey = new CameraCharacteristics.Key<>("com.meta.extra_metadata.camera_source", Integer.class);
                sourceVal = camCharacteristics.get(sourceKey);
            } catch (Exception e) {
                Log.e("QuestCamera", "Error querying custom key 'position'", e);
            }
            onCustomKeyValues(positionVal, sourceVal);
        } catch (Exception e) {
            Log.e("QuestCamera", "error querying keys for camera " + cameraId, e);
        }
    }
}