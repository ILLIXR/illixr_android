package com.example.ILLIXR;

import android.content.Context;
public class NativeBridge {
    static {
        System.loadLibrary("native-lib");
    }

    public static native void setContext(Context context);
}
