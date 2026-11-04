package com.scand3d

import android.app.Application
import android.util.Log
import com.scand3d.core.bridge.NativeCore

/**
 * Loads the native core at process start: a broken or missing .so then surfaces in logcat
 * immediately instead of during the first scan.
 */
class Scand3dApplication : Application() {

    override fun onCreate() {
        super.onCreate()
        val error = NativeCore.loadError
        if (error == null) {
            Log.i(LOG_TAG, "native core loaded")
        } else {
            Log.e(LOG_TAG, "native core failed to load", error)
        }
    }

    private companion object {
        const val LOG_TAG = "Scand3dApp"
    }
}
