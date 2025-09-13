package com.hzc23.nonocontroller

import android.app.Application
import android.util.Log
import java.io.File
import java.io.PrintWriter
import java.io.StringWriter

class NonoApplication : Application() {

    override fun onCreate() {
        super.onCreate()
        Thread.setDefaultUncaughtExceptionHandler {
            thread, e -> handleUncaughtException(thread, e)
        }
    }

    private fun handleUncaughtException(thread: Thread, e: Throwable) {
        val stackTrace = StringWriter()
        e.printStackTrace(PrintWriter(stackTrace))
        val message = e.message

        val log = "FATAL EXCEPTION: ${thread.name}\n" +
                "MESSAGE: $message\n" +
                "STACKTRACE:\n$stackTrace"

        Log.e("NonoApplication", log)

        try {
            val file = File(filesDir, "crash_log.txt")
            file.writeText(log)
        } catch (e: Exception) {
            Log.e("NonoApplication", "Failed to write crash log to file", e)
        }

        // Terminate the process
        android.os.Process.killProcess(android.os.Process.myPid())
        System.exit(10)
    }
}