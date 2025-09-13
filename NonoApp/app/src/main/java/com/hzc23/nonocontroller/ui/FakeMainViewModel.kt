package com.hzc23.nonocontroller.ui

import com.hzc23.nonocontroller.MainViewModel

import android.content.Context

class FakeMainViewModel(context: Context) : MainViewModel(FakeSettingsDataStore(context)) {
    override fun onSerialReceived(line: String) {
        // do nothing
    }
}