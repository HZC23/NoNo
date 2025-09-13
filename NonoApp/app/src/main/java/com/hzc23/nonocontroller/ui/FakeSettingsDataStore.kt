package com.hzc23.nonocontroller.ui

import android.content.Context
import com.hzc23.nonocontroller.SettingsDataStore

// This is a fake implementation of SettingsDataStore for use in previews and tests.
// It requires a Context, but it won't actually be used to create a real DataStore.
class FakeSettingsDataStore(context: Context) : SettingsDataStore(context) {
    init {
        // Override the internal dataStore with a dummy one for testing/preview purposes.
        (this as SettingsDataStore).dataStore = DummyDataStore()
    }
}
