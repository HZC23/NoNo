package com.hzc23.nonocontroller.ui

import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.emptyFlow

class DummyDataStore : DataStore<Preferences> {
    override val data: Flow<Preferences> = emptyFlow()

    override suspend fun updateData(transform: suspend (t: Preferences) -> Preferences): Preferences {
        return kotlinx.coroutines.runBlocking { transform(androidx.datastore.preferences.core.emptyPreferences()) }
    }
}
