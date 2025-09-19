package com.hzc23.nonocontroller

import android.content.Context
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.booleanPreferencesKey
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.preferencesDataStore
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

private val Context.dataStore: DataStore<Preferences> by preferencesDataStore(name = "settings")

class SettingsDataStore(private val context: Context) {

    private val isLayoutInvertedKey = booleanPreferencesKey("is_layout_inverted")
    private val isDarkModeEnabledKey = booleanPreferencesKey("is_dark_mode_enabled")

    val isLayoutInverted: Flow<Boolean> = context.dataStore.data
        .map { preferences ->
            preferences[isLayoutInvertedKey] ?: false
        }

    val isDarkModeEnabled: Flow<Boolean> = context.dataStore.data
        .map { preferences ->
            preferences[isDarkModeEnabledKey] ?: false
        }

    suspend fun toggleLayoutInversion() {
        context.dataStore.edit {
            val currentValue = it[isLayoutInvertedKey] ?: false
            it[isLayoutInvertedKey] = !currentValue
        }
    }

    suspend fun toggleDarkMode() {
        context.dataStore.edit {
            val currentValue = it[isDarkModeEnabledKey] ?: false
            it[isDarkModeEnabledKey] = !currentValue
        }
    }
}
