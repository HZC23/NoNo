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

open class SettingsDataStore(context: Context) {
    internal var dataStore: DataStore<Preferences> = context.dataStore

    private object PreferencesKeys {
        val IS_LAYOUT_INVERTED = booleanPreferencesKey("is_layout_inverted")
        val IS_DARK_MODE_ENABLED = booleanPreferencesKey("is_dark_mode_enabled")
    }

    val isLayoutInverted: Flow<Boolean> = dataStore.data.map { preferences ->
        preferences[PreferencesKeys.IS_LAYOUT_INVERTED] ?: false
    }

    val isDarkModeEnabled: Flow<Boolean> = dataStore.data.map { preferences ->
        preferences[PreferencesKeys.IS_DARK_MODE_ENABLED] ?: false
    }

    suspend fun toggleLayoutInversion() {
        dataStore.edit { preferences ->
            val currentValue = preferences[PreferencesKeys.IS_LAYOUT_INVERTED] ?: false
            preferences[PreferencesKeys.IS_LAYOUT_INVERTED] = !currentValue
        }
    }

    suspend fun toggleDarkMode() {
        dataStore.edit { preferences ->
            val currentValue = preferences[PreferencesKeys.IS_DARK_MODE_ENABLED] ?: false
            preferences[PreferencesKeys.IS_DARK_MODE_ENABLED] = !currentValue
        }
    }
}
