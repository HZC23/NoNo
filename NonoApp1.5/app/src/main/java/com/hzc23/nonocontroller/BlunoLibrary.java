package com.hzc23.nonocontroller;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Handler;
import android.os.IBinder;
import android.util.Log;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;

public class BlunoLibrary {

    public interface BlunoListener {
        void onConnectionStateChange(connectionStateEnum theconnectionStateEnum);
        void onSerialReceived(String theString);
        void onDeviceScanned(List<BluetoothDevice> devices);
    }

    private Context mContext;
    private BlunoListener mListener;
    private List<BluetoothDevice> mLeDevices;

    public BlunoLibrary(Context context) {
        mContext = context;
        mLeDevices = new ArrayList<>();
    }

    public void setBlunoListener(BlunoListener listener) {
        mListener = listener;
    }

    public void serialSend(String theString) {
        if (mConnectionState == connectionStateEnum.isConnected) {
            mSCharacteristic.setValue(theString);
            mBluetoothLeService.writeCharacteristic(mSCharacteristic);
        }
    }

    private int mBaudrate = 115200;
    private String mPassword = "AT+PASSWOR=DFRobot\r\n";
    private String mBaudrateBuffer = "AT+CURRUART=" + mBaudrate + "\r\n";

    public void serialBegin(int baud) {
        mBaudrate = baud;
        mBaudrateBuffer = "AT+CURRUART=" + mBaudrate + "\r\n";
    }

    private static BluetoothGattCharacteristic mSCharacteristic, mModelNumberCharacteristic, mSerialPortCharacteristic, mCommandCharacteristic;
    private BluetoothLeService mBluetoothLeService;
    private ArrayList<ArrayList<BluetoothGattCharacteristic>> mGattCharacteristics =
            new ArrayList<ArrayList<BluetoothGattCharacteristic>>();
    private BluetoothAdapter mBluetoothAdapter;
    private boolean mScanning = false;
    private String mDeviceName;
    private String mDeviceAddress;
    public enum connectionStateEnum { isNull, isScanning, isToScan, isConnecting, isConnected, isDisconnecting };
    public connectionStateEnum mConnectionState = connectionStateEnum.isNull;
    private static final int REQUEST_ENABLE_BT = 1;

    private Handler mHandler = new Handler();
    public boolean mConnected = false;

    private final static String TAG = BlunoLibrary.class.getSimpleName();

    private Runnable mConnectingOverTimeRunnable = new Runnable() {
        @Override
        public void run() {
            if (mConnectionState == connectionStateEnum.isConnecting) {
                mConnectionState = connectionStateEnum.isToScan;
                if (mListener != null) {
                    mListener.onConnectionStateChange(mConnectionState);
                }
                mBluetoothLeService.close();
            }
        }
    };

    private Runnable mDisonnectingOverTimeRunnable = new Runnable() {
        @Override
        public void run() {
            if (mConnectionState == connectionStateEnum.isDisconnecting) {
                mConnectionState = connectionStateEnum.isToScan;
                if (mListener != null) {
                    mListener.onConnectionStateChange(mConnectionState);
                }
                mBluetoothLeService.close();
            }
        }
    };

    public static final String SerialPortUUID = "0000dfb1-0000-1000-8000-00805f9b34fb";
    public static final String CommandUUID = "0000dfb2-0000-1000-8000-00805f9b34fb";
    public static final String ModelNumberStringUUID = "00002a24-0000-1000-8000-00805f9b34fb";

    public void initialize() {
        if (!initiate()) {
            Toast.makeText(mContext, "Bluetooth not supported", Toast.LENGTH_SHORT).show();
            return;
        }

        Intent gattServiceIntent = new Intent(mContext, BluetoothLeService.class);
        mContext.bindService(gattServiceIntent, mServiceConnection, Context.BIND_AUTO_CREATE);
    }

    public void onResume() {
        Log.d(TAG, "onResume");
        if (mBluetoothAdapter != null && !mBluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            ((Activity) mContext).startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }
        Log.d(TAG, "registering receiver");
        if (Build.VERSION.SDK_INT >= 33) {
            mContext.registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter(), Context.RECEIVER_NOT_EXPORTED);
        } else {
            mContext.registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        }
    }

    public void onPause() {
        Log.d(TAG, "onPause");
        scanLeDevice(false);
        Log.d(TAG, "unregistering receiver");
        mContext.unregisterReceiver(mGattUpdateReceiver);
        mLeDevices.clear();
        mConnectionState = connectionStateEnum.isToScan;
        if (mListener != null) {
            mListener.onConnectionStateChange(mConnectionState);
        }
        if (mBluetoothLeService != null) {
            mBluetoothLeService.disconnect();
            mHandler.postDelayed(mDisonnectingOverTimeRunnable, 10000);
        }
        mSCharacteristic = null;
    }

    public void onDestroy() {
        if (mBluetoothLeService != null) {
            mHandler.removeCallbacks(mDisonnectingOverTimeRunnable);
            mBluetoothLeService.close();
            mContext.unbindService(mServiceConnection);
            mBluetoothLeService = null;
        }
    }

    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (requestCode == REQUEST_ENABLE_BT && resultCode == Activity.RESULT_CANCELED) {
            // User chose not to enable Bluetooth.
        }
    }

    private boolean initiate() {
        if (!mContext.getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
            return false;
        }

        final BluetoothManager bluetoothManager = (BluetoothManager) mContext.getSystemService(Context.BLUETOOTH_SERVICE);
        mBluetoothAdapter = bluetoothManager.getAdapter();

        return mBluetoothAdapter != null;
    }

    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
        @SuppressLint("DefaultLocale")
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            Log.d(TAG, "onReceive action: " + action);
            if (BluetoothLeService.ACTION_GATT_CONNECTED.equals(action)) {
                mConnected = true;
                mHandler.removeCallbacks(mConnectingOverTimeRunnable);
                if (mListener != null) {
                    mListener.onConnectionStateChange(connectionStateEnum.isConnected);
                }
            } else if (BluetoothLeService.ACTION_GATT_DISCONNECTED.equals(action)) {
                mConnected = false;
                mConnectionState = connectionStateEnum.isToScan;
                if (mListener != null) {
                    mListener.onConnectionStateChange(mConnectionState);
                }
                mHandler.removeCallbacks(mDisonnectingOverTimeRunnable);
                mBluetoothLeService.close();
            } else if (BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {
                getGattServices(mBluetoothLeService.getSupportedGattServices());
            } else if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {
                if (mSCharacteristic == mModelNumberCharacteristic) {
                    if (intent.getStringExtra(BluetoothLeService.EXTRA_DATA).toUpperCase().startsWith("DF BLUNO")) {
                        mBluetoothLeService.setCharacteristicNotification(mSCharacteristic, false);
                        mSCharacteristic = mCommandCharacteristic;
                        mSCharacteristic.setValue(mPassword);
                        mBluetoothLeService.writeCharacteristic(mSCharacteristic);
                        mSCharacteristic.setValue(mBaudrateBuffer);
                        mBluetoothLeService.writeCharacteristic(mSCharacteristic);
                        mSCharacteristic = mSerialPortCharacteristic;
                        mBluetoothLeService.setCharacteristicNotification(mSCharacteristic, true);
                        mConnectionState = connectionStateEnum.isConnected;
                        if (mListener != null) {
                            mListener.onConnectionStateChange(mConnectionState);
                        }
                    } else {
                        Toast.makeText(mContext, "Please select DFRobot devices", Toast.LENGTH_SHORT).show();
                        mConnectionState = connectionStateEnum.isToScan;
                        if (mListener != null) {
                            mListener.onConnectionStateChange(mConnectionState);
                        }
                    }
                } else if (mSCharacteristic == mSerialPortCharacteristic) {
                    if (mListener != null) {
                        mListener.onSerialReceived(intent.getStringExtra(BluetoothLeService.EXTRA_DATA));
                    }
                }
            }
        }
    };

    public void scanLeDevice(final boolean enable) {
        if (enable) {
            mLeDevices.clear();
            if (!mScanning) {
                mScanning = true;
                mBluetoothAdapter.startLeScan(mLeScanCallback);
            }
        } else {
            if (mScanning) {
                mScanning = false;
                mBluetoothAdapter.stopLeScan(mLeScanCallback);
            }
        }
    }

    public void connect(String address) {
        if (mBluetoothLeService != null) {
            if (mBluetoothLeService.connect(address)) {
                Log.d(TAG, "Connect request success");
                mConnectionState = connectionStateEnum.isConnecting;
                if (mListener != null) {
                    mListener.onConnectionStateChange(mConnectionState);
                }
                mHandler.postDelayed(mConnectingOverTimeRunnable, 10000);
            } else {
                Log.d(TAG, "Connect request fail");
                mConnectionState = connectionStateEnum.isToScan;
                if (mListener != null) {
                    mListener.onConnectionStateChange(mConnectionState);
                }
            }
        }
    }

    public void disconnect() {
        if (mBluetoothLeService != null) {
            mBluetoothLeService.disconnect();
            mHandler.postDelayed(mDisonnectingOverTimeRunnable, 10000);
            mConnectionState = connectionStateEnum.isDisconnecting;
            if (mListener != null) {
                mListener.onConnectionStateChange(mConnectionState);
            }
        }
    }

    private ServiceConnection mServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName componentName, IBinder service) {
            mBluetoothLeService = ((BluetoothLeService.LocalBinder) service).getService();
            if (!mBluetoothLeService.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            mBluetoothLeService = null;
        }
    };

    private BluetoothAdapter.LeScanCallback mLeScanCallback = new BluetoothAdapter.LeScanCallback() {
        @Override
        public void onLeScan(final BluetoothDevice device, int rssi, byte[] scanRecord) {
            Log.d(TAG, "onLeScan: device found: " + device.getName());
            if (!mLeDevices.contains(device)) {
                mLeDevices.add(device);
                if (mListener != null) {
                    mListener.onDeviceScanned(new ArrayList<>(mLeDevices));
                }
            }
        }
    };

    private void getGattServices(List<BluetoothGattService> gattServices) {
        if (gattServices == null) return;
        String uuid;
        mModelNumberCharacteristic = null;
        mSerialPortCharacteristic = null;
        mCommandCharacteristic = null;
        mGattCharacteristics.clear();

        for (BluetoothGattService gattService : gattServices) {
            uuid = gattService.getUuid().toString();
            List<BluetoothGattCharacteristic> gattCharacteristics = gattService.getCharacteristics();
            ArrayList<BluetoothGattCharacteristic> charas = new ArrayList<>();

            for (BluetoothGattCharacteristic gattCharacteristic : gattCharacteristics) {
                charas.add(gattCharacteristic);
                uuid = gattCharacteristic.getUuid().toString();
                if (uuid.equals(ModelNumberStringUUID)) {
                    mModelNumberCharacteristic = gattCharacteristic;
                } else if (uuid.equals(SerialPortUUID)) {
                    mSerialPortCharacteristic = gattCharacteristic;
                } else if (uuid.equals(CommandUUID)) {
                    mCommandCharacteristic = gattCharacteristic;
                }
            }
            mGattCharacteristics.add(charas);
        }

        if (mModelNumberCharacteristic == null || mSerialPortCharacteristic == null || mCommandCharacteristic == null) {
            Toast.makeText(mContext, "Please select DFRobot devices", Toast.LENGTH_SHORT).show();
            mConnectionState = connectionStateEnum.isToScan;
            if (mListener != null) {
                mListener.onConnectionStateChange(mConnectionState);
            }
        } else {
            mSCharacteristic = mModelNumberCharacteristic;
            mBluetoothLeService.setCharacteristicNotification(mSCharacteristic, true);
            mBluetoothLeService.readCharacteristic(mSCharacteristic);
        }
    }

    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_CONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(BluetoothLeService.ACTION_DATA_AVAILABLE);
        return intentFilter;
    }

    public void buttonScanOnClickProcess() {
        switch (mConnectionState) {
            case isNull:
            case isToScan:
                mConnectionState = connectionStateEnum.isScanning;
                if (mListener != null) {
                    mListener.onConnectionStateChange(mConnectionState);
                }
                scanLeDevice(true);
                break;
            case isConnected:
                disconnect();
                break;
            default:
                break;
        }
    }
}
