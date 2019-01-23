package com.example.stephaniegu.dronecontroller;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.ImageButton;
import android.widget.TextView;
import android.view.View.OnClickListener;
import java.io.IOException;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;
public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "bluetooth";
    private SensorManager sensorManager;
    private Sensor accelerometer;
    private TextView text;
    private boolean initialized;
    private BluetoothAdapter bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
    private BluetoothDevice device;
    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private OutputStream outputStream;
    private BluetoothSocket socket;
    private ImageButton testButton;
    private ImageButton upButton;
    private ImageButton downButton;
    private boolean start = true;
    private TextView speed;
    private int speedValue = 200;
    private String tempText;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        initialized = false;
        //creates the sensor to use accelerometer
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        if (bluetoothAdapter == null) {
            //bluetooth not supported output error message
        }
        //enable the bluetooth if not already
        if (!bluetoothAdapter.isEnabled()) {
            Intent enable = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enable, 0);
        }
        speed = findViewById(R.id.speed);
        speed.setText("0");
        testButton = findViewById(R.id.test_button);
        upButton = findViewById(R.id.up_button);
        downButton = findViewById(R.id.down_button);
        testButton.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View arg0) {
                if (start == true) {
                    //1 starts the speed
                    sendData("1");
                    testButton.setImageResource(R.drawable.stopedit);
                    speedValue = 200;
                    speed.setText(Integer.toString(speedValue));
                    start = false;
                } else {
                    //2 stops the speed
                    sendData("2");
                    testButton.setImageResource(R.drawable.startedit);
                    speedValue = 0;
                    speed.setText(Integer.toString(speedValue));
                    start = true;
                }
            }
        });
        upButton.setOnClickListener(new OnClickListener() {
            public void onClick(View arg0) {
                if (speedValue <= 235) {
                    speedValue+= 20;
                    speed.setText(Integer.toString(speedValue));
                    sendData("3");
                } else {
                    tempText = "MAX: " + Integer.toString(speedValue);
                    speed.setText(tempText);
                }
            }
        });
        downButton.setOnClickListener(new OnClickListener() {
            public void onClick(View arg0) {
                if (speedValue >= 20) {
                    speedValue -= 20;
                    speed.setText(Integer.toString(speedValue));
                    sendData("4");
                } else {
                    tempText = "MIN: " + Integer.toString(speedValue);
                    speed.setText(tempText);
                }
            }
        });
    }
    //unregisters the accelerometer when the app is closed
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
        if (outputStream != null) {
            try {
                outputStream.flush();
            } catch (IOException exception) {
                //error
                exception.printStackTrace();
            }
        }
        try {
            outputStream.close();
        } catch (IOException exception2) {
            //error message
        }
    }
    //turns on sensor when the app is opened
    protected void onResume() {
        super.onResume();
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        Set<BluetoothDevice> bonded = bluetoothAdapter.getBondedDevices();
        //check for connected device
        if (bonded.isEmpty()) {
            //device not connected
            Log.d(TAG, "Device not connected.");
        } else {
            for (BluetoothDevice iterator: bonded) {
                if (iterator.getName().equals("SICKODRONE")) {
                    device = iterator;
                }
            }
        }
        //creates the socket
        try {
            socket = device.createInsecureRfcommSocketToServiceRecord(MY_UUID);
        } catch (IOException exception) {
            exception.printStackTrace();
        }
        //bluetoothAdapter.cancelDiscovery();
        Log.d(TAG, "connecting");
        Thread connection = new Thread(new Runnable() {
            @Override
            public void run() {
                bluetoothAdapter.cancelDiscovery();
                try {
                    socket.connect();
                } catch (IOException exception) {
                    exception.printStackTrace();
                    try {
                        socket.close();
                    } catch (IOException exception2) {
                        exception2.printStackTrace();
                    }
                }
            }
        });
        connection.start();
        //creating outputstream for communication
        try {
            outputStream = socket.getOutputStream();
        } catch (IOException exception) {
            exception.printStackTrace();
        }
    }
    private void sendData(String message) {
        byte[] msg = message.getBytes();
        Log.d(TAG, "sending data");
        try {
            outputStream.write(msg);
        } catch (IOException exception) {
            exception.printStackTrace();
            Log.d(TAG, "crashing");
        }
    }
    @Override
    public void onSensorChanged(SensorEvent event) {
        float originX = 0;
        float originY = 0;
        float originZ = 0;
        float x = event.values[0];
        float y = event.values[1];
        float z = event.values[2];
        if (!initialized) {
            originX = x;
            originY = y;
            originZ = z;
            initialized = true;
        } else {
            if (originX - x >= 3) {
                //text.setText("reached X");
            } else if (originY - y >= 3) {
                //text.setText("reached Y ");
            } else if (originZ - z >= 3) {
                //text.setText("reached Z");
            }
        }
    }
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
}