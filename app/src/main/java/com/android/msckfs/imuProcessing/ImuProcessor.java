package com.android.msckfs.imuProcessing;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

import java.util.function.Consumer;

public class ImuProcessor implements SensorEventListener {

    private SensorManager mSensorManager;

    private static final String tag = "MotionPublisher";
    private Sensor mGyro;
    private Sensor mAcc;
    private float[] angVel = null; // most recent angular velocity reading
    private float[] linAcc = null;// most recent linear acceleration reading

    //private Thread publishingJob;


    private Consumer<ImuMessage> callback;

    public ImuProcessor(Context context, Consumer<ImuMessage> callback) {
        this.callback = callback;
        mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);

        mGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mAcc = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        // TODO: what is a reasonable sensor delay for vr?
        mSensorManager.registerListener(this, mGyro, SensorManager.SENSOR_DELAY_NORMAL);
        mSensorManager.registerListener(this, mAcc, SensorManager.SENSOR_DELAY_NORMAL);



        //publishingJob = new PublishingJob();
        //publishingJob.start();
        Log.i(tag, "Publishing Job started");
    }




    /*
    public void onCreate(Bundle bundle) {
        super.onCreate(bundle);
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        mGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mAcc = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        publishingJob = new PublishingJob();
        publishingJob.start();

    }

     */



    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if (sensorEvent.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            angVel = sensorEvent.values;
        } else if (sensorEvent.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
            linAcc = sensorEvent.values;
        }

        if (angVel != null && linAcc != null) {
            // TODO: reset angVel and linAcc to null?
            callback.accept(new ImuMessage(sensorEvent.timestamp, angVel, linAcc));
            // Log.i(tag, String.format("angVel: %s, linAcc: %s", Arrays.toString(angVel), Arrays.toString(linAcc)));
        }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }



    public void stop() {
        //if (publishingJob != null) publishingJob.interrupt();
        mSensorManager.unregisterListener(this); // not needed? In no example code.
    }


    /*

    private class PublishingJob extends Thread {

        @Override
        public void run() {
            Log.i(tag, "run()");
            try {
                while (!this.isInterrupted()) {
                    Thread.sleep(4000); // TODO: how long sleep?
                    if (angVel != null) Log.i(tag, Arrays.toString(angVel) ); // TODO async access?
                    if (linAcc != null) Log.i(tag, Arrays.toString(linAcc)); // TODO async access?

                }

            } catch (InterruptedException ex) {
                Log.i(tag, "Finished publishing IMU");
            }

        }
    }

     */
}