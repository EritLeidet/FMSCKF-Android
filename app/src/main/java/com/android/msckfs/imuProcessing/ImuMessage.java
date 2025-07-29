package com.android.msckfs.imuProcessing;

import org.ejml.simple.SimpleMatrix;

public class ImuMessage {

    public final float[] angularVelocity; // vec3
    public final float[] linearAcceleration; // vec3

    public final long timestamp; // unix time

    public ImuMessage(long timestamp, float[] angularVelocity, float[] linearAcceleration) {
        this.angularVelocity = angularVelocity;
        this.linearAcceleration = linearAcceleration;
        this.timestamp = timestamp;
    }
}
