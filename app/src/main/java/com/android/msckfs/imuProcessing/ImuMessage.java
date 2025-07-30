package com.android.msckfs.imuProcessing;

import org.ejml.simple.SimpleMatrix;

public class ImuMessage {

    // TODO: there should be a compiler error, bc. change from float[] -> SimpleMatrix
    public final SimpleMatrix angularVelocity; // vec3
    public final SimpleMatrix linearAcceleration; // vec3

    public final long timestamp; // unix time

    public ImuMessage(long timestamp, float[] angularVelocity, float[] linearAcceleration) {
        this.angularVelocity = new SimpleMatrix(angularVelocity);
        this.linearAcceleration = new SimpleMatrix(linearAcceleration);
        this.timestamp = timestamp;
    }
}
