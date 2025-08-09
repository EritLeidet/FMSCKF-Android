package com.android.msckfs.imuProcessing;

import org.ejml.simple.SimpleMatrix;

public class ImuMessage {
    public final SimpleMatrix angularVelocity; // vec3
    public final SimpleMatrix linearAcceleration; // vec3

    public final long timestamp; // unix time

    public ImuMessage(long timestamp, float[] angularVelocity, float[] linearAcceleration) {
        this.angularVelocity = new SimpleMatrix(angularVelocity);
        this.linearAcceleration = new SimpleMatrix(linearAcceleration);
        this.timestamp = timestamp;
    }

    @Override
    public String toString() {
        return "ImuMessage{" +
                "angularVelocity=" + angularVelocity +
                ", linearAcceleration=" + linearAcceleration +
                ", timestamp=" + timestamp +
                '}';
    }
}
