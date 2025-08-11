package com.android.msckfs.imuProcessing;

import static com.android.msckfs.utils.MathUtils.NANOSECOND_TO_SECOND;

import org.ejml.simple.SimpleMatrix;

public class ImuMessage {
    public final SimpleMatrix angularVelocity; // vec3
    public final SimpleMatrix linearAcceleration; // vec3

    public final double time;

    public ImuMessage(long timestamp, float[] angularVelocity, float[] linearAcceleration) {
        this.time = timestamp * NANOSECOND_TO_SECOND;
        this.angularVelocity = new SimpleMatrix(angularVelocity);
        this.linearAcceleration = new SimpleMatrix(linearAcceleration);
    }

    @Override
    public String toString() {
        return "ImuMessage{" +
                "angularVelocity=" + angularVelocity +
                ", linearAcceleration=" + linearAcceleration +
                ", timestamp=" + time +
                '}';
    }
}
