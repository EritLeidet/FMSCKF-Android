package com.android.msckfs;

import org.ejml.data.DMatrixRMaj;

public class ImuMessage {

    // TODO: used to be MatOfFloat, but changed bc. MSCKF Matrix type
    public final DMatrixRMaj angularVelocity; // vec3
    public final DMatrixRMaj linearAcceleration; // vec3

    public final long timestamp; // unix time

    public ImuMessage(long timestamp, DMatrixRMaj angularVelocity, DMatrixRMaj linearAcceleration) {
        this.angularVelocity = angularVelocity;
        this.linearAcceleration = linearAcceleration;
        this.timestamp = timestamp;
    }
}
