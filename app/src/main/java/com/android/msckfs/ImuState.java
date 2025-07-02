package com.android.msckfs;

import org.apache.commons.numbers.quaternion.Quaternion;
import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

/**
 *
 */
public class ImuState extends State {

    // TODO: Instantialize where?

    /**
     Transformation offset from the IMU frame to the body frame.
     The transformation takes a vector from the IMU frame to the
     body frame. The z axis of the body frame should point upwards.
     Normally, this transform should be identity.
     **/
    private Isometry tImu;

    public final SimpleMatrix rImuCam;
    public final SimpleMatrix tCamImu; // rotation from imu to cam0, translation from cam0 to imu


    //public final static Vector gravity;

    public Quaternion orientation;

    public SimpleMatrix gyroBias = new DMatrixRMaj(3,1);
    public SimpleMatrix accBias = new DMatrixRMaj(3,1);

    // Velocity of the IMU.
    public SimpleMatrix velocity = new DMatrixRMaj(3,1);
    public SimpleMatrix position = new DMatrixRMaj(3,1);

    public static final DMatrixRMaj GRAVITY = new DMatrixRMaj(new double[]{0,0,-9.81});

    public static int nextId; //TODO: initialize? // TODO: what this for?
    public ImuState(int id, long timestamp) {
        super(id, timestamp);
    }
}
