package com.android.msckfs.msckfs;
import com.android.msckfs.utils.Isometry3D;

import org.ejml.simple.SimpleMatrix;

/**
 *
 */
public class ImuState extends State {


    /**
     Transformation offset from the IMU frame to the body frame.
     The transformation takes a vector from the IMU frame to the
     body frame. The z axis of the body frame should point upwards.
     Normally, this transform should be identity.
     **/
    public static Isometry3D tImuBody = new Isometry3D(SimpleMatrix.identity(3), new SimpleMatrix(3,1));

    //public final static Vector gravity;

    public SimpleMatrix orientation; // Quaternion

    public SimpleMatrix orientationNull = new SimpleMatrix(new double[]{0,0,0,1}); // Quaternion

    public SimpleMatrix gyroBias = new SimpleMatrix(3,1);
    public SimpleMatrix accBias = new SimpleMatrix(3,1);

    // Velocity of the IMU.
    public SimpleMatrix velocity = new SimpleMatrix(3,1);

    public SimpleMatrix velocityNull = new SimpleMatrix(3,1);

    public SimpleMatrix position = new SimpleMatrix(3,1);
    public SimpleMatrix positionNull = new SimpleMatrix(3,1);

    // Transformation between the IMU and the left camera (cam0) // TODO: what are these for my phone? How to determine? See paper.
    public SimpleMatrix rImuCam = SimpleMatrix.identity(3);
    public SimpleMatrix tCamImu = new SimpleMatrix(3,1);

    public static SimpleMatrix GRAVITY = new SimpleMatrix(new double[]{0,0,-9.81});

    public static int nextId = 0;
    public ImuState() {  // TODO: sollte ID nicht im Konstruktor mitgegeben werden?
        super(0, -1);
    }
}
