package com.android.msckfs.msckfs;

import org.ejml.simple.SimpleMatrix;

public class CamState extends State {
    public CamState(int id, long timestamp) {
        super(id, timestamp);
    }

    public SimpleMatrix orientation; // Quaternion
    public SimpleMatrix position; // vec3d

    public SimpleMatrix orientationNull; // Quaternion
    public  SimpleMatrix positionNull;

}
