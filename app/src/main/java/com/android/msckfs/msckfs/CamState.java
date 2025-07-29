package com.android.msckfs.msckfs;

import org.ejml.simple.SimpleMatrix;

public class CamState extends State {
    public CamState(int id, long timestamp) {
        super(id, timestamp);
    }


    public SimpleMatrix orientation; // Quaternion
    public SimpleMatrix position; // vec3d

    // TODO: what are these for? Does tutorial have them too?
    public final SimpleMatrix orientationNull; // Quaternion
    public final SimpleMatrix positionNull;

}
