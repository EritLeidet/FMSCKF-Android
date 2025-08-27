package com.msckf.lib.msckfs;

import org.ejml.simple.SimpleMatrix;

public class CamState extends State {
    public CamState(int id, double time) {
        super(id, time);
    }

    public SimpleMatrix orientation; // Quaternion
    public SimpleMatrix position = new SimpleMatrix(3,1); // vec3d

    public SimpleMatrix orientationNull = new SimpleMatrix(new double[]{0,0,0,1}); // Quaternion
    public  SimpleMatrix positionNull = new SimpleMatrix(3,1);

}
