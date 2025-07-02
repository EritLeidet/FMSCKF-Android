package com.android.msckfs;

import org.apache.commons.numbers.quaternion.Quaternion;
import org.ejml.simple.SimpleMatrix;

public class CamState extends State {
    public CamState(int id, long timestamp) {
        super(id, timestamp);
    }


    public Quaternion orientation; // vec4d
    public SimpleMatrix position; // vec3d

    // TODO: what are these for? Does tutorial have them too?
    public Quaternion orientationNull;
    SimpleMatrix positionNull;

}
