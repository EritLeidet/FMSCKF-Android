package com.android.msckfs;

import org.apache.commons.numbers.quaternion.Quaternion;
import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

public class CamState extends State {
    public CamState(int id, long timestamp) {
        super(id, timestamp);
    }


    public Quaternion orientation; // vec4d
    public DMatrixRMaj position; // vec3d // TODO: consider making Simplematrix? Why not?

    // TODO: what are these for? Does tutorial have them too?
    public Quaternion orientationNull;
    DMatrixRMaj positionNull;

}
