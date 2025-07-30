package com.android.msckfs.utils;

import static org.ejml.dense.row.CommonOps_DDRM.identity;
import static org.ejml.dense.row.CommonOps_DDRM.insert;

import org.ejml.simple.SimpleMatrix;

/**
 * 3d rigid transform.
 * https://github.com/rohiitb/msckf_vio_python/blob/main/utils.py#L137
 */
public class Isometry3D {

    public SimpleMatrix R; // matrix

    public SimpleMatrix t; // vector
    public Isometry3D(SimpleMatrix R, SimpleMatrix t) {
        assert(R.getNumRows() == R.getNumCols() && R.getNumRows() == 3);
        assert(t.isVector() && t.getNumRows() == 3);
        this.R = R;
        this.t = t;
    }

    public SimpleMatrix matrix() {
        SimpleMatrix m = SimpleMatrix.identity(4);
        m.insertIntoThis(0,0,R);
        m.insertIntoThis(0,3,t);
        return m;

    }

    public Isometry3D inverse(){
        return null; //TODO
    }

    public Isometry3D mult(Isometry3D i){
        return null; //TODO
    }

}
