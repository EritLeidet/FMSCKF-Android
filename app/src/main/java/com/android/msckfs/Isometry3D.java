package com.android.msckfs;

import static org.ejml.dense.row.CommonOps_DDRM.identity;
import static org.ejml.dense.row.CommonOps_DDRM.insert;

import org.ejml.data.DMatrixRMaj;

/**
 * 3d rigid transform.
 * https://github.com/rohiitb/msckf_vio_python/blob/main/utils.py#L137
 */
public class Isometry3D {

    public DMatrixRMaj R; // matrix

    public DMatrixRMaj t; // vector
    public Isometry3D(DMatrixRMaj R, DMatrixRMaj t) {
        this.R = R;
        this.t = t;
    }

    public DMatrixRMaj matrix() {
        DMatrixRMaj m = identity(4);
        insert(R, m, 0, 0);
        insert(t, m, 0, 3);
        return m;

    }

    public Isometry3D inverse(){
        return null; //TODO
    }

    public Isometry3D mult(Isometry3D i){
        return null; //TODO
    }

}
