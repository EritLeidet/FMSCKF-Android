package com.msckf.lib.benchmark.datatypes.cam;

public class CameraExtrinsicParameters {

    // camera to body transform
    float[] q; // x y z w
    float[] p; // [m]
    float t; // [s]

    public float[] getQ() {
        return q;
    }

    public void setQ(float[] q) {
        this.q = q;
    }

    public float[] getP() {
        return p;
    }

    public void setP(float[] p) {
        this.p = p;
    }

    public float getT() {
        return t;
    }

    public void setT(float t) {
        this.t = t;
    }
}