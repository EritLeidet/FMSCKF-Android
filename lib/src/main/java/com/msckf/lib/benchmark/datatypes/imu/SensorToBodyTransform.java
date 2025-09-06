package com.msckf.lib.benchmark.datatypes.imu;

public class SensorToBodyTransform {
    float[] q;
    float[] p;
    float t;

    public float getT() {
        return t;
    }

    public void setT(float t) {
        this.t = t;
    }

    public float[] getP() {
        return p;
    }

    public void setP(float[] p) {
        this.p = p;
    }

    public float[] getQ() {
        return q;
    }

    public void setQ(float[] q) {
        this.q = q;
    }
}
