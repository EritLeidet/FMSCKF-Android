package com.msckf.lib.benchmark.datatypes.cam;

public class CameraInstrinsicParameters {
    float[] camera; // fx fy cx cy
    float[] distortion; // k1 k2 r1 r2

    public float[] getCamera() {
        return camera;
    }

    public void setCamera(float[] camera) {
        this.camera = camera;
    }

    public float[] getDistortion() {
        return distortion;
    }

    public void setDistortion(float[] distortion) {
        this.distortion = distortion;
    }
}
