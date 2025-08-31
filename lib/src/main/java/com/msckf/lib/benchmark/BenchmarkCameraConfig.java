package com.msckf.lib.benchmark;

public class BenchmarkCameraConfig {
    String type;
    String description;

    float frequency;

    String camera_model;

    String distortion_model;
    CameraInstrinsicParameters intrinsic;
    CameraExtrinsicParameters extrinsic; // camera to body transform

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public String getDescription() {
        return description;
    }

    public void setDescription(String description) {
        this.description = description;
    }

    public float getFrequency() {
        return frequency;
    }

    public void setFrequency(float frequency) {
        this.frequency = frequency;
    }

    public String getCamera_model() {
        return camera_model;
    }

    public void setCamera_model(String camera_model) {
        this.camera_model = camera_model;
    }

    public String getDistortion_model() {
        return distortion_model;
    }

    public void setDistortion_model(String distortion_model) {
        this.distortion_model = distortion_model;
    }

    public CameraInstrinsicParameters getIntrinsic() {
        return intrinsic;
    }

    public void setIntrinsic(CameraInstrinsicParameters intrinsic) {
        this.intrinsic = intrinsic;
    }

    public CameraExtrinsicParameters getExtrinsic() {
        return extrinsic;
    }

    public void setExtrinsic(CameraExtrinsicParameters extrinsic) {
        this.extrinsic = extrinsic;
    }
}
