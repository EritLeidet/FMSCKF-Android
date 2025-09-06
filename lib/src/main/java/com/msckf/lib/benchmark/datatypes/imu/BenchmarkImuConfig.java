package com.msckf.lib.benchmark.datatypes.imu;

public class BenchmarkImuConfig {
    String type;
    String description;
    float frequency;

    DiscreteNoiseParameters intrinsic;
    SensorToBodyTransform extrinsic;

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

    public DiscreteNoiseParameters getIntrinsic() {
        return intrinsic;
    }

    public void setIntrinsic(DiscreteNoiseParameters intrinsic) {
        this.intrinsic = intrinsic;
    }

    public SensorToBodyTransform getExtrinsic() {
        return extrinsic;
    }

    public void setExtrinsic(SensorToBodyTransform extrinsic) {
        this.extrinsic = extrinsic;
    }
}
