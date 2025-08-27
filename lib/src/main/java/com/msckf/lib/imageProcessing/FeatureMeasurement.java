package com.msckf.lib.imageProcessing;

public class FeatureMeasurement {
    public final long id;

    // double instead of float, bc. represents opencv.core.point coordinate
    public final double u0; // horizontal coordinate in cam0
    public final double v0; // vertical coordinate in cam0


    public FeatureMeasurement(long id, double u0, double v0) {
        this.id = id;
        this.u0 = u0;
        this.v0 = v0;
    }

    @Override
    public String toString() {
        return "FeatureMeasurement{" +
                "id=" + id +
                ", u0=" + u0 +
                ", v0=" + v0 +
                '}';
    }
}
