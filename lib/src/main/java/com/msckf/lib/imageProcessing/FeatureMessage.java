package com.msckf.lib.imageProcessing;


import com.msckf.lib.utils.MathUtils;

import java.util.List;

public class FeatureMessage {

    public final double time; // unix time
    public final List<FeatureMeasurement> features; // TODO: type FeatureMeasurement

    public FeatureMessage(double time, List<FeatureMeasurement> features) {
        this.features = features;
        this.time = time;

    }

    public FeatureMessage(long timestamp, List<FeatureMeasurement> features) {
        this(timestamp * MathUtils.NANOSECOND_TO_SECOND, features);

    }
}
