package com.android.msckfs.imageProcessing;

import com.android.msckfs.utils.MathUtils;

import java.util.List;

public class FeatureMessage {

    public final double time; // unix time
    public final List<FeatureMeasurement> features; // TODO: type FeatureMeasurement

    public FeatureMessage(long timestamp, List<FeatureMeasurement> features) {
        this.features = features;
        this.time = timestamp * MathUtils.NANOSECOND_TO_SECOND;

    }
}
