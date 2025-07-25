package com.android.msckfs;

import java.util.List;

public class FeatureMessage {

    public final long timestamp; // unix time
    public final List<FeatureMeasurement> features; // TODO: type FeatureMeasurement

    public FeatureMessage(long timestamp, List<FeatureMeasurement> features) {
        this.features = features;
        this.timestamp = timestamp;

    }
}
